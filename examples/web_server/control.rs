//! One ordered web command stream, with bounded admission and priority stopping.
use super::DriveParams;
use robot_control::{Robot, RobotError};
use serde::Serialize;
use std::{
    collections::VecDeque,
    future::Future,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
};
use tokio::{
    sync::{Notify, oneshot},
    task::JoinHandle,
};

pub const QUEUE_CAPACITY: usize = 32;
pub type Receipt = oneshot::Receiver<Result<(), ControlError>>;

#[derive(Clone, Debug, Serialize)]
pub struct ControlError {
    pub code: &'static str,
    pub error: String,
}

impl ControlError {
    pub fn new(code: &'static str, message: impl Into<String>) -> Self {
        Self {
            code,
            error: message.into(),
        }
    }

    pub fn json(&self) -> String {
        serde_json::json!({"code": self.code, "error": self.error}).to_string()
    }
}

#[derive(Clone)]
pub struct Session(Arc<AtomicBool>);

impl Session {
    pub fn new() -> Self {
        Self(Arc::new(AtomicBool::new(true)))
    }
    fn same(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

struct Command {
    params: DriveParams,
    reply: oneshot::Sender<Result<(), ControlError>>,
}

struct Stop {
    reply: Option<oneshot::Sender<Result<(), ControlError>>>,
}

#[derive(Default)]
struct State {
    owner: Option<Session>,
    queue: VecDeque<Command>,
    stop: Option<Stop>,
    releasing: bool,
    closed: bool,
    failure: Option<ControlError>,
}

impl State {
    fn discard(&mut self, error: &ControlError) {
        for command in self.queue.drain(..) {
            let _ = command.reply.send(Err(error.clone()));
        }
        if let Some(Stop { reply: Some(reply) }) = self.stop.take() {
            let _ = reply.send(Err(error.clone()));
        }
    }
}

#[derive(Default)]
struct Shared {
    state: Mutex<State>,
    wake: Notify,
}

#[derive(Clone)]
pub struct Control(Arc<Shared>);

// Private seam for deterministic tests of commands held in flight. It is not
// a second motor controller: production only calls Robot's existing API.
pub trait CommandTarget: Send + 'static {
    fn apply(
        &self,
        params: Option<DriveParams>,
    ) -> impl Future<Output = Result<(), ControlError>> + Send;
}

impl CommandTarget for Arc<Robot> {
    async fn apply(&self, params: Option<DriveParams>) -> Result<(), ControlError> {
        let result = match params {
            Some(params) => self.drive(params.left, params.right).await,
            None => self.stop().await,
        };
        result.map_err(|error| {
            ControlError::new(
                if matches!(error, RobotError::Shutdown) {
                    "shutdown"
                } else {
                    "fault"
                },
                error.to_string(),
            )
        })
    }
}

impl Control {
    pub fn start(target: impl CommandTarget) -> (Self, JoinHandle<()>) {
        let shared = Arc::new(Shared::default());
        let worker = tokio::spawn(run(shared.clone(), target));
        (Self(shared), worker)
    }

    pub fn submit(
        &self,
        session: Option<&Session>,
        params: DriveParams,
    ) -> Result<Receipt, ControlError> {
        // Validate before claiming ownership, consuming queue capacity or
        // superseding a pending stop. Robot retains its finite-input clamping.
        if !params.left.is_finite() || !params.right.is_finite() {
            return Err(ControlError::new(
                "invalid",
                "left and right must be finite numbers",
            ));
        }
        let mut state = self.0.state.lock().unwrap();
        if let Some(error) = &state.failure {
            return Err(error.clone());
        }
        if state.closed {
            return Err(ControlError::new("shutdown", "web control is closed"));
        }
        if session.is_some_and(|s| !s.0.load(Ordering::Relaxed)) {
            return Err(ControlError::new(
                "disconnected",
                "session has disconnected",
            ));
        }
        if state.releasing
            || state
                .owner
                .as_ref()
                .is_some_and(|owner| !session.is_some_and(|s| s.same(owner)))
        {
            return Err(ControlError::new(
                "busy",
                "another WebSocket session owns control or is stopping",
            ));
        }
        let is_stop = params.left == 0.0 && params.right == 0.0;
        if !is_stop && state.queue.len() == QUEUE_CAPACITY {
            return Err(ControlError::new(
                "queue_full",
                "web command queue is full; command rejected",
            ));
        }
        if state.owner.is_none() {
            state.owner = session.cloned();
        }
        let (reply, receipt) = oneshot::channel();
        if is_stop {
            // One reserved stop slot. Supersede older queued targets instead
            // of letting them restart motion after this stop. Later accepted
            // targets run after it, in order. Repeated stops remain bounded.
            state.discard(&ControlError::new(
                "superseded",
                "command superseded by stop",
            ));
            state.stop = Some(Stop { reply: Some(reply) });
        } else {
            state.queue.push_back(Command { params, reply });
        }
        drop(state);
        self.0.wake.notify_one();
        Ok(receipt)
    }

    pub fn disconnect(&self, session: &Session) {
        let mut state = self.0.state.lock().unwrap();
        session.0.store(false, Ordering::Relaxed);
        if !state
            .owner
            .as_ref()
            .is_some_and(|owner| owner.same(session))
        {
            return;
        }
        state.owner = None;
        state.discard(&ControlError::new(
            "disconnected",
            "owner disconnected; pending commands discarded",
        ));
        state.releasing = true;
        state.stop = Some(Stop { reply: None });
        drop(state);
        self.0.wake.notify_one();
    }

    /// Call after Robot shutdown; joins never depend on an ordinary queue slot.
    pub fn close(&self) {
        let mut state = self.0.state.lock().unwrap();
        state.closed = true;
        state.owner = None;
        state.discard(&ControlError::new("shutdown", "web control is closed"));
        drop(state);
        self.0.wake.notify_one();
    }
}

enum Work {
    Drive(Command),
    Stop(Stop, bool),
}

async fn run(shared: Arc<Shared>, target: impl CommandTarget) {
    loop {
        let work = {
            let mut state = shared.state.lock().unwrap();
            if state.closed {
                return;
            }
            if let Some(stop) = state.stop.take() {
                Some(Work::Stop(stop, state.releasing))
            } else {
                state.queue.pop_front().map(Work::Drive)
            }
        };
        let Some(work) = work else {
            shared.wake.notified().await;
            continue;
        };
        // A command removed from the queue is in flight. Disconnect cannot
        // interrupt it; its priority stop runs immediately afterward. No
        // queued command from the disconnected session survives that stop.
        let (params, reply, releasing) = match work {
            Work::Drive(command) => (Some(command.params), Some(command.reply), false),
            Work::Stop(stop, releasing) => (None, stop.reply, releasing),
        };
        let result = target.apply(params).await;
        {
            let mut state = shared.state.lock().unwrap();
            if releasing {
                state.releasing = false;
            }
            if let Err(error) = &result {
                state.failure = Some(error.clone());
                state.owner = None;
                state.discard(error);
                state.closed = true;
                tracing::warn!(code = error.code, error = %error.error, "web control failed");
            }
        }
        if let Some(reply) = reply {
            let _ = reply.send(result);
        }
    }
}

pub async fn receipt_result(receipt: Receipt) -> Result<(), ControlError> {
    receipt
        .await
        .unwrap_or_else(|_| Err(ControlError::new("shutdown", "web command worker stopped")))
}
