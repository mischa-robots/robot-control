//! One ordered web command stream, with bounded admission and priority stopping.
use super::DriveParams;
use robot_control::{Robot, RobotError};
use serde::Serialize;
use std::{
    collections::VecDeque,
    future::Future,
    sync::{
        Arc, Mutex,
        atomic::{AtomicU8, Ordering},
    },
};
use tokio::{
    sync::{Notify, oneshot},
    task::JoinHandle,
    time::{Duration, Instant},
};

pub const QUEUE_CAPACITY: usize = 32;
pub const COMMAND_LIFETIME: Duration = Duration::from_millis(500);
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
pub struct Session(Arc<AtomicU8>);

impl Session {
    pub fn new() -> Self {
        Self(Arc::new(AtomicU8::new(1)))
    }
    fn same(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DriveRequest {
    pub params: DriveParams,
    pub deadline: Instant,
    pub renew: bool,
}

struct Command {
    params: DriveParams,
    deadline: Instant,
    reply: oneshot::Sender<Result<(), ControlError>>,
}

struct Stop {
    reply: Option<oneshot::Sender<Result<(), ControlError>>>,
}

#[derive(Default)]
struct State {
    owner: Option<Session>,
    deadline: Option<Instant>,
    queue: VecDeque<Command>,
    stop: Option<Stop>,
    releasing: bool,
    closed: bool,
    failure: Option<ControlError>,
}

impl State {
    fn expire(&mut self) {
        if let Some(owner) = self.owner.take() {
            owner.0.store(2, Ordering::Relaxed);
        }
        self.deadline = None;
        self.discard(&ControlError::new(
            "expired",
            "command lease expired; reconnect and submit fresh input",
        ));
        self.releasing = true;
        self.stop = Some(Stop { reply: None });
    }

    fn expire_if_due(&mut self) {
        if self
            .deadline
            .is_some_and(|deadline| deadline <= Instant::now())
        {
            self.expire();
        }
    }

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
        params: Option<DriveRequest>,
    ) -> impl Future<Output = Result<(), ControlError>> + Send;
}

impl CommandTarget for Arc<Robot> {
    async fn apply(&self, params: Option<DriveRequest>) -> Result<(), ControlError> {
        let result = match params {
            Some(request) => {
                let accepted = if request.renew {
                    self.renew_until(
                        request.params.left,
                        request.params.right,
                        request.deadline.into_std(),
                    )
                    .await
                } else {
                    self.drive_until(
                        request.params.left,
                        request.params.right,
                        request.deadline.into_std(),
                    )
                    .await
                };
                match accepted {
                    Ok(true) => Ok(()),
                    Ok(false) => {
                        return Err(ControlError::new(
                            "expired",
                            "command lease expired before admission",
                        ));
                    }
                    Err(error) => Err(error),
                }
            }
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
        let deadline = Instant::now() + COMMAND_LIFETIME;
        let mut state = self.0.state.lock().unwrap();
        state.expire_if_due();
        self.0.wake.notify_one();
        if let Some(error) = &state.failure {
            return Err(error.clone());
        }
        if state.closed {
            return Err(ControlError::new("shutdown", "web control is closed"));
        }
        if session.is_some_and(|s| s.0.load(Ordering::Relaxed) == 2) {
            return Err(ControlError::new(
                "expired",
                "session lease expired; reconnect and submit fresh input",
            ));
        }
        if session.is_some_and(|s| s.0.load(Ordering::Relaxed) == 0) {
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
            state.deadline = None;
            state.stop = Some(Stop { reply: Some(reply) });
        } else {
            state.queue.push_back(Command {
                params,
                deadline,
                reply,
            });
        }
        drop(state);
        self.0.wake.notify_one();
        Ok(receipt)
    }

    pub fn disconnect(&self, session: &Session) {
        let mut state = self.0.state.lock().unwrap();
        if session.0.load(Ordering::Relaxed) != 2 {
            session.0.store(0, Ordering::Relaxed);
        }
        if !state
            .owner
            .as_ref()
            .is_some_and(|owner| owner.same(session))
        {
            return;
        }
        state.owner = None;
        state.deadline = None;
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
        state.deadline = None;
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
            state.expire_if_due();
            if let Some(stop) = state.stop.take() {
                Some(Work::Stop(stop, state.releasing))
            } else {
                state.queue.pop_front().map(Work::Drive)
            }
        };
        let Some(work) = work else {
            let deadline = shared.state.lock().unwrap().deadline;
            tokio::select! {
                () = shared.wake.notified() => {},
                () = async { match deadline {
                    Some(deadline) => tokio::time::sleep_until(deadline).await,
                    None => std::future::pending::<()>().await,
                } } => {},
            }
            continue;
        };
        // A command removed from the queue is in flight. Disconnect cannot
        // interrupt it; its priority stop runs immediately afterward. No
        // queued command from the disconnected session survives that stop.
        let (params, reply, releasing) = match work {
            Work::Drive(command) => (
                Some(DriveRequest {
                    params: command.params,
                    deadline: command.deadline,
                    renew: shared.state.lock().unwrap().deadline.is_some(),
                }),
                Some(command.reply),
                false,
            ),
            Work::Stop(stop, releasing) => (None, stop.reply, releasing),
        };
        let result = if params.is_some_and(|request| request.deadline <= Instant::now()) {
            Err(ControlError::new(
                "expired",
                "queued command deadline elapsed",
            ))
        } else {
            let result = target.apply(params).await;
            if result.is_ok() && params.is_some_and(|request| request.deadline <= Instant::now()) {
                Err(ControlError::new(
                    "expired",
                    "command expired during admission",
                ))
            } else {
                result
            }
        };
        {
            let mut state = shared.state.lock().unwrap();
            if releasing {
                state.releasing = false;
            }
            if result.is_ok() && !state.releasing {
                state.deadline = params.map(|request| request.deadline);
            }
            if let Err(error) = &result {
                if error.code == "expired" {
                    state.expire();
                } else {
                    state.failure = Some(error.clone());
                    state.owner = None;
                    state.discard(error);
                    state.closed = true;
                    tracing::warn!(code = error.code, error = %error.error, "web control failed");
                }
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
