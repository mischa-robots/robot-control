use super::*;
use actix_web::{http::StatusCode, test as http_test};
use control::{CommandTarget, DriveRequest, QUEUE_CAPACITY, Receipt};
use std::time::Duration;
use tokio::{
    sync::{mpsc, oneshot},
    task::JoinHandle,
};

struct Call {
    params: Option<DriveParams>,
    request: Option<DriveRequest>,
    finish: oneshot::Sender<Result<(), ControlError>>,
}

struct Target(mpsc::UnboundedSender<Call>);
impl CommandTarget for Target {
    async fn apply(&self, request: Option<DriveRequest>) -> Result<(), ControlError> {
        let (finish, receipt) = oneshot::channel();
        self.0
            .send(Call {
                params: request.map(|request| request.params),
                request,
                finish,
            })
            .unwrap();
        receipt.await.unwrap()
    }
}

struct Rig {
    control: Control,
    worker: JoinHandle<()>,
    calls: mpsc::UnboundedReceiver<Call>,
}

impl Rig {
    fn new() -> Self {
        let (send, calls) = mpsc::unbounded_channel();
        let (control, worker) = Control::start(Target(send));
        Self {
            control,
            worker,
            calls,
        }
    }

    async fn next(&mut self) -> Call {
        tokio::time::timeout(Duration::from_secs(5), self.calls.recv())
            .await
            .unwrap()
            .unwrap()
    }

    async fn finish(self) {
        self.control.close();
        tokio::time::timeout(Duration::from_secs(5), self.worker)
            .await
            .unwrap()
            .unwrap();
    }

    async fn after_release(&self, session: Option<&Session>, params: DriveParams) -> Receipt {
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                match self.control.submit(session, params) {
                    Ok(receipt) => return receipt,
                    Err(error) => assert_eq!(error.code, "busy"),
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .unwrap()
    }
}

fn demand(value: f32) -> DriveParams {
    DriveParams {
        left: value,
        right: -value,
    }
}

async fn accepted(receipt: Receipt) {
    tokio::time::timeout(Duration::from_secs(5), receipt_result(receipt))
        .await
        .unwrap()
        .unwrap();
}

#[tokio::test]
async fn websocket_commands_apply_in_receive_order_despite_delayed_first_command() {
    let mut rig = Rig::new();
    let actor = RobotWsActor::new(rig.control.clone());
    let first = actor.submit_text(r#"{"left":0.2,"right":-0.2}"#).unwrap();
    let blocked = rig.next().await;
    let second = actor.submit_text(r#"{"left":0.4,"right":-0.4}"#).unwrap();
    let third = actor.submit_text(r#"{"left":0.6,"right":-0.6}"#).unwrap();
    assert_eq!(blocked.params.unwrap().left, 0.2);
    assert!(rig.calls.try_recv().is_err());
    blocked.finish.send(Ok(())).unwrap();
    accepted(first).await;
    for (receipt, value) in [(second, 0.4), (third, 0.6)] {
        let call = rig.next().await;
        let params = call.params.unwrap();
        assert_eq!((params.left, params.right), (value, -value));
        call.finish.send(Ok(())).unwrap();
        accepted(receipt).await;
    }
    rig.finish().await;
}

#[tokio::test]
async fn ownership_rejects_other_clients_including_stop_and_spectator_disconnect() {
    let mut rig = Rig::new();
    let owner = Session::new();
    let spectator = Session::new();
    let first = rig.control.submit(Some(&owner), demand(0.2)).unwrap();
    for params in [demand(0.5), demand(0.0)] {
        assert_eq!(
            rig.control
                .submit(Some(&spectator), params)
                .unwrap_err()
                .code,
            "busy"
        );
        assert_eq!(rig.control.submit(None, params).unwrap_err().code, "busy");
    }
    rig.control.disconnect(&spectator);
    rig.next().await.finish.send(Ok(())).unwrap();
    accepted(first).await;
    let next = rig.control.submit(Some(&owner), demand(0.4)).unwrap();
    let call = rig.next().await;
    assert_eq!(call.params.unwrap().left, 0.4); // No spectator stop inserted.
    call.finish.send(Ok(())).unwrap();
    accepted(next).await;
    rig.finish().await;
}

#[tokio::test]
async fn disconnect_invalidates_full_queue_and_blocks_reconnect_until_stop_completes() {
    let mut rig = Rig::new();
    let owner = Session::new();
    let first = rig.control.submit(Some(&owner), demand(0.2)).unwrap();
    let blocked = rig.next().await;
    let mut queued = Vec::new();
    for _ in 0..QUEUE_CAPACITY {
        queued.push(rig.control.submit(Some(&owner), demand(0.4)).unwrap());
    }
    assert_eq!(
        rig.control
            .submit(Some(&owner), demand(0.6))
            .unwrap_err()
            .code,
        "queue_full"
    );
    rig.control.disconnect(&owner);
    rig.control.disconnect(&owner); // Repeated close/stopped callbacks are harmless.
    for receipt in queued {
        assert_eq!(
            receipt_result(receipt).await.unwrap_err().code,
            "disconnected"
        );
    }
    assert_eq!(
        rig.control
            .submit(Some(&owner), demand(0.8))
            .unwrap_err()
            .code,
        "disconnected"
    );
    let new_owner = Session::new();
    assert_eq!(
        rig.control
            .submit(Some(&new_owner), demand(0.8))
            .unwrap_err()
            .code,
        "busy"
    );
    blocked.finish.send(Ok(())).unwrap();
    accepted(first).await;
    let stop = rig.next().await;
    assert!(stop.params.is_none());
    assert_eq!(
        rig.control.submit(None, demand(0.8)).unwrap_err().code,
        "busy"
    );
    stop.finish.send(Ok(())).unwrap();
    let receipt = rig.after_release(Some(&new_owner), demand(0.8)).await;
    let call = rig.next().await;
    assert_eq!(call.params.unwrap().left, 0.8); // No old 0.4 target follows stop.
    call.finish.send(Ok(())).unwrap();
    accepted(receipt).await;
    rig.finish().await;
}

#[tokio::test]
async fn zero_stop_bypasses_overflow_and_supersedes_old_targets_without_losing_new_ones() {
    let mut rig = Rig::new();
    let first = rig.control.submit(None, demand(0.2)).unwrap();
    let blocked = rig.next().await;
    let mut queued = Vec::new();
    for _ in 0..QUEUE_CAPACITY {
        queued.push(rig.control.submit(None, demand(0.4)).unwrap());
    }
    let old_stop = rig.control.submit(None, demand(0.0)).unwrap();
    for receipt in queued {
        assert_eq!(
            receipt_result(receipt).await.unwrap_err().code,
            "superseded"
        );
    }
    let stop_receipt = rig.control.submit(None, demand(0.0)).unwrap();
    assert_eq!(
        receipt_result(old_stop).await.unwrap_err().code,
        "superseded"
    );
    let fresh = rig.control.submit(None, demand(0.8)).unwrap();
    blocked.finish.send(Ok(())).unwrap();
    accepted(first).await;
    let stop = rig.next().await;
    assert!(stop.params.is_none());
    stop.finish.send(Ok(())).unwrap();
    accepted(stop_receipt).await;
    let next = rig.next().await;
    assert_eq!(next.params.unwrap().left, 0.8);
    next.finish.send(Ok(())).unwrap();
    accepted(fresh).await;
    rig.finish().await;
}

#[tokio::test]
async fn invalid_websocket_messages_never_claim_and_release_existing_ownership() {
    let mut rig = Rig::new();
    for bad in [
        r#"{"left":0.5}"#,
        r#"{"left":"quoted\\value","right":0}"#,
        r#"{"left":1e100,"right":0}"#,
    ] {
        let actor = RobotWsActor::new(rig.control.clone());
        let error = actor.submit_text(bad).unwrap_err();
        assert_eq!(error.code, "invalid");
        let json: serde_json::Value = serde_json::from_str(&error.json()).unwrap();
        assert_eq!(json["error"], error.error);
    }
    let invalid = rig
        .control
        .submit(Some(&Session::new()), demand(f32::NAN))
        .unwrap_err();
    assert_eq!(invalid.code, "invalid");
    // No invalid request took ownership, so HTTP remains admissible.
    let http = rig.control.submit(None, demand(0.2)).unwrap();
    rig.next().await.finish.send(Ok(())).unwrap();
    accepted(http).await;
    let actor = RobotWsActor::new(rig.control.clone());
    let queued = actor.submit_text(r#"{"left":0.5,"right":0.5}"#).unwrap();
    assert_eq!(actor.submit_text("invalid").unwrap_err().code, "invalid");
    assert_eq!(
        receipt_result(queued).await.unwrap_err().code,
        "disconnected"
    );
    let stop = rig.next().await;
    assert!(stop.params.is_none());
    stop.finish.send(Ok(())).unwrap();
    let receipt = rig.after_release(None, demand(0.2)).await;
    rig.next().await.finish.send(Ok(())).unwrap();
    accepted(receipt).await;
    rig.finish().await;
}

#[tokio::test]
async fn worker_failure_rejects_pending_and_future_commands_with_escaped_diagnostic() {
    let mut rig = Rig::new();
    let first = rig.control.submit(None, demand(0.2)).unwrap();
    let call = rig.next().await;
    let pending = rig.control.submit(None, demand(0.4)).unwrap();
    call.finish
        .send(Err(ControlError::new("fault", "failed \"motor\"\nI/O")))
        .unwrap();
    for receipt in [first, pending] {
        let error = receipt_result(receipt).await.unwrap_err();
        assert_eq!(error.code, "fault");
        let json: serde_json::Value = serde_json::from_str(&error.json()).unwrap();
        assert_eq!(json["error"], "failed \"motor\"\nI/O");
    }
    assert_eq!(
        rig.control
            .submit(Some(&Session::new()), demand(0.5))
            .unwrap_err()
            .code,
        "fault"
    );
    rig.finish().await;
}

#[tokio::test]
async fn closing_dispatcher_discards_queued_commands_and_joins_in_flight_request() {
    let mut rig = Rig::new();
    let first = rig.control.submit(None, demand(0.2)).unwrap();
    let call = rig.next().await;
    let queued = rig.control.submit(None, demand(0.4)).unwrap();
    rig.control.close();
    assert_eq!(receipt_result(queued).await.unwrap_err().code, "shutdown");
    assert!(!rig.worker.is_finished());
    call.finish.send(Ok(())).unwrap();
    accepted(first).await;
    rig.finish().await;
}

#[tokio::test]
async fn http_uses_shared_queue_and_returns_json_for_invalid_busy_overflow_and_success() {
    let mut rig = Rig::new();
    let app = http_test::init_service(
        App::new()
            .app_data(web::Data::new(rig.control.clone()))
            .app_data(query_config())
            .route("/drive", web::get().to(drive_robot)),
    )
    .await;
    for query in [
        "left=no&right=0",
        "left=NaN&right=0",
        "left=0.5",
        "left=inf&right=0",
    ] {
        let request = http_test::TestRequest::get()
            .uri(&format!("/drive?{query}"))
            .to_request();
        let response = http_test::call_service(&app, request).await;
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
        let json: serde_json::Value = http_test::read_body_json(response).await;
        assert_eq!(json["code"], "invalid");
    }
    let request = http_test::TestRequest::get()
        .uri("/drive?left=0.2&right=0.3")
        .to_request();
    let (response, ()) = tokio::join!(http_test::call_service(&app, request), async {
        let call = rig.next().await;
        let params = call.params.unwrap();
        assert_eq!((params.left, params.right), (0.2, 0.3));
        call.finish.send(Ok(())).unwrap();
    });
    assert_eq!(response.status(), StatusCode::OK);
    let json: serde_json::Value = http_test::read_body_json(response).await;
    assert_eq!(json["status"], "accepted");
    let mut queued = Vec::new();
    for _ in 0..QUEUE_CAPACITY {
        queued.push(rig.control.submit(None, demand(0.4)).unwrap());
    }
    let response = http_test::call_service(
        &app,
        http_test::TestRequest::get()
            .uri("/drive?left=0.2&right=0.3")
            .to_request(),
    )
    .await;
    assert_eq!(response.status(), StatusCode::TOO_MANY_REQUESTS);
    let json: serde_json::Value = http_test::read_body_json(response).await;
    assert_eq!(json["code"], "queue_full");
    // A rejected claim must not acquire ownership. A stop can still claim it.
    let owner = Session::new();
    let stop_receipt = rig.control.submit(Some(&owner), demand(0.0)).unwrap();
    let response = http_test::call_service(
        &app,
        http_test::TestRequest::get()
            .uri("/drive?left=0&right=0")
            .to_request(),
    )
    .await;
    assert_eq!(response.status(), StatusCode::CONFLICT);
    let json: serde_json::Value = http_test::read_body_json(response).await;
    assert_eq!(json["code"], "busy");
    rig.next().await.finish.send(Ok(())).unwrap();
    accepted(stop_receipt).await;
    rig.finish().await;
}

struct RecordingMotor(mpsc::UnboundedSender<u16>);

impl MotorDriver for RecordingMotor {
    fn set_motor_speed(
        &self,
        _: Motor,
        speed: u16,
        _: bool,
    ) -> Result<(), robot_control::MotorError> {
        self.0.send(speed).unwrap();
        Ok(())
    }

    fn stop_motor(&self, _: Motor) -> Result<(), robot_control::MotorError> {
        self.0.send(0).unwrap();
        Ok(())
    }
}

#[tokio::test(start_paused = true)]
async fn real_robot_target_stops_smoothly_and_reports_shutdown() {
    let (send, mut outputs) = mpsc::unbounded_channel();
    let robot = Robot::new(
        Arc::new(Mutex::new(RecordingMotor(send))),
        RobotConfig::default(),
    )
    .unwrap();
    let (control, worker) = Control::start(robot.clone());
    let owner = Session::new();
    accepted(
        control
            .submit(
                Some(&owner),
                DriveParams {
                    left: 0.3,
                    right: 0.3,
                },
            )
            .unwrap(),
    )
    .await;
    let mut previous = 0;
    for _ in 0..3 {
        tokio::time::advance(Duration::from_millis(10)).await;
        previous = outputs.recv().await.unwrap();
        assert_eq!(outputs.recv().await.unwrap(), previous);
    }
    accepted(control.submit(Some(&owner), demand(0.0)).unwrap()).await;
    tokio::time::advance(Duration::from_millis(10)).await;
    let ramped = outputs.recv().await.unwrap();
    assert!(ramped > 0 && ramped < previous);
    assert_eq!(outputs.recv().await.unwrap(), ramped);
    robot.shutdown().await;
    let error = receipt_result(control.submit(Some(&owner), demand(0.5)).unwrap())
        .await
        .unwrap_err();
    assert_eq!(error.code, "shutdown");
    control.close();
    worker.await.unwrap();
}

#[tokio::test(start_paused = true)]
async fn queued_deadlines_keep_receipt_time_and_renew_only_an_active_lease() {
    use tokio::time::{Instant, advance};
    let mut rig = Rig::new();
    let started = Instant::now();
    let first = rig.control.submit(None, demand(0.2)).unwrap();
    let blocked = rig.next().await;
    assert!(!blocked.request.unwrap().renew);
    advance(Duration::from_millis(100)).await;
    let second = rig.control.submit(None, demand(0.4)).unwrap();
    blocked.finish.send(Ok(())).unwrap();
    accepted(first).await;
    advance(Duration::from_millis(100)).await;
    let next = rig.next().await;
    let request = next.request.unwrap();
    assert!(request.renew);
    assert_eq!(request.deadline, started + Duration::from_millis(600));
    next.finish.send(Ok(())).unwrap();
    accepted(second).await;
    advance(Duration::from_millis(399)).await;
    assert!(rig.calls.try_recv().is_err());
    advance(Duration::from_millis(1)).await;
    let stop = rig.next().await;
    assert!(stop.params.is_none());
    stop.finish.send(Ok(())).unwrap();
    rig.finish().await;
}

#[tokio::test(start_paused = true)]
async fn expiry_discards_even_unexpired_queued_renewals_and_revokes_old_session() {
    let mut rig = Rig::new();
    let owner = Session::new();
    let first = rig.control.submit(Some(&owner), demand(0.2)).unwrap();
    let blocked = rig.next().await;
    tokio::time::advance(Duration::from_millis(400)).await;
    let queued = rig.control.submit(Some(&owner), demand(0.4)).unwrap();
    tokio::time::advance(Duration::from_millis(101)).await;
    blocked.finish.send(Ok(())).unwrap();
    for receipt in [first, queued] {
        assert_eq!(receipt_result(receipt).await.unwrap_err().code, "expired");
    }
    assert_eq!(
        rig.control
            .submit(Some(&owner), demand(0.5))
            .unwrap_err()
            .code,
        "expired"
    );
    let stop = rig.next().await;
    assert!(stop.params.is_none());
    assert_eq!(
        rig.control.submit(None, demand(0.5)).unwrap_err().code,
        "busy"
    );
    stop.finish.send(Ok(())).unwrap();
    let fresh = Session::new();
    let receipt = rig.after_release(Some(&fresh), demand(0.6)).await;
    let call = rig.next().await;
    assert_eq!(call.params.unwrap().left, 0.6);
    assert!(!call.request.unwrap().renew);
    call.finish.send(Ok(())).unwrap();
    accepted(receipt).await;
    rig.finish().await;
}

#[tokio::test(start_paused = true)]
async fn already_expired_queued_command_never_reaches_target() {
    let mut rig = Rig::new();
    let stopping = rig.control.submit(None, demand(0.0)).unwrap();
    let stop = rig.next().await;
    let queued = rig.control.submit(None, demand(0.5)).unwrap();
    tokio::time::advance(Duration::from_millis(501)).await;
    stop.finish.send(Ok(())).unwrap();
    accepted(stopping).await;
    assert_eq!(receipt_result(queued).await.unwrap_err().code, "expired");
    let next = rig.next().await;
    assert!(next.params.is_none());
    next.finish.send(Ok(())).unwrap();
    rig.finish().await;
}

#[tokio::test(start_paused = true)]
async fn valid_renewal_holds_motion_but_invalid_and_spectator_traffic_does_not() {
    let mut rig = Rig::new();
    let owner = Session::new();
    for index in 0..11 {
        let receipt = rig.control.submit(Some(&owner), demand(0.4)).unwrap();
        let call = rig.next().await;
        assert_eq!(call.request.unwrap().renew, index != 0);
        call.finish.send(Ok(())).unwrap();
        accepted(receipt).await;
        tokio::time::advance(Duration::from_millis(100)).await;
    }
    tokio::time::advance(Duration::from_millis(300)).await;
    assert_eq!(
        rig.control
            .submit(Some(&Session::new()), demand(0.7))
            .unwrap_err()
            .code,
        "busy"
    );
    assert_eq!(
        rig.control
            .submit(Some(&owner), demand(f32::INFINITY))
            .unwrap_err()
            .code,
        "invalid"
    );
    assert!(parse_drive(r#"{"ping":true}"#).is_err());
    assert!(rig.calls.try_recv().is_err());
    tokio::time::advance(Duration::from_millis(100)).await;
    let stop = rig.next().await;
    assert!(stop.params.is_none());
    stop.finish.send(Ok(())).unwrap();
    assert_eq!(
        rig.control
            .submit(Some(&owner), demand(0.4))
            .unwrap_err()
            .code,
        "expired"
    );
    rig.finish().await;
}

#[tokio::test(start_paused = true)]
async fn one_shot_http_command_expires_in_robot_without_more_requests() {
    let (send, mut outputs) = mpsc::unbounded_channel();
    let robot = Robot::new(
        Arc::new(Mutex::new(RecordingMotor(send))),
        RobotConfig::default(),
    )
    .unwrap();
    let (control, worker) = Control::start(robot.clone());
    let app = http_test::init_service(
        App::new()
            .app_data(web::Data::new(control.clone()))
            .app_data(query_config())
            .route("/drive", web::get().to(drive_robot)),
    )
    .await;
    let response = http_test::call_service(
        &app,
        http_test::TestRequest::get()
            .uri("/drive?left=0.5&right=0.5")
            .to_request(),
    )
    .await;
    assert_eq!(response.status(), StatusCode::OK);
    let mut previous = 0;
    for _ in 0..5 {
        tokio::time::advance(Duration::from_millis(10)).await;
        previous = outputs.recv().await.unwrap();
        assert_eq!(outputs.recv().await.unwrap(), previous);
    }
    assert_eq!(previous, 2048);
    tokio::time::advance(Duration::from_millis(450)).await;
    let ramped = outputs.recv().await.unwrap();
    assert!(ramped > 0 && ramped < previous);
    assert_eq!(outputs.recv().await.unwrap(), ramped);
    previous = ramped;
    for _ in 0..6 {
        tokio::time::advance(Duration::from_millis(10)).await;
        let ramped = outputs.recv().await.unwrap();
        assert!(ramped <= previous && previous - ramped <= 411);
        assert_eq!(outputs.recv().await.unwrap(), ramped);
        previous = ramped;
    }
    assert_eq!(previous, 0);
    assert_eq!(robot.status().await.expired_commands, 1);
    robot.shutdown().await;
    control.close();
    worker.await.unwrap();
}

#[tokio::test(start_paused = true)]
async fn owner_disconnect_ramps_real_robot_to_zero() {
    let (send, mut outputs) = mpsc::unbounded_channel();
    let robot = Robot::new(
        Arc::new(Mutex::new(RecordingMotor(send))),
        RobotConfig::default(),
    )
    .unwrap();
    let (control, worker) = Control::start(robot.clone());
    let owner = Session::new();
    accepted(
        control
            .submit(
                Some(&owner),
                DriveParams {
                    left: 0.5,
                    right: 0.5,
                },
            )
            .unwrap(),
    )
    .await;
    for _ in 0..5 {
        tokio::time::advance(Duration::from_millis(10)).await;
        assert!(outputs.recv().await.unwrap() > 0);
        assert!(outputs.recv().await.unwrap() > 0);
    }
    control.disconnect(&owner);
    tokio::task::yield_now().await;
    assert!(outputs.try_recv().is_err());
    let mut previous = 2048;
    for _ in 0..7 {
        tokio::time::advance(Duration::from_millis(10)).await;
        let ramped = outputs.recv().await.unwrap();
        assert!(ramped <= previous && previous - ramped <= 411);
        assert_eq!(outputs.recv().await.unwrap(), ramped);
        previous = ramped;
    }
    assert_eq!(previous, 0);
    assert_eq!(robot.status().await.expired_commands, 0);
    robot.shutdown().await;
    control.close();
    worker.await.unwrap();
}
