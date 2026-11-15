use super::*;
use actix_web::{http::StatusCode, test as http_test};
use robot_control::{MotorError, RobotOutputState};
use std::{future::pending, task::Poll, time::Duration};
use tokio::sync::{Notify, oneshot};

#[test]
fn accepts_negative_motor_factors() {
    let cli = Cli::try_parse_from([
        "web_server",
        "--host",
        "0.0.0.0",
        "--port",
        "8080",
        "--left-factor",
        "-1.0",
        "--right-factor",
        "-0.5",
    ])
    .expect("negative motor factors should be accepted");
    assert_eq!(cli.left_factor, -1.0);
    assert_eq!(cli.right_factor, -0.5);
    assert_eq!(cli.port, 8080);
}

#[derive(Default)]
struct TestDriver {
    stops: std::sync::Mutex<Vec<Motor>>,
    fail_stop: bool,
    fail_write: bool,
    write_failed: Arc<Notify>,
    write_applied: Arc<Notify>,
}

impl MotorDriver for TestDriver {
    fn set_motor_speed(&self, _: Motor, _: u16, _: bool) -> Result<(), MotorError> {
        if self.fail_write {
            self.write_failed.notify_one();
            Err(MotorError::I2c("injected output failure".into()))
        } else {
            self.write_applied.notify_one();
            Ok(())
        }
    }

    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        self.stops.lock().unwrap().push(motor);
        if self.fail_stop && motor == Motor::Motor1 {
            Err(MotorError::I2c("injected cleanup failure".into()))
        } else {
            Ok(())
        }
    }
}

fn fixture(driver: TestDriver) -> (Arc<Robot>, Arc<Mutex<TestDriver>>) {
    let driver = Arc::new(Mutex::new(driver));
    let robot = Robot::new(driver.clone(), RobotConfig::default()).unwrap();
    (robot, driver)
}

async fn assert_pending(future: std::pin::Pin<&mut impl Future>) {
    let mut future = future;
    std::future::poll_fn(|cx| {
        assert!(future.as_mut().poll(cx).is_pending());
        Poll::Ready(())
    })
    .await;
}

async fn assert_cleaned_up(robot: &Robot, driver: &Mutex<TestDriver>) {
    let status = robot.status().await;
    assert!(status.cleanup_complete);
    assert_eq!(status.phase, RobotPhase::Shutdown);
    assert_eq!(status.output, RobotOutputState::Stopped);
    assert!(robot.drive(0.5, 0.5).await.is_err());
    assert_eq!(
        *driver.lock().await.stops.lock().unwrap(),
        [Motor::Motor1, Motor::Motor2, Motor::Motor3, Motor::Motor4]
    );
}

#[tokio::test]
async fn startup_errors_wait_for_cleanup_and_keep_original_error() {
    tokio::time::timeout(Duration::from_secs(5), async {
        for kind in [io::ErrorKind::AddrInUse, io::ErrorKind::PermissionDenied] {
            let (robot, driver) = fixture(TestDriver::default());
            // Model hardware still owned by another operation. Cleanup cannot
            // finish until it gets the board, even with surviving robot clones.
            let held = driver.lock().await;
            let operation = async { Err(io::Error::new(kind, "injected startup failure")) };
            let cleanup = with_robot_cleanup(&robot, operation);
            tokio::pin!(cleanup);
            assert_pending(cleanup.as_mut()).await;
            assert_eq!(robot.status().await.phase, RobotPhase::ShuttingDown);
            assert!(!robot.status().await.cleanup_complete);
            assert!(robot.drive(0.5, 0.5).await.is_err());
            assert!(held.stops.lock().unwrap().is_empty());
            drop(held);
            let error = cleanup.await.unwrap_err();
            assert_eq!(error.kind(), kind);
            assert_eq!(error.to_string(), "injected startup failure");
            assert_cleaned_up(&robot, &driver).await;
        }
    })
    .await
    .unwrap();
}

#[tokio::test]
async fn server_completion_and_failure_both_clean_up() {
    for failed in [false, true] {
        let (robot, driver) = fixture(TestDriver::default());
        let server = async move {
            if failed {
                Err(io::Error::new(
                    io::ErrorKind::ConnectionAborted,
                    "injected serve failure",
                ))
            } else {
                Ok(())
            }
        };
        let result = with_robot_cleanup(
            &robot,
            serve_until_exit(&robot, server, pending(), || async {
                panic!("completed server needs no stop")
            }),
        )
        .await;
        if failed {
            let error = result.unwrap_err();
            assert_eq!(error.kind(), io::ErrorKind::ConnectionAborted);
            assert_eq!(error.to_string(), "injected serve failure");
        } else {
            result.unwrap();
        }
        assert_cleaned_up(&robot, &driver).await;
    }
}

#[tokio::test(start_paused = true)]
async fn signal_and_signal_errors_stop_robot_before_joining_server() {
    tokio::time::timeout(Duration::from_secs(5), async {
        for signal_failed in [false, true] {
            let (robot, driver) = fixture(TestDriver::default());
            let applied = driver.lock().await.write_applied.clone();
            robot.drive(0.5, 0.5).await.unwrap();
            let (stop_tx, stop_rx) = oneshot::channel();
            let (done_tx, done_rx) = oneshot::channel();
            let server = async {
                stop_rx.await.unwrap();
                done_tx.send(()).unwrap();
                Ok(())
            };
            let signal = async {
                // Deliver the event only after the mock has applied motion.
                applied.notified().await;
                if signal_failed {
                    Err(io::Error::other("injected signal failure"))
                } else {
                    Ok(()) // Inject Ctrl-C/SIGTERM delivery, not a real OS signal.
                }
            };
            let stop = || async {
                assert_cleaned_up(&robot, &driver).await;
                stop_tx.send(()).unwrap();
                // Requires continued polling of the server during stop.
                done_rx.await.unwrap();
            };
            let result =
                with_robot_cleanup(&robot, serve_until_exit(&robot, server, signal, stop)).await;
            if signal_failed {
                assert_eq!(result.unwrap_err().to_string(), "injected signal failure");
            } else {
                result.unwrap();
            }
            assert_cleaned_up(&robot, &driver).await;
        }
    })
    .await
    .unwrap();
}

#[tokio::test]
async fn cleanup_failure_is_retained_without_replacing_server_error() {
    for server_failed in [false, true] {
        let (robot, driver) = fixture(TestDriver {
            fail_stop: true,
            ..TestDriver::default()
        });
        let result = with_robot_cleanup(&robot, async {
            if server_failed {
                Err(io::Error::new(
                    io::ErrorKind::AddrInUse,
                    "original bind failure",
                ))
            } else {
                Ok(())
            }
        })
        .await;
        let error = result.unwrap_err();
        if server_failed {
            assert_eq!(error.kind(), io::ErrorKind::AddrInUse);
            assert_eq!(error.to_string(), "original bind failure");
        } else {
            assert!(error.to_string().contains("injected cleanup failure"));
        }
        let status = robot.status().await;
        assert!(status.cleanup_complete);
        assert_eq!(status.phase, RobotPhase::Faulted);
        assert_eq!(status.output, RobotOutputState::Unknown);
        assert!(
            status
                .cleanup_error
                .unwrap()
                .contains("injected cleanup failure")
        );
        assert!(robot.drive(0.5, 0.5).await.is_err());
        assert_eq!(driver.lock().await.stops.lock().unwrap().len(), 4);
    }
}

#[tokio::test]
async fn health_reports_running_shutting_down_and_shutdown() {
    let (robot, driver) = fixture(TestDriver::default());
    let app = http_test::init_service(
        App::new()
            .app_data(web::Data::new(robot.clone()))
            .route("/health", web::get().to(health_check)),
    )
    .await;
    let response = http_test::call_service(
        &app,
        http_test::TestRequest::get().uri("/health").to_request(),
    )
    .await;
    assert_eq!(response.status(), StatusCode::OK);
    let body: serde_json::Value = http_test::read_body_json(response).await;
    assert_eq!(body["phase"], "Running");
    assert_eq!(body["output"], "Unknown");
    assert_eq!(body["cleanup_complete"], false);
    assert!(body["fault"].is_null());

    let held = driver.lock().await;
    let cleanup = with_robot_cleanup(&robot, async { Ok(()) });
    tokio::pin!(cleanup);
    assert_pending(cleanup.as_mut()).await;
    let response = http_test::call_service(
        &app,
        http_test::TestRequest::get().uri("/health").to_request(),
    )
    .await;
    assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    let body: serde_json::Value = http_test::read_body_json(response).await;
    assert_eq!(body["phase"], "ShuttingDown");
    assert_eq!(body["cleanup_complete"], false);
    drop(held);
    cleanup.await.unwrap();

    let response = http_test::call_service(
        &app,
        http_test::TestRequest::get().uri("/health").to_request(),
    )
    .await;
    assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    let body: serde_json::Value = http_test::read_body_json(response).await;
    assert_eq!(body["phase"], "Shutdown");
    assert_eq!(body["output"], "Stopped");
    assert_eq!(body["cleanup_complete"], true);
}

#[tokio::test(start_paused = true)]
async fn health_reports_background_fault_even_after_successful_cleanup() {
    let driver = TestDriver {
        fail_write: true,
        ..TestDriver::default()
    };
    let failed = driver.write_failed.clone();
    let (robot, _) = fixture(driver);
    let app = http_test::init_service(
        App::new()
            .app_data(web::Data::new(robot.clone()))
            .route("/health", web::get().to(health_check)),
    )
    .await;
    robot.drive(0.5, 0.5).await.unwrap();
    // Paused time advances to the first controller tick; no hardware or wall sleep.
    tokio::time::timeout(Duration::from_secs(5), failed.notified())
        .await
        .unwrap();
    let response = http_test::call_service(
        &app,
        http_test::TestRequest::get().uri("/health").to_request(),
    )
    .await;
    assert_eq!(response.status(), StatusCode::SERVICE_UNAVAILABLE);
    let body: serde_json::Value = http_test::read_body_json(response).await;
    assert_eq!(body["phase"], "Faulted");
    assert!(
        body["fault"]
            .as_str()
            .unwrap()
            .contains("injected output failure")
    );
    let error = with_robot_cleanup(&robot, async { Ok(()) })
        .await
        .unwrap_err();
    assert!(error.to_string().contains("injected output failure"));
    assert_eq!(robot.status().await.output, RobotOutputState::Stopped);
}
