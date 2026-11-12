use std::sync::{
    Condvar, Mutex as StdMutex,
    atomic::{AtomicBool, Ordering},
};

use tokio::sync::Notify;

use super::*;

const MOTORS: [Motor; 4] = [Motor::Motor1, Motor::Motor2, Motor::Motor3, Motor::Motor4];

#[derive(Debug, Clone, PartialEq, Eq)]
enum Event {
    Speed(Motor, u16, bool),
    Cleanup,
    StopAttempt(Motor),
}

// A synchronous driver call really stays in flight until the test releases it.
// Notify is the async entry barrier; the condvar holds the executor's I/O thread.
#[derive(Default)]
struct Gate {
    claimed: AtomicBool,
    entered: Notify,
    released: StdMutex<bool>,
    wake: Condvar,
}

impl Gate {
    fn block_once(&self) {
        if self.claimed.swap(true, Ordering::SeqCst) {
            return;
        }
        self.entered.notify_one();
        let mut released = self.released.lock().unwrap();
        while !*released {
            released = self.wake.wait(released).unwrap();
        }
    }

    fn release(&self) {
        *self.released.lock().unwrap() = true;
        self.wake.notify_all();
    }
}

// Release blocked synchronous I/O even if an assertion or deadlock guard fails.
struct ReleaseOnDrop(Arc<Gate>);

impl Drop for ReleaseOnDrop {
    fn drop(&mut self) {
        self.0.release();
    }
}

#[derive(Default)]
struct TestDriver {
    events: Arc<StdMutex<Vec<Event>>>,
    write_gate: Option<Arc<Gate>>,
    cleanup_gate: Option<Arc<Gate>>,
    cleanup_finished: Arc<Notify>,
    fail_stop: bool,
    fail_write: Option<Motor>,
    fail_zero_stop: bool,
    failure_detail: Option<String>,
    panic_write: bool,
}

impl MotorDriver for TestDriver {
    fn set_motor_speed(&self, motor: Motor, speed: u16, forward: bool) -> Result<(), MotorError> {
        if let Some(gate) = &self.write_gate {
            gate.block_once();
        }
        self.events
            .lock()
            .unwrap()
            .push(Event::Speed(motor, speed, forward));
        assert!(!self.panic_write, "injected adjustment panic");
        if self.fail_write == Some(motor) {
            return Err(MotorError::I2c(
                self.failure_detail
                    .clone()
                    .unwrap_or_else(|| "injected write failure".into()),
            ));
        }
        Ok(())
    }

    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        self.events.lock().unwrap().push(Event::StopAttempt(motor));
        if (self.fail_stop && motor == Motor::Motor2)
            || (self.fail_zero_stop && motor == Motor::Motor4)
        {
            Err(MotorError::I2c(
                self.failure_detail
                    .clone()
                    .unwrap_or_else(|| "injected stop failure".into()),
            ))
        } else {
            Ok(())
        }
    }

    fn stop_all(&self) -> Result<(), MotorError> {
        self.events.lock().unwrap().push(Event::Cleanup);
        if let Some(gate) = &self.cleanup_gate {
            gate.block_once();
        }
        let mut failure = None;
        for motor in MOTORS {
            if let Err(error) = self.stop_motor(motor) {
                failure = Some(error);
            }
        }
        self.cleanup_finished.notify_one();
        failure.map_or(Ok(()), Err)
    }
}

fn create_robot(driver: TestDriver) -> (Arc<Robot>, Arc<Mutex<TestDriver>>) {
    let driver = Arc::new(Mutex::new(driver));
    let robot = Robot::new(
        driver.clone(),
        RobotConfig {
            left_motor: Motor::Motor4,
            right_motor: Motor::Motor1,
            ..RobotConfig::default()
        },
    )
    .unwrap();
    (robot, driver)
}

async fn assert_pending<F: std::future::Future>(mut future: std::pin::Pin<&mut F>) {
    std::future::poll_fn(|cx| {
        assert!(future.as_mut().poll(cx).is_pending());
        std::task::Poll::Ready(())
    })
    .await;
}

fn assert_final_cleanup(events: &[Event]) {
    assert_eq!(
        events
            .iter()
            .filter(|event| **event == Event::Cleanup)
            .count(),
        1
    );
    let cleanup = events
        .iter()
        .position(|event| *event == Event::Cleanup)
        .unwrap();
    assert_eq!(&events[cleanup + 1..], MOTORS.map(Event::StopAttempt));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn shutdown_joins_in_flight_write_before_final_stop() {
    tokio::time::timeout(Duration::from_secs(5), async {
        let gate = Arc::new(Gate::default());
        let _release = ReleaseOnDrop(gate.clone());
        let driver = TestDriver {
            write_gate: Some(gate.clone()),
            ..TestDriver::default()
        };
        let events = driver.events.clone();
        let (robot, _driver) = create_robot(driver);
        robot.drive(0.25, -0.25).await.unwrap();
        gate.entered.notified().await;

        let mut first = Box::pin(robot.shutdown());
        let mut second = Box::pin(robot.shutdown());
        assert_pending(first.as_mut()).await;
        assert_pending(second.as_mut()).await;
        assert!(matches!(
            robot.drive(-0.5, 0.5).await,
            Err(RobotError::Shutdown)
        ));
        assert!(matches!(robot.stop().await, Err(RobotError::Shutdown)));
        assert!(!events.lock().unwrap().contains(&Event::Cleanup));

        gate.release();
        tokio::join!(first, second);
        let completed = events.lock().unwrap().clone();
        assert!(
            completed
                .iter()
                .any(|event| matches!(event, Event::Speed(_, 1.., _)))
        );
        assert_final_cleanup(&completed);
        // Both callers joined the only writer before the final stop. A repeated
        // shutdown cannot restart adjustment or run a second cleanup sequence.
        robot.shutdown().await;
        tokio::task::yield_now().await;
        assert_eq!(*events.lock().unwrap(), completed);
        assert!(robot.cleanup.lock().await.is_none());
    })
    .await
    .expect("shutdown did not join the in-flight write");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn cancelled_shutdown_waiter_preserves_cleanup_for_other_callers() {
    tokio::time::timeout(Duration::from_secs(5), async {
        let gate = Arc::new(Gate::default());
        let _release = ReleaseOnDrop(gate.clone());
        let driver = TestDriver {
            cleanup_gate: Some(gate.clone()),
            ..TestDriver::default()
        };
        let events = driver.events.clone();
        let (robot, _driver) = create_robot(driver);
        let mut first = Box::pin(robot.shutdown());
        assert_pending(first.as_mut()).await;
        // Drop the actual pinned future, not just a Pin<&mut _> reference.
        drop(first);
        gate.entered.notified().await;

        let mut second = Box::pin(robot.shutdown());
        let mut third = Box::pin(robot.shutdown());
        assert_pending(second.as_mut()).await;
        assert_pending(third.as_mut()).await;
        assert!(matches!(
            robot.drive(0.5, 0.5).await,
            Err(RobotError::Shutdown)
        ));
        gate.release();
        tokio::join!(second, third);
        assert_final_cleanup(&events.lock().unwrap());
    })
    .await
    .expect("a cancelled waiter lost cleanup");
}

#[tokio::test(start_paused = true)]
async fn last_handle_drop_cleans_up_with_external_driver_reference() {
    tokio::time::timeout(Duration::from_secs(5), async {
        let driver = TestDriver::default();
        let events = driver.events.clone();
        let finished = driver.cleanup_finished.clone();
        let (robot, external_driver) = create_robot(driver);
        let public = Arc::downgrade(&robot);
        let state = Arc::downgrade(&robot.state);
        let last = robot.clone();
        drop(robot);
        assert!(public.upgrade().is_some());
        last.drive(0.25, 0.25).await.unwrap();
        tokio::task::yield_now().await;
        tokio::time::advance(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;
        assert!(
            events
                .lock()
                .unwrap()
                .iter()
                .any(|event| matches!(event, Event::Speed(_, 1.., _)))
        );

        drop(last);
        assert!(public.upgrade().is_none(), "task retained the public Robot");
        finished.notified().await;
        // On this single-thread runtime, cleanup finishes its synchronous stop
        // and drops its state before this notified test can run again.
        assert!(
            state.upgrade().is_none(),
            "cleanup retained its internal state"
        );
        assert_eq!(Arc::strong_count(&external_driver), 1);
        let completed = events.lock().unwrap().clone();
        assert_final_cleanup(&completed);
        tokio::time::advance(Duration::from_secs(1)).await;
        tokio::task::yield_now().await;
        assert_eq!(*events.lock().unwrap(), completed);
    })
    .await
    .expect("last-handle drop retained the adjustment task");
}

#[tokio::test]
async fn cancelled_shutdown_and_last_handle_drop_still_finish_cleanup() {
    tokio::time::timeout(Duration::from_secs(5), async {
        let driver = TestDriver::default();
        let events = driver.events.clone();
        let finished = driver.cleanup_finished.clone();
        let (robot, external_driver) = create_robot(driver);
        let board = external_driver.lock().await;
        let mut shutdown = Box::pin(robot.shutdown());
        assert_pending(shutdown.as_mut()).await;
        drop(shutdown);
        let public = Arc::downgrade(&robot);
        drop(robot);
        assert!(public.upgrade().is_none());
        drop(board);
        finished.notified().await;
        assert_final_cleanup(&events.lock().unwrap());
    })
    .await
    .expect("cleanup depended on a surviving shutdown caller");
}

#[tokio::test(start_paused = true)]
async fn shutdown_discards_adjustment_waiting_for_board() {
    tokio::time::timeout(Duration::from_secs(5), async {
        let driver = TestDriver::default();
        let events = driver.events.clone();
        let (robot, external_driver) = create_robot(driver);
        let board = external_driver.lock().await;
        robot.drive(0.25, 0.25).await.unwrap();
        tokio::task::yield_now().await;
        tokio::time::advance(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;
        let mut shutdown = Box::pin(robot.shutdown());
        assert_pending(shutdown.as_mut()).await;
        drop(board);
        shutdown.await;
        let completed = events.lock().unwrap().clone();
        assert_final_cleanup(&completed);
        assert!(
            !completed
                .iter()
                .any(|event| matches!(event, Event::Speed(..)))
        );
        tokio::time::advance(Duration::from_secs(1)).await;
        tokio::task::yield_now().await;
        assert_eq!(*events.lock().unwrap(), completed);
    })
    .await
    .expect("shutdown did not finish after the board was released");
}

#[tokio::test]
async fn stop_failure_does_not_prevent_cleanup_completion() {
    tokio::time::timeout(Duration::from_secs(5), async {
        let driver = TestDriver {
            fail_stop: true,
            ..TestDriver::default()
        };
        let events = driver.events.clone();
        let (robot, _driver) = create_robot(driver);
        tokio::join!(robot.shutdown(), robot.shutdown());
        robot.shutdown().await;
        assert_final_cleanup(&events.lock().unwrap());
        assert!(matches!(
            robot.drive(0.5, 0.5).await,
            Err(RobotError::Motor(MotorError::Pwm(message))) if message.contains("injected stop failure")
        ));
        let status = robot.status().await;
        assert_eq!(status.phase, RobotPhase::Faulted);
        assert_eq!(status.output, RobotOutputState::Unknown);
        assert!(status.cleanup_complete);
        assert!(status.fault.is_none());
        assert!(status.cleanup_error.unwrap().contains("injected stop failure"));
    })
    .await
    .expect("stop failure prevented shutdown completion");
}

#[tokio::test(start_paused = true)]
async fn left_and_right_failures_latch_and_stop_without_later_drive_writes() {
    for (motor, side, attempted) in [(Motor::Motor4, "left", 1), (Motor::Motor1, "right", 2)] {
        for fail_stop in [false, true] {
            let driver = TestDriver {
                fail_write: Some(motor),
                fail_stop,
                ..TestDriver::default()
            };
            let events = driver.events.clone();
            let finished = driver.cleanup_finished.clone();
            let (robot, _driver) = create_robot(driver);
            assert_eq!(robot.status().await.phase, RobotPhase::Running);
            robot.drive(0.25, -0.25).await.unwrap();
            // Acceptance has not performed any hardware I/O or confirmed zero.
            assert!(events.lock().unwrap().is_empty());
            assert_eq!(robot.status().await.output, RobotOutputState::Unknown);
            tokio::time::timeout(Duration::from_secs(5), finished.notified())
                .await
                .unwrap();
            robot.shutdown().await;
            let status = robot.status().await;
            assert_eq!(status.phase, RobotPhase::Faulted);
            assert!(status.cleanup_complete);
            assert!(
                status
                    .fault
                    .as_ref()
                    .unwrap()
                    .contains(&format!("{side} motor {motor:?}"))
            );
            assert!(
                status
                    .fault
                    .as_ref()
                    .unwrap()
                    .contains("injected write failure")
            );
            assert_eq!(status.cleanup_error.is_some(), fail_stop);
            assert_eq!(
                status.output,
                if fail_stop {
                    RobotOutputState::Unknown
                } else {
                    RobotOutputState::Stopped
                }
            );
            assert_eq!(robot.state.control.lock().await.targets, (0.0, 0.0));
            for _ in 0..3 {
                assert!(matches!(robot.drive(0.5, 0.5).await,
                    Err(RobotError::Motor(MotorError::Pwm(message))) if message.contains("injected write failure")));
                assert!(matches!(
                    robot.stop().await,
                    Err(RobotError::Motor(MotorError::Pwm(_)))
                ));
                robot.shutdown().await;
                assert_eq!(robot.status().await, status);
            }
            let completed = events.lock().unwrap().clone();
            assert_eq!(
                completed
                    .iter()
                    .filter(|event| matches!(event, Event::Speed(..)))
                    .count(),
                attempted
            );
            assert_final_cleanup(&completed);
            tokio::time::advance(Duration::from_secs(1)).await;
            tokio::task::yield_now().await;
            assert_eq!(*events.lock().unwrap(), completed);
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn fault_clears_pending_targets_and_rejects_waiters_while_cleanup_is_blocked() {
    tokio::time::timeout(Duration::from_secs(5), async {
        let write = Arc::new(Gate::default());
        let cleanup = Arc::new(Gate::default());
        let _release_write = ReleaseOnDrop(write.clone());
        let _release_cleanup = ReleaseOnDrop(cleanup.clone());
        let driver = TestDriver {
            write_gate: Some(write.clone()),
            cleanup_gate: Some(cleanup.clone()),
            fail_write: Some(Motor::Motor4),
            ..TestDriver::default()
        };
        let events = driver.events.clone();
        let (robot, _driver) = create_robot(driver);
        robot.drive(0.25, 0.25).await.unwrap();
        write.entered.notified().await;
        // Accepted while the preceding output is in flight; must not survive
        // that output's failure or get applied after fault cleanup.
        robot.drive(-0.75, 0.5).await.unwrap();
        write.release();
        cleanup.entered.notified().await;
        let status = robot.status().await;
        assert_eq!(status.phase, RobotPhase::Faulted);
        assert_eq!(status.output, RobotOutputState::Unknown);
        assert!(!status.cleanup_complete);
        assert!(
            status
                .fault
                .as_ref()
                .unwrap()
                .contains("injected write failure")
        );

        let control = robot.state.control.lock().await;
        assert_eq!(control.targets, (0.0, 0.0));
        let mut drive = Box::pin(robot.drive(1.0, -1.0));
        let mut stop = Box::pin(robot.stop());
        assert_pending(drive.as_mut()).await;
        assert_pending(stop.as_mut()).await;
        drop(control);
        assert!(matches!(
            drive.await,
            Err(RobotError::Motor(MotorError::Pwm(_)))
        ));
        assert!(matches!(
            stop.await,
            Err(RobotError::Motor(MotorError::Pwm(_)))
        ));
        let mut shutdown = Box::pin(robot.shutdown());
        assert_pending(shutdown.as_mut()).await;
        drop(shutdown);
        assert_eq!(robot.status().await, status);
        cleanup.release();
        robot.shutdown().await;
        let finished = robot.status().await;
        assert_eq!(finished.fault, status.fault);
        assert_eq!(finished.phase, RobotPhase::Faulted);
        assert_eq!(finished.output, RobotOutputState::Stopped);
        assert!(finished.cleanup_complete);
        let events = events.lock().unwrap();
        assert_eq!(
            events
                .iter()
                .filter(|event| matches!(event, Event::Speed(..)))
                .count(),
            1
        );
        assert_final_cleanup(&events);
    })
    .await
    .expect("fault handling or cleanup deadlocked");
}

#[tokio::test(start_paused = true)]
async fn healthy_status_distinguishes_admission_shutdown_and_successful_stop() {
    let (robot, driver) = create_robot(TestDriver::default());
    let initial = robot.status().await;
    assert_eq!(initial.phase, RobotPhase::Running);
    assert_eq!(initial.output, RobotOutputState::Unknown);
    assert!(!initial.cleanup_complete);
    robot.drive(0.25, 0.25).await.unwrap();
    let board = driver.lock().await;
    let mut shutdown = Box::pin(robot.shutdown());
    assert_pending(shutdown.as_mut()).await;
    let stopping = robot.status().await;
    assert_eq!(stopping.phase, RobotPhase::ShuttingDown);
    assert_eq!(stopping.output, RobotOutputState::Unknown);
    assert!(!stopping.cleanup_complete);
    drop(board);
    shutdown.await;
    let stopped = robot.status().await;
    assert_eq!(stopped.phase, RobotPhase::Shutdown);
    assert_eq!(stopped.output, RobotOutputState::Stopped);
    assert!(stopped.cleanup_complete);
    assert!(stopped.fault.is_none() && stopped.cleanup_error.is_none());
}

#[tokio::test(start_paused = true)]
async fn fault_diagnostics_are_bounded_and_survive_repeated_commands() {
    let driver = TestDriver {
        fail_write: Some(Motor::Motor4),
        fail_stop: true,
        failure_detail: Some("故障".repeat(5000)),
        ..TestDriver::default()
    };
    let finished = driver.cleanup_finished.clone();
    let (robot, _driver) = create_robot(driver);
    robot.drive(0.25, 0.25).await.unwrap();
    tokio::time::timeout(Duration::from_secs(5), finished.notified())
        .await
        .unwrap();
    robot.shutdown().await;
    let status = robot.status().await;
    for diagnostic in [&status.fault, &status.cleanup_error] {
        let message = diagnostic.as_ref().unwrap();
        assert!(message.len() <= 4096);
        assert!(message.ends_with(" [truncated]"));
    }
    for _ in 0..10 {
        assert!(robot.drive(0.5, 0.5).await.is_err());
        robot.shutdown().await;
        assert_eq!(robot.status().await, status);
    }
}

#[tokio::test(start_paused = true)]
async fn adjustment_panic_is_observable_and_cleanup_still_runs() {
    let driver = TestDriver {
        panic_write: true,
        ..TestDriver::default()
    };
    let finished = driver.cleanup_finished.clone();
    let events = driver.events.clone();
    let (robot, _driver) = create_robot(driver);
    robot.drive(0.25, 0.25).await.unwrap();
    tokio::time::timeout(Duration::from_secs(5), finished.notified())
        .await
        .unwrap();
    robot.shutdown().await;
    let status = robot.status().await;
    assert_eq!(status.phase, RobotPhase::Faulted);
    assert!(status.fault.unwrap().contains("injected adjustment panic"));
    assert_eq!(status.output, RobotOutputState::Stopped);
    assert!(status.cleanup_complete);
    assert_final_cleanup(&events.lock().unwrap());
}

#[tokio::test(start_paused = true)]
async fn background_zero_output_failure_also_latches() {
    let driver = TestDriver {
        fail_zero_stop: true,
        ..TestDriver::default()
    };
    let finished = driver.cleanup_finished.clone();
    let events = driver.events.clone();
    let (robot, _driver) = create_robot(driver);
    // No nonzero command: the adjustment loop still writes stopped outputs.
    tokio::time::timeout(Duration::from_secs(5), finished.notified())
        .await
        .unwrap();
    robot.shutdown().await;
    let status = robot.status().await;
    assert_eq!(status.phase, RobotPhase::Faulted);
    assert_eq!(status.output, RobotOutputState::Unknown);
    assert!(status.fault.unwrap().contains("left motor Motor4"));
    assert!(
        status
            .cleanup_error
            .unwrap()
            .contains("injected stop failure")
    );
    let events = events.lock().unwrap();
    assert_eq!(events[0], Event::StopAttempt(Motor::Motor4));
    assert_final_cleanup(&events);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn in_flight_failure_is_retained_when_shutdown_already_started() {
    tokio::time::timeout(Duration::from_secs(5), async {
        let gate = Arc::new(Gate::default());
        let _release = ReleaseOnDrop(gate.clone());
        let driver = TestDriver {
            write_gate: Some(gate.clone()),
            fail_write: Some(Motor::Motor4),
            ..TestDriver::default()
        };
        let (robot, _driver) = create_robot(driver);
        robot.drive(0.25, 0.25).await.unwrap();
        gate.entered.notified().await;
        let mut shutdown = Box::pin(robot.shutdown());
        assert_pending(shutdown.as_mut()).await;
        assert_eq!(robot.status().await.phase, RobotPhase::ShuttingDown);
        gate.release();
        shutdown.await;
        let status = robot.status().await;
        assert_eq!(status.phase, RobotPhase::Faulted);
        assert_eq!(status.output, RobotOutputState::Stopped);
        assert!(status.cleanup_complete);
        assert!(status.fault.unwrap().contains("injected write failure"));
    })
    .await
    .expect("shutdown lost an in-flight fault");
}
