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
        Ok(())
    }

    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        self.events.lock().unwrap().push(Event::StopAttempt(motor));
        if self.fail_stop && motor == Motor::Motor2 {
            Err(MotorError::I2c("injected stop failure".into()))
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
            Err(RobotError::Shutdown)
        ));
    })
    .await
    .expect("stop failure prevented shutdown completion");
}
