use super::*;
use crate::mock::{MockMotorDriver, MotorCommand};

async fn fixture(interval: Duration) -> (Arc<Robot>, Arc<Mutex<MockMotorDriver>>) {
    let driver = Arc::new(Mutex::new(MockMotorDriver::new()));
    let robot = Robot::new(
        driver.clone(),
        RobotConfig {
            adjustment_interval: interval,
            ..RobotConfig::default()
        },
    )
    .unwrap();
    tokio::task::yield_now().await;
    (robot, driver)
}

async fn advance(ms: u64) {
    tokio::time::advance(Duration::from_millis(ms)).await;
    tokio::task::yield_now().await;
}

fn only_stops(commands: &[MotorCommand]) -> bool {
    !commands.is_empty()
        && commands
            .iter()
            .all(|command| matches!(command, MotorCommand::Stop { .. }))
}

#[tokio::test(start_paused = true)]
async fn finite_expiry_preserves_ramp_and_stays_stopped() {
    let (robot, driver) = fixture(Duration::from_millis(10)).await;
    robot
        .drive_for(1.0, 1.0, Duration::from_millis(35))
        .await
        .unwrap();
    for _ in 0..3 {
        advance(10).await;
    }
    driver.lock().await.clear();
    advance(5).await;
    // Expiry clears intent at the deadline, without an extra output tick.
    assert_eq!(driver.lock().await.commands(), Vec::<MotorCommand>::new());
    assert_eq!(robot.state.control.lock().await.targets, (0.0, 0.0));
    assert_eq!(robot.status().await.expired_commands, 1);
    advance(5).await;
    let mut previous = 1229; // Three nominal 0.1 steps.
    for _ in 0..5 {
        let commands = driver.lock().await.commands();
        assert_eq!(commands.len(), 2);
        for command in &commands {
            let pwm = match command {
                MotorCommand::SetSpeed { speed, forward, .. } => {
                    assert!(*forward);
                    *speed
                }
                MotorCommand::Stop { .. } => 0,
            };
            assert!(pwm <= previous && previous - pwm <= 411);
        }
        previous = match commands[0] {
            MotorCommand::SetSpeed { speed, .. } => speed,
            MotorCommand::Stop { .. } => 0,
        };
        driver.lock().await.clear();
        advance(10).await;
    }
    assert_eq!(previous, 0);
    assert!(only_stops(&driver.lock().await.commands()));
    driver.lock().await.clear();
    advance(1000).await;
    assert!(only_stops(&driver.lock().await.commands()));
    robot
        .drive_for(1.0, 1.0, Duration::from_millis(500))
        .await
        .unwrap();
    driver.lock().await.clear();
    advance(10).await;
    assert!(
        driver
            .lock()
            .await
            .commands()
            .iter()
            .any(|command| matches!(command, MotorCommand::SetSpeed { speed: 410, .. }))
    );
    robot.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn lease_timer_wakes_before_long_or_unrepresentable_ramp_tick() {
    for interval in [Duration::from_secs(10), Duration::MAX] {
        let (robot, driver) = fixture(interval).await;
        robot
            .drive_for(0.5, -0.5, Duration::from_millis(50))
            .await
            .unwrap();
        advance(49).await;
        assert_eq!(driver.lock().await.commands(), Vec::<MotorCommand>::new());
        advance(1).await;
        assert_eq!(driver.lock().await.commands(), Vec::<MotorCommand>::new());
        assert_eq!(robot.state.control.lock().await.targets, (0.0, 0.0));
        assert!(robot.state.control.lock().await.expires.is_none());
        assert_eq!(robot.status().await.expired_commands, 1);
        robot.shutdown().await;
    }
}

#[tokio::test(start_paused = true)]
async fn renewal_is_atomic_and_late_renewal_cannot_restart_expired_motion() {
    let (robot, driver) = fixture(Duration::from_millis(10)).await;
    robot
        .drive_for(0.5, 0.5, Duration::from_millis(100))
        .await
        .unwrap();
    advance(90).await;
    let deadline = Instant::now() + Duration::from_millis(100);
    assert!(
        robot
            .renew_until(-0.5, -0.5, deadline.into_std())
            .await
            .unwrap()
    );
    advance(10).await;
    assert_eq!(robot.status().await.expired_commands, 0);
    assert_eq!(robot.state.control.lock().await.targets, (-0.5, -0.5));
    advance(90).await;
    driver.lock().await.clear();
    assert!(
        !robot
            .renew_until(
                1.0,
                1.0,
                (Instant::now() + Duration::from_secs(1)).into_std()
            )
            .await
            .unwrap()
    );
    advance(10).await;
    assert!(only_stops(&driver.lock().await.commands()));
    assert_eq!(robot.status().await.expired_commands, 1);
    robot.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn invalid_finite_commands_preserve_targets_and_deadline() {
    let (robot, _) = fixture(Duration::from_millis(10)).await;
    robot
        .drive_for(0.5, -0.5, Duration::from_secs(1))
        .await
        .unwrap();
    let deadline = robot.state.control.lock().await.expires;
    for duration in [Duration::ZERO, Duration::MAX] {
        assert!(robot.drive_for(1.0, 1.0, duration).await.is_err());
    }
    assert!(
        robot
            .drive_for(f32::NAN, 0.0, Duration::from_secs(1))
            .await
            .is_err()
    );
    assert!(
        !robot
            .drive_until(1.0, 1.0, Instant::now().into_std())
            .await
            .unwrap()
    );
    assert!(
        !robot
            .renew_until(1.0, 1.0, Instant::now().into_std())
            .await
            .unwrap()
    );
    let control = robot.state.control.lock().await;
    assert_eq!(control.targets, (0.5, -0.5));
    assert_eq!(control.expires, deadline);
    drop(control);
    robot.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn legacy_drive_stays_persistent_and_ordinary_stop_stays_smooth() {
    let (robot, driver) = fixture(Duration::from_millis(10)).await;
    robot
        .drive_for(0.5, 0.5, Duration::from_millis(100))
        .await
        .unwrap();
    robot.drive(0.5, 0.5).await.unwrap();
    for _ in 0..5 {
        advance(10).await;
    }
    advance(1000).await;
    assert_eq!(robot.status().await.expired_commands, 0);
    driver.lock().await.clear();
    robot.stop().await.unwrap();
    advance(10).await;
    assert!(
        driver
            .lock()
            .await
            .commands()
            .iter()
            .any(|command| matches!(command, MotorCommand::SetSpeed { speed, .. } if *speed > 0))
    );
    for _ in 0..5 {
        driver.lock().await.clear();
        advance(10).await;
    }
    assert!(only_stops(&driver.lock().await.commands()));
    robot.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn expiry_rechecks_after_waiting_for_board_and_late_lock_waiter_cannot_renew() {
    let (robot, driver) = fixture(Duration::from_millis(10)).await;
    robot
        .drive_for(0.5, 0.5, Duration::from_millis(50))
        .await
        .unwrap();
    let held = driver.lock().await;
    advance(100).await;
    assert!(
        !robot
            .renew_until(
                1.0,
                1.0,
                (Instant::now() + Duration::from_secs(1)).into_std()
            )
            .await
            .unwrap()
    );
    held.clear();
    drop(held);
    tokio::task::yield_now().await;
    assert!(only_stops(&driver.lock().await.commands()));
    robot.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn fresh_intent_after_expiry_preserves_ramp_continuity() {
    let (robot, driver) = fixture(Duration::from_millis(10)).await;
    robot
        .drive_for(1.0, 1.0, Duration::from_millis(50))
        .await
        .unwrap();
    for _ in 0..3 {
        advance(10).await;
    }
    let held = driver.lock().await;
    held.clear();
    advance(30).await;
    robot
        .drive_for(-1.0, -1.0, Duration::from_secs(1))
        .await
        .unwrap();
    drop(held);
    tokio::task::yield_now().await;
    // A fresh reverse command after expiry must decelerate from +0.3,
    // rather than resetting the ramp and jumping straight to negative output.
    let commands = driver.lock().await.commands();
    assert_eq!(commands.len(), 2);
    assert!(commands.iter().all(|command| matches!(
        command,
        MotorCommand::SetSpeed {
            speed: 819,
            forward: true,
            ..
        }
    )));
    assert_eq!(robot.status().await.expired_commands, 1);
    for _ in 0..3 {
        driver.lock().await.clear();
        advance(10).await;
    }
    assert!(
        driver
            .lock()
            .await
            .commands()
            .iter()
            .all(|command| matches!(
                command,
                MotorCommand::SetSpeed {
                    speed: 409..=410,
                    forward: false,
                    ..
                }
            ))
    );
    robot.shutdown().await;
}

struct StopFailure;
impl MotorDriver for StopFailure {
    fn set_motor_speed(&self, _: Motor, _: u16, _: bool) -> Result<(), MotorError> {
        Ok(())
    }
    fn stop_motor(&self, _: Motor) -> Result<(), MotorError> {
        Err(MotorError::I2c("injected expiry stop failure".into()))
    }
}

#[tokio::test(start_paused = true)]
async fn expiry_stop_failure_latches_fault_and_blocks_restart() {
    let robot = Robot::new(Arc::new(Mutex::new(StopFailure)), RobotConfig::default()).unwrap();
    robot
        .drive_for(0.5, 0.5, Duration::from_millis(5))
        .await
        .unwrap();
    tokio::task::yield_now().await;
    advance(10).await;
    robot.shutdown().await;
    let status = robot.status().await;
    assert_eq!(status.phase, RobotPhase::Faulted);
    assert!(
        status
            .fault
            .unwrap()
            .contains("injected expiry stop failure")
    );
    assert!(status.cleanup_error.is_some());
    assert!(
        robot
            .drive_for(0.5, 0.5, Duration::from_secs(1))
            .await
            .is_err()
    );
    assert!(robot.stop().await.is_err());
}

#[tokio::test(start_paused = true)]
async fn renewal_waiting_on_state_lock_rechecks_expiry_before_replacing_targets() {
    let (robot, driver) = fixture(Duration::from_millis(10)).await;
    robot
        .drive_for(0.5, 0.5, Duration::from_millis(50))
        .await
        .unwrap();
    let held = robot.state.control.lock().await;
    let renewal = robot.renew_until(
        1.0,
        1.0,
        (Instant::now() + Duration::from_secs(1)).into_std(),
    );
    tokio::pin!(renewal);
    super::tests::assert_pending(renewal.as_mut()).await;
    advance(50).await;
    drop(held);
    assert!(!renewal.await.unwrap());
    tokio::task::yield_now().await;
    assert!(only_stops(&driver.lock().await.commands()));
    robot.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn expiry_and_explicit_stop_have_the_same_paired_ramp() {
    let (finite, finite_driver) = fixture(Duration::from_millis(10)).await;
    let (persistent, persistent_driver) = fixture(Duration::from_millis(10)).await;
    finite
        .drive_for(0.5, -0.3, Duration::from_millis(55))
        .await
        .unwrap();
    persistent.drive(0.5, -0.3).await.unwrap();
    for _ in 0..5 {
        advance(10).await;
    }
    advance(5).await;
    persistent.stop().await.unwrap();
    finite_driver.lock().await.clear();
    persistent_driver.lock().await.clear();
    advance(5).await;
    for _ in 0..6 {
        let expired = finite_driver.lock().await.commands();
        let stopped = persistent_driver.lock().await.commands();
        assert_eq!(expired.len(), 2);
        assert_eq!(expired, stopped);
        finite_driver.lock().await.clear();
        persistent_driver.lock().await.clear();
        advance(10).await;
    }
    assert!(only_stops(&finite_driver.lock().await.commands()));
    finite.shutdown().await;
    persistent.shutdown().await;
}
