use std::sync::Mutex as StdMutex;

use super::*;

fn close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 0.000_002,
        "{actual} != {expected}"
    );
}

#[test]
fn elapsed_step_scales_early_updates_and_caps_delayed_updates() {
    let interval = Duration::from_millis(10);
    for (millis, expected) in [
        (0, 0.0),
        (2, 0.02),
        (7, 0.07),
        (10, 0.1),
        (13, 0.1),
        (1000, 0.1),
    ] {
        close(
            elapsed_step(0.1, interval, Duration::from_millis(millis)),
            expected,
        );
    }
    assert_eq!(elapsed_step(0.1, interval, Duration::MAX), 0.1);
    assert_eq!(
        elapsed_step(f32::MAX, Duration::from_secs(1), Duration::MAX),
        f32::MAX
    );
    assert_eq!(elapsed_step(1.0, Duration::MAX, Duration::MAX), 1.0);
}

#[test]
fn equal_elapsed_partitions_give_equal_progress_when_no_gap_is_capped() {
    let partitions = [
        vec![10; 10],
        vec![20; 5],
        vec![1, 19, 3, 17, 7, 13, 9, 11, 8, 12],
    ];
    for (initial, target, expected) in [
        (0.0, 1.0, 0.1),
        (0.0, -1.0, -0.1),
        (1.0, 0.0, 0.9),
        (-1.0, 0.0, -0.9),
    ] {
        for partition in &partitions {
            let mut current = initial;
            for &millis in partition {
                let change = elapsed_step(
                    0.02,
                    Duration::from_millis(20),
                    Duration::from_millis(millis),
                );
                current = ramp_toward(current, target, change);
            }
            close(current, expected);
        }
    }
}

#[test]
fn elapsed_ramp_stops_reverses_and_never_overshoots() {
    let interval = Duration::from_millis(10);
    let mut current = 0.0;
    for target in [0.5, 0.0, -0.5, 0.8, -1.0, 1.0, 0.0] {
        for millis in [0, 2, 9, 30, 1, 10, 1000].into_iter().cycle().take(200) {
            let before = current;
            let change = elapsed_step(0.1, interval, Duration::from_millis(millis));
            current = ramp_toward(current, target, change);
            assert!(current >= before.min(target) && current <= before.max(target));
            assert!((current - before).abs() <= change + f32::EPSILON);
            assert!((-1.0..=1.0).contains(&current));
        }
        close(current, target);
    }
    // Preserve the existing reversal policy: no new zero dwell is introduced.
    close(
        ramp_toward(0.05, -0.5, elapsed_step(0.1, interval, interval)),
        -0.05,
    );
    assert_eq!(
        ramp_toward(
            0.0,
            1.0,
            elapsed_step(f32::MAX, Duration::from_secs(1), Duration::from_secs(1))
        ),
        1.0
    );
}

#[test]
fn deadline_grid_skips_missed_ticks_without_adding_io_time() {
    let origin = Instant::now();
    for (period_ms, finish_ms, expected_ms, skipped) in [
        (10, 10, 20, 0),
        (10, 13, 20, 0),
        (10, 20, 30, 1),
        (10, 105, 110, 9),
        (1, 4, 5, 3),
    ] {
        let interval = Duration::from_millis(period_ms);
        let first = origin.checked_add(interval).unwrap();
        let now = origin
            .checked_add(Duration::from_millis(finish_ms))
            .unwrap();
        let (next, count) = next_tick(first, interval, now);
        assert_eq!(next, origin.checked_add(Duration::from_millis(expected_ms)));
        assert_eq!(count, skipped);
        assert!(next.unwrap() > now);
    }
    assert!(origin.checked_add(Duration::MAX).is_none());
    assert!(next_tick(origin, Duration::MAX, origin).0.is_none());
}

#[test]
fn timing_diagnostics_count_overruns_and_saturate() {
    let interval = Duration::from_millis(10);
    let mut timing = RobotTiming::default();
    for millis in [10, 7, 12, 100, 3] {
        timing.record_elapsed(Duration::from_millis(millis), interval);
    }
    assert_eq!(timing.overruns, 2);
    assert_eq!(timing.last_elapsed, Duration::from_millis(3));
    assert_eq!(timing.max_elapsed, Duration::from_millis(100));
    timing.overruns = u64::MAX;
    timing.record_elapsed(Duration::MAX, interval);
    assert_eq!(timing.overruns, u64::MAX);
    assert_eq!(timing.max_elapsed, Duration::MAX);
}

type OutputLog = Arc<StdMutex<Vec<(Instant, Motor, u16)>>>;

struct TimedDriver(OutputLog);

impl MotorDriver for TimedDriver {
    fn set_motor_speed(&self, motor: Motor, speed: u16, _: bool) -> Result<(), MotorError> {
        self.0.lock().unwrap().push((Instant::now(), motor, speed));
        Ok(())
    }

    fn stop_motor(&self, motor: Motor) -> Result<(), MotorError> {
        self.0.lock().unwrap().push((Instant::now(), motor, 0));
        Ok(())
    }
}

fn controller() -> (Arc<Robot>, Arc<Mutex<TimedDriver>>, OutputLog) {
    let log = Arc::new(StdMutex::new(Vec::new()));
    let driver = Arc::new(Mutex::new(TimedDriver(log.clone())));
    let robot = Robot::new(driver.clone(), RobotConfig::default()).unwrap();
    (robot, driver, log)
}

async fn advance(millis: u64) {
    tokio::time::advance(Duration::from_millis(millis)).await;
    tokio::task::yield_now().await;
}

#[tokio::test(start_paused = true)]
async fn delayed_tick_is_capped_and_not_replayed() {
    let (robot, _driver, log) = controller();
    robot.drive(1.0, 1.0).await.unwrap();
    tokio::task::yield_now().await;
    assert!(log.lock().unwrap().is_empty()); // No immediate first tick.
    advance(105).await;
    let first = log.lock().unwrap().clone();
    assert_eq!(first.len(), 2);
    assert_eq!(first[0].2, 410); // One 0.1 step, not a jump to full output.
    assert_eq!(first[1].2, 410);
    let timing = robot.status().await.timing;
    assert_eq!(timing.overruns, 1);
    assert_eq!(timing.skipped_ticks, 9);
    assert_eq!(timing.last_elapsed, Duration::from_millis(105));
    tokio::task::yield_now().await;
    assert_eq!(*log.lock().unwrap(), first);
    advance(4).await;
    assert_eq!(*log.lock().unwrap(), first);
    advance(1).await;
    let outputs = log.lock().unwrap().clone();
    assert_eq!(outputs.len(), 4);
    assert_eq!(outputs[2].2, 614); // Only 5 ms since the delayed output: +0.05.
    assert_eq!(
        robot.status().await.timing.last_elapsed,
        Duration::from_millis(5)
    );
    robot.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn board_wait_is_in_elapsed_time_but_does_not_shift_deadlines() {
    let (robot, driver, log) = controller();
    let board = driver.lock().await;
    let origin = Instant::now();
    robot.drive(1.0, 1.0).await.unwrap();
    tokio::task::yield_now().await;
    advance(10).await;
    assert!(log.lock().unwrap().is_empty());
    advance(3).await;
    drop(board);
    tokio::task::yield_now().await;
    assert_eq!(log.lock().unwrap().len(), 2);
    assert_eq!(
        robot.status().await.timing.last_elapsed,
        Duration::from_millis(13)
    );
    advance(6).await;
    assert_eq!(log.lock().unwrap().len(), 2);
    advance(1).await;
    let outputs = log.lock().unwrap().clone();
    assert_eq!(outputs.len(), 4);
    assert_eq!(
        outputs[2].0.duration_since(origin),
        Duration::from_millis(20)
    );
    assert_eq!(outputs[2].2, 696); // 0.1 capped first step + 0.07 over 7 ms.
    assert_eq!(robot.status().await.timing.skipped_ticks, 0);
    robot.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn equal_nominal_rates_match_at_different_intervals() {
    for (interval_ms, step) in [(10, 0.01), (20, 0.02)] {
        let log = Arc::new(StdMutex::new(Vec::new()));
        let robot = Robot::new(
            Arc::new(Mutex::new(TimedDriver(log.clone()))),
            RobotConfig {
                adjustment_step: step,
                adjustment_interval: Duration::from_millis(interval_ms),
                ..RobotConfig::default()
            },
        )
        .unwrap();
        robot.drive(0.75, 0.75).await.unwrap();
        tokio::task::yield_now().await;
        for _ in 0..400 / interval_ms {
            advance(interval_ms).await;
        }
        assert_eq!(log.lock().unwrap().last().unwrap().2, 1638); // 0.4 power after 0.4 s.
        let timing = robot.status().await.timing;
        assert_eq!(timing.overruns, 0);
        assert_eq!(timing.skipped_ticks, 0);
        robot.shutdown().await;
        assert_eq!(robot.status().await.timing, timing);
    }
}

#[tokio::test(start_paused = true)]
async fn unrepresentably_distant_deadline_stays_cancellable() {
    let log = Arc::new(StdMutex::new(Vec::new()));
    let robot = Robot::new(
        Arc::new(Mutex::new(TimedDriver(log.clone()))),
        RobotConfig {
            adjustment_step: 1.0,
            adjustment_interval: Duration::MAX,
            ..RobotConfig::default()
        },
    )
    .unwrap();
    robot.drive(1.0, 1.0).await.unwrap();
    tokio::task::yield_now().await;
    assert!(log.lock().unwrap().is_empty());
    tokio::time::timeout(Duration::from_secs(1), robot.shutdown())
        .await
        .unwrap();
    assert_eq!(robot.status().await.phase, RobotPhase::Shutdown);
    assert_eq!(robot.status().await.timing, RobotTiming::default());
}
