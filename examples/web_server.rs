//! # Web Server Example
//!
//! A complete web-based robot control server with dual-joystick UI and WebSocket support.
//!
//! ## Usage
//!
//! ```bash
//! cargo run --example web_server --features pca9685
//! cargo run --example web_server --features pca9685 -- --host 0.0.0.0 --port 8080
//! ```
//!
//! Then open `http://<robot-ip>:8000` in a browser for the joystick control UI.

use actix::prelude::*;
use actix_files::{Files, NamedFile};
use actix_web::{App, HttpRequest, HttpResponse, HttpServer, Responder, web};
use actix_web_actors::ws;
use clap::Parser;
use serde::Deserialize;
use std::path::PathBuf;
use std::sync::Arc;
use std::{future::Future, io};
use tokio::sync::Mutex;
use tracing::{debug, info, warn};
use tracing_subscriber::EnvFilter;

use robot_control::motor::{Motor, MotorDriver};
use robot_control::robot::{Robot, RobotConfig, RobotPhase};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

/// A web-controlled robot server using PCA9685 motor driver boards.
#[derive(Parser, Debug)]
#[command(version, about)]
struct Cli {
    /// Address to bind the HTTP server to.
    #[arg(long, default_value = "0.0.0.0")]
    host: String,

    /// Port to bind the HTTP server to.
    #[arg(long, default_value_t = 8000)]
    port: u16,

    /// I2C bus device path.
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_bus: String,

    /// I2C address of the PCA9685 board (hex, e.g., 0x60).
    #[arg(long, default_value_t = 0x60)]
    i2c_addr: u8,

    /// Left motor speed factor (-1.0 to reverse polarity).
    #[arg(long, default_value_t = 1.0, allow_hyphen_values = true)]
    left_factor: f32,

    /// Right motor speed factor (-1.0 to reverse polarity).
    #[arg(long, default_value_t = 1.0, allow_hyphen_values = true)]
    right_factor: f32,

    /// Path to static web files directory.
    #[arg(long, default_value = "./static")]
    static_dir: PathBuf,
}

#[cfg(test)]
#[path = "web_server/tests.rs"]
mod tests;

// ---------------------------------------------------------------------------
// WebSocket actor
// ---------------------------------------------------------------------------

struct RobotWsActor {
    robot: Arc<Robot>,
}

impl RobotWsActor {
    fn new(robot: Arc<Robot>) -> Self {
        Self { robot }
    }
}

impl Actor for RobotWsActor {
    type Context = ws::WebsocketContext<Self>;

    fn started(&mut self, _ctx: &mut Self::Context) {
        info!("WebSocket client connected");
    }

    fn stopped(&mut self, _ctx: &mut Self::Context) {
        info!("WebSocket client disconnected");
        let robot = self.robot.clone();
        actix_rt::spawn(async move {
            if let Err(e) = robot.stop().await {
                warn!("failed to stop robot on disconnect: {e}");
            }
        });
    }
}

#[derive(Debug, Deserialize)]
struct DriveParams {
    left: f32,
    right: f32,
}

impl StreamHandler<Result<ws::Message, ws::ProtocolError>> for RobotWsActor {
    fn handle(&mut self, msg: Result<ws::Message, ws::ProtocolError>, ctx: &mut Self::Context) {
        match msg {
            Ok(ws::Message::Text(text)) => match serde_json::from_str::<DriveParams>(&text) {
                Ok(params) => {
                    let robot = self.robot.clone();
                    actix_rt::spawn(async move {
                        if let Err(e) = robot.drive(params.left, params.right).await {
                            warn!("drive command failed: {e}");
                        }
                    });
                }
                Err(e) => {
                    debug!("invalid WebSocket message: {e}");
                    ctx.text(format!(r#"{{"error":"invalid message: {e}"}}"#));
                }
            },
            Ok(ws::Message::Ping(data)) => ctx.pong(&data),
            Ok(ws::Message::Close(reason)) => {
                info!("WebSocket close received");
                let robot = self.robot.clone();
                actix_rt::spawn(async move {
                    if let Err(e) = robot.stop().await {
                        warn!("failed to stop robot on close: {e}");
                    }
                });
                ctx.close(reason);
                ctx.stop();
            }
            Ok(_) => {}
            Err(e) => {
                warn!("WebSocket protocol error: {e}");
                ctx.stop();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------

async fn health_check(robot: web::Data<Arc<Robot>>) -> HttpResponse {
    let status = robot.status().await;
    let mut response = if status.phase == RobotPhase::Running {
        HttpResponse::Ok()
    } else {
        HttpResponse::ServiceUnavailable()
    };
    // Reports controller state, not a hardware self-test or measured motion.
    response.json(serde_json::json!({
        "phase": format!("{:?}", status.phase),
        "output": format!("{:?}", status.output),
        "fault": status.fault,
        "cleanup_error": status.cleanup_error,
        "cleanup_complete": status.cleanup_complete,
    }))
}

async fn index() -> actix_web::Result<NamedFile> {
    let path: PathBuf = "./static/index.html".parse().expect("static path is valid");
    Ok(NamedFile::open(path)?)
}

async fn robot_ws(
    req: HttpRequest,
    stream: web::Payload,
    robot: web::Data<Arc<Robot>>,
) -> Result<HttpResponse, actix_web::Error> {
    let actor = RobotWsActor::new(robot.get_ref().clone());
    ws::start(actor, &req, stream)
}

async fn drive_robot(
    robot: web::Data<Arc<Robot>>,
    params: web::Query<DriveParams>,
) -> impl Responder {
    match robot.drive(params.left, params.right).await {
        Ok(()) => HttpResponse::Ok().body("Driving"),
        Err(e) => {
            warn!("drive request failed: {e}");
            HttpResponse::InternalServerError().body(format!("Error: {e}"))
        }
    }
}

// ---------------------------------------------------------------------------
// Motor driver factory
// ---------------------------------------------------------------------------

fn create_motor_driver(cli: &Cli) -> io::Result<Arc<Mutex<dyn MotorDriver>>> {
    #[cfg(feature = "pca9685")]
    {
        let board = robot_control::Pca9685MotorBoard::new(&cli.i2c_bus, cli.i2c_addr)
            .map_err(io::Error::other)?;
        Ok(Arc::new(Mutex::new(board)))
    }

    #[cfg(not(feature = "pca9685"))]
    {
        let _ = cli;
        Err(io::Error::other(
            "No motor driver available. Build with `--features pca9685` for real hardware, \
             or implement `MotorDriver` for your board.",
        ))
    }
}

// ---------------------------------------------------------------------------
// Server lifecycle (injected futures keep tests independent of sockets/signals)
// ---------------------------------------------------------------------------

async fn with_robot_cleanup(
    robot: &Robot,
    operation: impl Future<Output = io::Result<()>>,
) -> io::Result<()> {
    let result = operation.await;
    robot.shutdown().await;
    let status = robot.status().await;
    info!(?status, "robot cleanup completed");
    if let Some(error) = &status.fault {
        warn!(%error, "robot controller fault");
    }
    if let Some(error) = &status.cleanup_error {
        warn!(%error, "robot cleanup failed; outputs are uncertain");
    }
    // Never replace the original bind/serve/signal error with a cleanup error.
    result?;
    if let Some(error) = status.cleanup_error.or(status.fault) {
        return Err(io::Error::other(error));
    }
    Ok(())
}

async fn serve_until_exit<Stop, Stopped>(
    robot: &Robot,
    server: impl Future<Output = io::Result<()>>,
    signal: impl Future<Output = io::Result<()>>,
    stop_server: Stop,
) -> io::Result<()>
where
    Stop: FnOnce() -> Stopped,
    Stopped: Future<Output = ()>,
{
    tokio::pin!(server);
    tokio::select! {
        result = &mut server => result,
        result = signal => {
            info!("server shutdown requested; stopping robot");
            // Close motion admission and finish motor cleanup before waiting on
            // HTTP workers or long-lived WebSocket connections.
            robot.shutdown().await;
            // Actix's server future must keep being polled while its handle's
            // stop acknowledgement is awaited. Do not just drop the server.
            let (server_result, ()) = tokio::join!(server, stop_server());
            result.and(server_result)
        }
    }
}

fn shutdown_signal() -> io::Result<impl Future<Output = io::Result<()>>> {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{SignalKind, signal};
        // Install both handlers before hardware initialization.
        let mut interrupt = signal(SignalKind::interrupt())?;
        let mut terminate = signal(SignalKind::terminate())?;
        Ok(async move {
            let received = tokio::select! {
                received = interrupt.recv() => received,
                received = terminate.recv() => received,
            };
            received.ok_or_else(|| io::Error::other("shutdown signal stream closed"))
        })
    }
    #[cfg(not(unix))]
    {
        Ok(tokio::signal::ctrl_c())
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

#[tokio::main]
async fn main() -> std::io::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::try_from_default_env().unwrap_or_else(|_| {
            EnvFilter::new("robot_control=info,web_server=info,actix_web=info")
        }))
        .init();

    let cli = Cli::parse();
    let signal = shutdown_signal()?;
    info!(host = %cli.host, port = cli.port, "starting robot control server");

    let motor_board = create_motor_driver(&cli)?;
    let config = RobotConfig {
        left_motor: Motor::Motor4,
        right_motor: Motor::Motor1,
        left_motor_factor: cli.left_factor,
        right_motor_factor: cli.right_factor,
        ..RobotConfig::default()
    };
    let robot = match Robot::new(motor_board.clone(), config) {
        Ok(robot) => robot,
        Err(error) => {
            if let Err(cleanup_error) = motor_board.lock().await.stop_all() {
                warn!(%cleanup_error, "failed to stop board after robot startup failure");
            }
            return Err(io::Error::other(error));
        }
    };

    let static_dir = cli.static_dir.clone();
    let bind_addr = format!("{}:{}", cli.host, cli.port);

    let app_robot = robot.clone();
    with_robot_cleanup(&robot, async {
        let server = HttpServer::new(move || {
            App::new()
                .app_data(web::Data::new(app_robot.clone()))
                .route("/health", web::get().to(health_check))
                .route("/drive", web::get().to(drive_robot))
                .route("/", web::get().to(index))
                .route("/ws", web::get().to(robot_ws))
                .service(Files::new("/static", static_dir.clone()).show_files_listing())
        })
        .disable_signals()
        .bind(&bind_addr)?
        .run();
        let handle = server.handle();
        serve_until_exit(&robot, server, signal, || handle.stop(false)).await
    })
    .await
}
