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
use actix_web::{App, HttpRequest, HttpResponse, HttpServer, web};
use actix_web_actors::ws;
use clap::Parser;
use serde::Deserialize;
use std::path::PathBuf;
use std::sync::Arc;
use std::{future::Future, io};
use tokio::sync::Mutex;
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;

use robot_control::motor::{Motor, MotorDriver};
use robot_control::robot::{Robot, RobotConfig, RobotPhase};

#[path = "web_server/control.rs"]
mod control;
use control::{Control, ControlError, Session, receipt_result};

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
#[path = "web_server/control_tests.rs"]
mod control_tests;
#[cfg(test)]
#[path = "web_server/tests.rs"]
mod tests;

// ---------------------------------------------------------------------------
// WebSocket actor
// ---------------------------------------------------------------------------

struct RobotWsActor {
    control: Control,
    session: Session,
}

impl RobotWsActor {
    fn new(control: Control) -> Self {
        Self {
            control,
            session: Session::new(),
        }
    }

    fn submit_text(&self, text: &str) -> Result<control::Receipt, ControlError> {
        match parse_drive(text) {
            Ok(params) => self.control.submit(Some(&self.session), params),
            Err(error) => {
                self.control.disconnect(&self.session);
                Err(error)
            }
        }
    }

    fn reject_invalid(&self, error: &ControlError, ctx: &mut ws::WebsocketContext<Self>) {
        ctx.text(error.json());
        // Invalid traffic cannot hold ownership indefinitely. Release any
        // owned stream before closing, invalidating its pending commands.
        self.control.disconnect(&self.session);
        ctx.close(Some(ws::CloseReason {
            code: ws::CloseCode::Policy,
            description: Some("invalid drive message".into()),
        }));
        ctx.stop();
    }
}

impl Actor for RobotWsActor {
    type Context = ws::WebsocketContext<Self>;

    fn started(&mut self, ctx: &mut Self::Context) {
        info!("WebSocket client connected");
        ctx.text(serde_json::json!({"status": "ready"}).to_string());
    }

    fn stopped(&mut self, _ctx: &mut Self::Context) {
        info!("WebSocket client disconnected");
        self.control.disconnect(&self.session);
    }
}

#[derive(Debug, Clone, Copy, Deserialize)]
struct DriveParams {
    left: f32,
    right: f32,
}

fn parse_drive(text: &str) -> Result<DriveParams, ControlError> {
    let params: DriveParams = serde_json::from_str(text)
        .map_err(|error| ControlError::new("invalid", format!("invalid drive message: {error}")))?;
    if !params.left.is_finite() || !params.right.is_finite() {
        return Err(ControlError::new(
            "invalid",
            "left and right must be finite numbers",
        ));
    }
    Ok(params)
}

impl StreamHandler<Result<ws::Message, ws::ProtocolError>> for RobotWsActor {
    fn handle(&mut self, msg: Result<ws::Message, ws::ProtocolError>, ctx: &mut Self::Context) {
        match msg {
            Ok(ws::Message::Text(text)) => {
                match self.submit_text(&text) {
                    Ok(receipt) => {
                        // Only await an acknowledgement here. Admission happens
                        // synchronously in receive order; one worker applies it.
                        ctx.spawn(receipt_result(receipt).into_actor(self).map(
                            |result, _, ctx| match result {
                                Ok(()) => {
                                    ctx.text(serde_json::json!({"status": "accepted"}).to_string());
                                }
                                Err(error) => ctx.text(error.json()),
                            },
                        ));
                    }
                    Err(error) if error.code == "invalid" => self.reject_invalid(&error, ctx),
                    Err(error) => ctx.text(error.json()),
                }
            }
            Ok(ws::Message::Ping(data)) => ctx.pong(&data),
            Ok(ws::Message::Close(reason)) => {
                self.control.disconnect(&self.session);
                ctx.close(reason);
                ctx.stop();
            }
            Ok(ws::Message::Binary(_) | ws::Message::Continuation(_)) => {
                self.reject_invalid(
                    &ControlError::new("invalid", "expected a complete JSON text message"),
                    ctx,
                );
            }
            Ok(_) => {}
            Err(error) => {
                warn!("WebSocket protocol error: {error}");
                self.control.disconnect(&self.session);
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
    control: web::Data<Control>,
) -> Result<HttpResponse, actix_web::Error> {
    let actor = RobotWsActor::new(control.get_ref().clone());
    ws::start(actor, &req, stream)
}

fn control_error_response(error: &ControlError) -> HttpResponse {
    let mut response = match error.code {
        "invalid" => HttpResponse::BadRequest(),
        "busy" | "superseded" | "disconnected" => HttpResponse::Conflict(),
        "queue_full" => HttpResponse::TooManyRequests(),
        _ => HttpResponse::ServiceUnavailable(),
    };
    response.json(error)
}

fn query_config() -> web::QueryConfig {
    web::QueryConfig::default().error_handler(|error, _| {
        let response = control_error_response(&ControlError::new("invalid", error.to_string()));
        actix_web::error::InternalError::from_response(error, response).into()
    })
}

async fn drive_robot(control: web::Data<Control>, params: web::Query<DriveParams>) -> HttpResponse {
    let result = match control.submit(None, params.into_inner()) {
        Ok(receipt) => receipt_result(receipt).await,
        Err(error) => Err(error),
    };
    match result {
        Ok(()) => HttpResponse::Ok().json(serde_json::json!({"status": "accepted"})),
        Err(error) => control_error_response(&error),
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
    let (control, worker) = Control::start(robot.clone());
    let app_control = control.clone();
    let result = with_robot_cleanup(&robot, async {
        let server = HttpServer::new(move || {
            App::new()
                .app_data(web::Data::new(app_robot.clone()))
                .app_data(web::Data::new(app_control.clone()))
                .app_data(query_config())
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
    .await;
    control.close();
    let joined = worker.await.map_err(io::Error::other);
    if let Err(error) = &joined {
        warn!(%error, "web command worker failed");
    }
    result.and(joined)
}
