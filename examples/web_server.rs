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
use tokio::sync::Mutex;
use tracing::{debug, info, warn};
use tracing_subscriber::EnvFilter;

use robot_control::motor::{Motor, MotorDriver};
use robot_control::robot::{Robot, RobotConfig};

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
mod tests {
    use super::Cli;
    use clap::Parser;

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
}

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

async fn health_check() -> impl Responder {
    HttpResponse::Ok().body("Healthy")
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

fn create_motor_driver(cli: &Cli) -> Arc<Mutex<dyn MotorDriver>> {
    #[cfg(feature = "pca9685")]
    {
        let board = robot_control::Pca9685MotorBoard::new(&cli.i2c_bus, cli.i2c_addr)
            .expect("failed to initialize PCA9685 motor board");
        Arc::new(Mutex::new(board))
    }

    #[cfg(not(feature = "pca9685"))]
    {
        let _ = cli;
        panic!(
            "No motor driver available. Build with `--features pca9685` for real hardware, \
             or implement `MotorDriver` for your board."
        );
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

#[tokio::main]
async fn main() -> std::io::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("robot_control=info,actix_web=info")),
        )
        .init();

    let cli = Cli::parse();
    info!(host = %cli.host, port = cli.port, "starting robot control server");

    let motor_board = create_motor_driver(&cli);
    let config = RobotConfig {
        left_motor: Motor::Motor1,
        right_motor: Motor::Motor4,
        left_motor_factor: cli.left_factor,
        right_motor_factor: cli.right_factor,
        ..RobotConfig::default()
    };
    let robot = Robot::new(motor_board, config).expect("failed to create robot");

    let static_dir = cli.static_dir.clone();
    let bind_addr = format!("{}:{}", cli.host, cli.port);

    HttpServer::new(move || {
        App::new()
            .app_data(web::Data::new(robot.clone()))
            .route("/health", web::get().to(health_check))
            .route("/drive", web::get().to(drive_robot))
            .route("/", web::get().to(index))
            .route("/ws", web::get().to(robot_ws))
            .service(Files::new("/static", static_dir.clone()).show_files_listing())
    })
    .bind(&bind_addr)?
    .run()
    .await
}
