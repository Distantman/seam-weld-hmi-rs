mod app;
mod comms;
mod config;
mod hal;

use app::SeamWelderHmiApp;
use comms::{spawn_comms_worker, HmiCommand, TelemetrySnapshot};
use config::MachineConfig;
use crossbeam_channel::bounded;
use eframe::egui;
use std::env;
use tracing::info;

fn main() -> eframe::Result<()> {
    // 1. Initialize structured industrial telemetry logging
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_target(false)
        .init();

    info!("==================================================================");
    info!(" MITUSA Longitudinal Seam Welder HMI (Raspberry Pi Kiosk Runtime) ");
    info!("==================================================================");

    // 2. Parse optional CLI overrides (--config <path>, --serial <pty_or_tty>)
    let args: Vec<String> = env::args().collect();
    let mut config_path = "configs/seam_welder_10ft.toml".to_string();
    let mut serial_override: Option<String> = env::var("SEAM_HMI_SERIAL_PORT").ok();

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--config" if i + 1 < args.len() => {
                config_path = args[i + 1].clone();
                i += 2;
            }
            "--serial" if i + 1 < args.len() => {
                serial_override = Some(args[i + 1].clone());
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }

    // 3. Load TOML Machine Profile (10ft Longitudinal Seam Welder)
    let machine_config = MachineConfig::load_or_default(&config_path);

    // 4. Create lock-free bounded channels between the 60Hz GUI thread and 50Hz RS-485 worker
    let (cmd_tx, cmd_rx) = bounded::<HmiCommand>(64);
    let (telemetry_tx, telemetry_rx) = bounded::<TelemetrySnapshot>(4);

    // 5. Spawn the 50Hz real-time Modbus RTU polling & 200ms Hardware Watchdog worker thread
    spawn_comms_worker(
        machine_config.clone(),
        serial_override,
        cmd_rx,
        telemetry_tx,
    );

    // 6. Launch the 1024x600 (7-inch Raspberry Pi Industrial Touchscreen) eframe Kiosk Window
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(format!(
                "MITUSA Industrial HMI — {}",
                machine_config.machine.model_name
            ))
            .with_inner_size([1024.0, 600.0])
            .with_min_inner_size([960.0, 560.0]),
        ..Default::default()
    };

    eframe::run_native(
        "seam-weld-hmi-rs",
        native_options,
        Box::new(move |cc| {
            Box::new(SeamWelderHmiApp::new(
                cc,
                machine_config,
                cmd_tx,
                telemetry_rx,
            ))
        }),
    )
}
