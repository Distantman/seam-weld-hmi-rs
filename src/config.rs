use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use tracing::{info, warn};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineConfig {
    pub machine: MachineProfile,
    pub serial: SerialBusConfig,
    pub safety: SafetyLimitsConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineProfile {
    pub model_name: String,
    pub serial_number: String,
    pub max_rail_length_mm: f64,
    pub home_offset_mm: f64,
    pub default_weld_start_mm: f64,
    pub default_weld_end_mm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerialBusConfig {
    pub port_path: String,
    pub baud_rate: u32,
    pub servo_slave_id: u8,
    pub welder_slave_id: u8,
    pub poll_interval_ms: u64,
    pub watchdog_timeout_ms: u64,
    pub response_timeout_ms: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyLimitsConfig {
    pub max_travel_speed_mm_min: f64,
    pub max_jog_speed_mm_min: f64,
    pub max_weld_current_amps: f64,
    pub over_current_trip_amps: f64,
    pub max_arc_voltage_v: f64,
    pub min_gas_pre_flow_ms: u64,
    pub clamping_dwell_ms: u64,
}

impl Default for MachineConfig {
    fn default() -> Self {
        Self {
            machine: MachineProfile {
                model_name: "MITUSA-LSW-3000 (10ft Longitudinal Seam Welder)".to_string(),
                serial_number: "LSW-2026-0042".to_string(),
                max_rail_length_mm: 3000.0,
                home_offset_mm: 25.0,
                default_weld_start_mm: 150.0,
                default_weld_end_mm: 2850.0,
            },
            serial: SerialBusConfig {
                port_path: "/dev/ttyAMA0".to_string(),
                baud_rate: 115200,
                servo_slave_id: 1,
                welder_slave_id: 2,
                poll_interval_ms: 20,
                watchdog_timeout_ms: 200,
                response_timeout_ms: 35,
            },
            safety: SafetyLimitsConfig {
                max_travel_speed_mm_min: 2500.0,
                max_jog_speed_mm_min: 1800.0,
                max_weld_current_amps: 350.0,
                over_current_trip_amps: 375.0,
                max_arc_voltage_v: 36.0,
                min_gas_pre_flow_ms: 600,
                clamping_dwell_ms: 500,
            },
        }
    }
}

impl MachineConfig {
    pub fn load_or_default<P: AsRef<Path>>(path: P) -> Self {
        let path_ref = path.as_ref();
        match fs::read_to_string(path_ref) {
            Ok(raw_toml) => match toml::from_str::<MachineConfig>(&raw_toml) {
                Ok(cfg) => {
                    info!(
                        "Loaded machine profile '{}' (S/N: {}) from {:?}",
                        cfg.machine.model_name, cfg.machine.serial_number, path_ref
                    );
                    cfg
                }
                Err(err) => {
                    warn!(
                        "Failed to parse TOML config {:?}: {}. Falling back to built-in safety profile.",
                        path_ref, err
                    );
                    Self::default()
                }
            },
            Err(err) => {
                warn!(
                    "Config file {:?} not found ({}). Using default 10ft seam welder profile.",
                    path_ref, err
                );
                Self::default()
            }
        }
    }
}
