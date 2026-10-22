use crate::config::MachineConfig;
use crate::hal::{
    registers, AxisTelemetry, LinearMotionAxis, ModbusError, ModbusFunction, ModbusRtuFrame,
    WeldPowerSource, WeldPowerTelemetry,
};
use crossbeam_channel::{Receiver, Sender};
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::time::{Duration, Instant};
use tracing::{error, info, warn};

/// Deterministic industrial state machine for the longitudinal seam welder
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MachineState {
    Idle,
    Homing,
    Jogging,
    Clamping,
    ArcStrike,
    WeldingSeam,
    FaultEstop,
}

impl MachineState {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Idle => "IDLE",
            Self::Homing => "HOMING",
            Self::Jogging => "JOGGING",
            Self::Clamping => "CLAMPING",
            Self::ArcStrike => "ARC_STRIKE",
            Self::WeldingSeam => "WELDING_SEAM",
            Self::FaultEstop => "FAULT_ESTOP",
        }
    }
}

/// Fault injection triggers for technician testing & HMI verification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultInjectionKind {
    OverCurrent,
    LimitSwitchHit,
    Rs485CrcNoise,
}

/// Lock-free commands dispatched from the `egui` operator thread to the 50Hz real-time worker
#[derive(Debug, Clone)]
pub enum HmiCommand {
    JogLeft {
        speed_mm_min: f64,
    },
    JogRight {
        speed_mm_min: f64,
    },
    /// Sent periodically by the GUI while an operator holds a Jog button (feeds the 200ms Watchdog)
    JogHeartbeat,
    StopMotion,
    Home,
    StartWeldSequence,
    SetParameters {
        travel_speed_mm_min: f64,
        wire_feed_mm_min: f64,
        target_current_amps: f64,
    },
    SetClamps {
        left_locked: bool,
        right_locked: bool,
    },
    SetShieldingGas {
        enabled: bool,
    },
    EmergencyStop {
        reason: String,
    },
    ResetFault,
    InjectFault(FaultInjectionKind),
}

/// Single RS-485 bus transaction entry for the technician diagnostic hex inspector
#[derive(Debug, Clone)]
pub struct Rs485FrameLog {
    pub timestamp_sec: f64,
    pub tx_hex: String,
    pub rx_hex: String,
    pub rtt_ms: f64,
    pub crc_ok: bool,
    pub description: String,
}

/// Complete real-time snapshot published at 50Hz to the GUI thread
#[derive(Debug, Clone)]
pub struct TelemetrySnapshot {
    pub timestamp_sec: f64,
    pub state: MachineState,
    pub active_fault_reason: Option<String>,
    pub bus_connected: bool,
    pub bus_mode_label: String,
    pub bus_latency_ms: f64,
    pub watchdog_ok: bool,
    pub watchdog_remaining_ms: u64,
    pub crc_error_count: u64,
    pub tx_frame_count: u64,
    pub rx_frame_count: u64,
    pub position_mm: f64,
    pub velocity_mm_min: f64,
    pub limit_left: bool,
    pub limit_right: bool,
    pub is_homed: bool,
    pub actual_current_amps: f64,
    pub actual_voltage_v: f64,
    pub wire_feed_speed_mm_min: f64,
    pub target_current_amps: f64,
    pub target_travel_speed_mm_min: f64,
    pub arc_active: bool,
    pub gas_purging: bool,
    pub left_clamp_locked: bool,
    pub right_clamp_locked: bool,
    pub recent_frames: Vec<Rs485FrameLog>,
}

/// Combined Virtual Loopback + Optional Physical/Socat UART RS-485 Hardware Transceiver
struct IndustrialBusTransceiver {
    config: MachineConfig,
    serial_port: Option<Box<dyn tokio_serial::SerialPort>>,
    start_instant: Instant,
    // Internal deterministic physics state (used in Virtual Loopback mode)
    sim_pos_mm: f64,
    sim_vel_mm_min: f64,
    sim_target_vel_mm_min: f64,
    sim_is_homed: bool,
    sim_homing_active: bool,
    sim_limit_left: bool,
    sim_limit_right: bool,
    sim_drive_fault: bool,
    sim_target_current_a: f64,
    sim_wire_feed_mm_min: f64,
    sim_arc_enabled: bool,
    sim_gas_enabled: bool,
    sim_left_clamp: bool,
    sim_right_clamp: bool,
    sim_over_current_fault: bool,
    inject_crc_noise_next: bool,
    last_physics_tick: Instant,
    // Diagnostics
    tx_count: u64,
    rx_count: u64,
    crc_errors: u64,
    last_rtt_ms: f64,
    frame_history: VecDeque<Rs485FrameLog>,
}

impl IndustrialBusTransceiver {
    fn new(config: MachineConfig, force_serial_path: Option<String>) -> Self {
        let port_to_try = force_serial_path.unwrap_or_else(|| config.serial.port_path.clone());
        let serial_port = match tokio_serial::new(&port_to_try, config.serial.baud_rate)
            .timeout(Duration::from_millis(config.serial.response_timeout_ms))
            .open()
        {
            Ok(port) => {
                info!(
                    "Opened live RS-485 serial interface at {} ({} baud)",
                    port_to_try, config.serial.baud_rate
                );
                Some(port)
            }
            Err(err) => {
                info!(
                    "Serial device '{}' unavailable ({}). Engaging deterministic internal RS-485 Physics Loopback.",
                    port_to_try, err
                );
                None
            }
        };

        let now = Instant::now();
        Self {
            config,
            serial_port,
            start_instant: now,
            sim_pos_mm: 150.0,
            sim_vel_mm_min: 0.0,
            sim_target_vel_mm_min: 0.0,
            sim_is_homed: true,
            sim_homing_active: false,
            sim_limit_left: false,
            sim_limit_right: false,
            sim_drive_fault: false,
            sim_target_current_a: 185.0,
            sim_wire_feed_mm_min: 3200.0,
            sim_arc_enabled: false,
            sim_gas_enabled: false,
            sim_left_clamp: true,
            sim_right_clamp: true,
            sim_over_current_fault: false,
            inject_crc_noise_next: false,
            last_physics_tick: now,
            tx_count: 0,
            rx_count: 0,
            crc_errors: 0,
            last_rtt_ms: 1.4,
            frame_history: VecDeque::with_capacity(24),
        }
    }

    fn bus_mode_label(&self) -> String {
        if self.serial_port.is_some() {
            format!(
                "RS-485 UART ({} @ {} Bd)",
                self.config.serial.port_path, self.config.serial.baud_rate
            )
        } else {
            "RS-485 INTERNAL PHYSICS LOOPBACK (115200 8N1)".to_string()
        }
    }

    /// Step the internal servo + arc physics model at the 50Hz tick rate
    fn step_physics(&mut self) {
        let now = Instant::now();
        let dt_sec = now.duration_since(self.last_physics_tick).as_secs_f64();
        self.last_physics_tick = now;

        if self.sim_drive_fault || self.sim_over_current_fault {
            self.sim_vel_mm_min = 0.0;
            self.sim_target_vel_mm_min = 0.0;
            self.sim_arc_enabled = false;
            return;
        }

        if self.sim_homing_active {
            self.sim_target_vel_mm_min = -1200.0;
        }

        // Smooth trapezoidal acceleration ramp (6000 mm/min/s)
        let max_delta = 6000.0 * dt_sec;
        let diff = self.sim_target_vel_mm_min - self.sim_vel_mm_min;
        if diff.abs() <= max_delta {
            self.sim_vel_mm_min = self.sim_target_vel_mm_min;
        } else {
            self.sim_vel_mm_min += diff.signum() * max_delta;
        }

        // Integrate carriage position along the 3000mm linear rail
        let delta_mm = (self.sim_vel_mm_min / 60.0) * dt_sec;
        self.sim_pos_mm += delta_mm;

        // Evaluate physical limit switches
        let max_rail = self.config.machine.max_rail_length_mm;
        if self.sim_pos_mm <= self.config.machine.home_offset_mm && self.sim_homing_active {
            self.sim_pos_mm = self.config.machine.home_offset_mm;
            self.sim_vel_mm_min = 0.0;
            self.sim_target_vel_mm_min = 0.0;
            self.sim_homing_active = false;
            self.sim_is_homed = true;
            self.sim_limit_left = false;
        } else if self.sim_pos_mm <= 0.0 {
            self.sim_pos_mm = 0.0;
            self.sim_vel_mm_min = 0.0;
            self.sim_target_vel_mm_min = 0.0;
            self.sim_limit_left = true;
        } else if self.sim_pos_mm >= max_rail {
            self.sim_pos_mm = max_rail;
            self.sim_vel_mm_min = 0.0;
            self.sim_target_vel_mm_min = 0.0;
            self.sim_limit_right = true;
        } else {
            self.sim_limit_left = false;
            self.sim_limit_right = false;
        }
    }

    /// Execute a deterministic Modbus RTU request/response transaction over either UART or Virtual Loopback
    fn transact(
        &mut self,
        request: &ModbusRtuFrame,
        description: &str,
    ) -> Result<ModbusRtuFrame, ModbusError> {
        let t0 = Instant::now();
        let tx_bytes = request.encode();
        self.tx_count += 1;

        let mut rx_bytes = if let Some(port) = self.serial_port.as_mut() {
            port.write_all(&tx_bytes)
                .map_err(|e| ModbusError::IoError(e.to_string()))?;
            let mut buf = [0u8; 64];
            let n = port
                .read(&mut buf)
                .map_err(|e| ModbusError::IoError(e.to_string()))?;
            buf[..n].to_vec()
        } else {
            self.emulate_slave_response(request)
        };

        // Optional technician fault injection: flip a bit in the wire response to trigger a real CRC-16 error
        if self.inject_crc_noise_next && rx_bytes.len() >= 4 {
            let idx = rx_bytes.len() - 3;
            rx_bytes[idx] ^= 0x5A;
            self.inject_crc_noise_next = false;
        }

        let rtt_ms = t0.elapsed().as_secs_f64() * 1000.0 + 1.15;
        self.last_rtt_ms = rtt_ms;

        let tx_hex = ModbusRtuFrame::format_hex(&tx_bytes);
        let rx_hex = ModbusRtuFrame::format_hex(&rx_bytes);

        match ModbusRtuFrame::decode(&rx_bytes) {
            Ok(decoded) => {
                self.rx_count += 1;
                self.record_frame_log(tx_hex, rx_hex, rtt_ms, true, description);
                Ok(decoded)
            }
            Err(err) => {
                if matches!(err, ModbusError::CrcMismatch { .. }) {
                    self.crc_errors += 1;
                }
                self.record_frame_log(tx_hex, rx_hex, rtt_ms, false, description);
                Err(err)
            }
        }
    }

    fn record_frame_log(
        &mut self,
        tx_hex: String,
        rx_hex: String,
        rtt_ms: f64,
        crc_ok: bool,
        description: &str,
    ) {
        if self.frame_history.len() >= 18 {
            self.frame_history.pop_front();
        }
        self.frame_history.push_back(Rs485FrameLog {
            timestamp_sec: self.start_instant.elapsed().as_secs_f64(),
            tx_hex,
            rx_hex,
            rtt_ms,
            crc_ok,
            description: description.to_string(),
        });
    }

    /// Synthesize an authentic Modbus RTU slave response packet with CRC-16 for internal loopback mode
    fn emulate_slave_response(&mut self, req: &ModbusRtuFrame) -> Vec<u8> {
        let elapsed = self.start_instant.elapsed().as_secs_f64();

        match (req.slave_id, req.function) {
            // Slave 0x01: Servo Drive Read Holding Registers (0x0010..0x0012)
            (1, ModbusFunction::ReadHoldingRegisters) => {
                let mut status_word: u16 = 0;
                if self.sim_limit_left {
                    status_word |= 1 << 0;
                }
                if self.sim_limit_right {
                    status_word |= 1 << 1;
                }
                if self.sim_is_homed {
                    status_word |= 1 << 2;
                }
                if self.sim_drive_fault {
                    status_word |= 1 << 3;
                }

                let pos_tenths = (self.sim_pos_mm.clamp(0.0, 6000.0) * 10.0).round() as u16;
                let vel_signed = self.sim_vel_mm_min.round() as i16 as u16;

                let mut payload = vec![6u8]; // byte count = 6 (3 registers)
                payload.extend_from_slice(&status_word.to_be_bytes());
                payload.extend_from_slice(&pos_tenths.to_be_bytes());
                payload.extend_from_slice(&vel_signed.to_be_bytes());

                ModbusRtuFrame {
                    slave_id: 1,
                    function: ModbusFunction::ReadHoldingRegisters,
                    payload,
                }
                .encode()
            }

            // Slave 0x02: Weld Power Supply Read Holding Registers (0x0030..0x0032)
            (2, ModbusFunction::ReadHoldingRegisters) => {
                let mut status_word: u16 = 0;
                if self.sim_arc_enabled {
                    status_word |= 1 << 0;
                }
                if self.sim_gas_enabled {
                    status_word |= 1 << 1;
                }
                if self.sim_left_clamp {
                    status_word |= 1 << 2;
                }
                if self.sim_right_clamp {
                    status_word |= 1 << 3;
                }
                if self.sim_over_current_fault {
                    status_word |= 1 << 4;
                }

                // Realistic high-frequency TIG/MIG seam arc ripple when the arc is active
                let (amps, volts) = if self.sim_over_current_fault {
                    (412.5, 34.8)
                } else if self.sim_arc_enabled {
                    let ripple_a = (elapsed * 37.0).sin() * 3.8 + (elapsed * 113.0).cos() * 1.6;
                    let ripple_v = (elapsed * 29.0).cos() * 0.45 + (elapsed * 79.0).sin() * 0.22;
                    let a = (self.sim_target_current_a + ripple_a).max(0.0);
                    // Characteristic arc voltage curve: V = 14.0 + 0.045 * I
                    let v = 14.0 + 0.045 * a + ripple_v;
                    (a, v)
                } else {
                    // Open-circuit / standby sensing voltage
                    let standby_v = if self.sim_gas_enabled { 68.0 } else { 0.2 };
                    (0.0, standby_v)
                };

                let amps_tenths = (amps * 10.0).round() as u16;
                let volts_tenths = (volts * 10.0).round() as u16;

                let mut payload = vec![6u8];
                payload.extend_from_slice(&status_word.to_be_bytes());
                payload.extend_from_slice(&amps_tenths.to_be_bytes());
                payload.extend_from_slice(&volts_tenths.to_be_bytes());

                ModbusRtuFrame {
                    slave_id: 2,
                    function: ModbusFunction::ReadHoldingRegisters,
                    payload,
                }
                .encode()
            }

            // Echo standard WriteSingleRegister (0x06) response
            (_, ModbusFunction::WriteSingleRegister) => req.encode(),
            _ => req.encode(),
        }
    }
}

impl LinearMotionAxis for IndustrialBusTransceiver {
    fn poll_telemetry(&mut self) -> Result<AxisTelemetry, ModbusError> {
        self.step_physics();
        let req = ModbusRtuFrame::read_holding_registers(
            self.config.serial.servo_slave_id,
            registers::SERVO_STATUS_WORD,
            3,
        );
        let resp = self.transact(&req, "Poll Servo [Pos/Vel/Limits]")?;
        if resp.payload.len() < 7 {
            return Err(ModbusError::FrameTooShort(resp.payload.len()));
        }
        let status = u16::from_be_bytes([resp.payload[1], resp.payload[2]]);
        let pos_tenths = u16::from_be_bytes([resp.payload[3], resp.payload[4]]);
        let vel_i16 = i16::from_be_bytes([resp.payload[5], resp.payload[6]]);

        Ok(AxisTelemetry {
            position_mm: (pos_tenths as f64) / 10.0,
            velocity_mm_min: vel_i16 as f64,
            limit_left_triggered: (status & (1 << 0)) != 0,
            limit_right_triggered: (status & (1 << 1)) != 0,
            is_homed: (status & (1 << 2)) != 0,
            drive_fault: (status & (1 << 3)) != 0,
        })
    }

    fn command_jog(&mut self, direction_right: bool, speed_mm_min: f64) -> Result<(), ModbusError> {
        let clamped_speed = speed_mm_min.clamp(0.0, self.config.safety.max_jog_speed_mm_min);
        self.sim_homing_active = false;
        self.sim_target_vel_mm_min = if direction_right {
            clamped_speed
        } else {
            -clamped_speed
        };
        let signed_val = (self.sim_target_vel_mm_min.round() as i16) as u16;
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.servo_slave_id,
            registers::SERVO_TARGET_SPEED,
            signed_val,
        );
        self.transact(&req, "Cmd Servo Jog Velocity")?;
        Ok(())
    }

    fn command_move_velocity(&mut self, speed_mm_min: f64) -> Result<(), ModbusError> {
        let clamped_speed = speed_mm_min.clamp(
            -self.config.safety.max_travel_speed_mm_min,
            self.config.safety.max_travel_speed_mm_min,
        );
        self.sim_homing_active = false;
        self.sim_target_vel_mm_min = clamped_speed;
        let signed_val = (clamped_speed.round() as i16) as u16;
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.servo_slave_id,
            registers::SERVO_TARGET_SPEED,
            signed_val,
        );
        self.transact(&req, "Cmd Servo Weld Velocity")?;
        Ok(())
    }

    fn command_home(&mut self) -> Result<(), ModbusError> {
        self.sim_homing_active = true;
        self.sim_is_homed = false;
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.servo_slave_id,
            registers::SERVO_CMD_WORD,
            0x0002,
        );
        self.transact(&req, "Cmd Servo Homing Sequence")?;
        Ok(())
    }

    fn stop_motion(&mut self) -> Result<(), ModbusError> {
        self.sim_homing_active = false;
        self.sim_target_vel_mm_min = 0.0;
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.servo_slave_id,
            registers::SERVO_TARGET_SPEED,
            0x0000,
        );
        self.transact(&req, "Cmd Servo Zero Velocity")?;
        Ok(())
    }

    fn emergency_halt(&mut self) -> Result<(), ModbusError> {
        self.sim_homing_active = false;
        self.sim_target_vel_mm_min = 0.0;
        self.sim_vel_mm_min = 0.0;
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.servo_slave_id,
            registers::SERVO_CMD_WORD,
            0x00FF,
        );
        self.transact(&req, "Cmd Servo EMERGENCY HALT")?;
        Ok(())
    }
}

impl WeldPowerSource for IndustrialBusTransceiver {
    fn poll_telemetry(&mut self) -> Result<WeldPowerTelemetry, ModbusError> {
        let req = ModbusRtuFrame::read_holding_registers(
            self.config.serial.welder_slave_id,
            registers::WELD_STATUS_WORD,
            3,
        );
        let resp = self.transact(&req, "Poll Welder [Amps/Volts/Clamps]")?;
        if resp.payload.len() < 7 {
            return Err(ModbusError::FrameTooShort(resp.payload.len()));
        }
        let status = u16::from_be_bytes([resp.payload[1], resp.payload[2]]);
        let amps_tenths = u16::from_be_bytes([resp.payload[3], resp.payload[4]]);
        let volts_tenths = u16::from_be_bytes([resp.payload[5], resp.payload[6]]);

        Ok(WeldPowerTelemetry {
            actual_current_amps: (amps_tenths as f64) / 10.0,
            actual_voltage_v: (volts_tenths as f64) / 10.0,
            wire_feed_speed_mm_min: if (status & 1) != 0 {
                self.sim_wire_feed_mm_min
            } else {
                0.0
            },
            arc_established: (status & (1 << 0)) != 0,
            gas_flow_active: (status & (1 << 1)) != 0,
            left_mandrel_clamped: (status & (1 << 2)) != 0,
            right_mandrel_clamped: (status & (1 << 3)) != 0,
            over_current_fault: (status & (1 << 4)) != 0,
        })
    }

    fn set_weld_schedule(
        &mut self,
        target_current_amps: f64,
        wire_feed_mm_min: f64,
    ) -> Result<(), ModbusError> {
        self.sim_target_current_a =
            target_current_amps.clamp(10.0, self.config.safety.max_weld_current_amps);
        self.sim_wire_feed_mm_min = wire_feed_mm_min.clamp(0.0, 8000.0);
        let tenths_a = (self.sim_target_current_a * 10.0).round() as u16;
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.welder_slave_id,
            registers::WELD_TARGET_CURRENT_TENTHS_A,
            tenths_a,
        );
        self.transact(&req, "Set Welder Target Current")?;
        Ok(())
    }

    fn set_shielding_gas(&mut self, enabled: bool) -> Result<(), ModbusError> {
        self.sim_gas_enabled = enabled;
        let val = u16::from(enabled);
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.welder_slave_id,
            registers::WELD_CMD_WORD,
            0x0100 | val,
        );
        self.transact(&req, "Set Argon Shielding Gas Solenoid")?;
        Ok(())
    }

    fn set_mandrel_clamps(
        &mut self,
        left_locked: bool,
        right_locked: bool,
    ) -> Result<(), ModbusError> {
        self.sim_left_clamp = left_locked;
        self.sim_right_clamp = right_locked;
        let mask = (u16::from(left_locked) << 1) | u16::from(right_locked);
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.welder_slave_id,
            registers::WELD_CMD_WORD,
            0x0200 | mask,
        );
        self.transact(&req, "Set Pneumatic Mandrel Clamps")?;
        Ok(())
    }

    fn set_arc_enable(&mut self, enabled: bool) -> Result<(), ModbusError> {
        self.sim_arc_enabled = enabled;
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.welder_slave_id,
            registers::WELD_CMD_WORD,
            if enabled { 0x0301 } else { 0x0300 },
        );
        self.transact(&req, if enabled { "Command HF Arc Strike" } else { "Extinguish Weld Arc" })?;
        Ok(())
    }

    fn clear_faults(&mut self) -> Result<(), ModbusError> {
        self.sim_drive_fault = false;
        self.sim_over_current_fault = false;
        self.sim_limit_left = false;
        self.sim_limit_right = false;
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.welder_slave_id,
            registers::WELD_CMD_WORD,
            0x00A5,
        );
        self.transact(&req, "Reset Hardware Fault Latches")?;
        Ok(())
    }

    fn emergency_shutdown(&mut self) -> Result<(), ModbusError> {
        self.sim_arc_enabled = false;
        self.sim_gas_enabled = false;
        let req = ModbusRtuFrame::write_single_register(
            self.config.serial.welder_slave_id,
            registers::WELD_CMD_WORD,
            0x00FF,
        );
        self.transact(&req, "Welder Contactor E-STOP Drop")?;
        Ok(())
    }
}

/// Spawn the asynchronous 50Hz real-time communications & safety watchdog worker
pub fn spawn_comms_worker(
    config: MachineConfig,
    force_serial_path: Option<String>,
    cmd_rx: Receiver<HmiCommand>,
    telemetry_tx: Sender<TelemetrySnapshot>,
) {
    std::thread::Builder::new()
        .name("rs485-realtime-50hz".to_string())
        .spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread()
                .enable_time()
                .build()
                .expect("Failed to initialize Tokio real-time runtime");

            rt.block_on(async move {
                run_50hz_control_loop(config, force_serial_path, cmd_rx, telemetry_tx).await;
            });
        })
        .expect("Failed to spawn RS-485 50Hz worker thread");
}

async fn run_50hz_control_loop(
    config: MachineConfig,
    force_serial_path: Option<String>,
    cmd_rx: Receiver<HmiCommand>,
    telemetry_tx: Sender<TelemetrySnapshot>,
) {
    let mut bus = IndustrialBusTransceiver::new(config.clone(), force_serial_path);
    let mut state = MachineState::Idle;
    let mut fault_reason: Option<String> = None;
    let mut state_entered_at = Instant::now();
    let mut last_jog_heartbeat = Instant::now();
    let mut watchdog_ok = true;

    let mut target_travel_speed_mm_min = 950.0;
    let mut target_wire_feed_mm_min = 3200.0;
    let mut target_current_amps = 185.0;

    let watchdog_timeout = Duration::from_millis(config.serial.watchdog_timeout_ms);
    let mut ticker = tokio::time::interval(Duration::from_millis(config.serial.poll_interval_ms));

    info!(
        "50Hz RS-485 Control & Safety Watchdog thread online (Watchdog timeout: {} ms)",
        config.serial.watchdog_timeout_ms
    );

    loop {
        ticker.tick().await;

        // 1. Drain all non-blocking GUI commands via lock-free channel
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                HmiCommand::EmergencyStop { reason } => {
                    error!("EMERGENCY STOP TRIGGERED: {}", reason);
                    let _ = bus.emergency_halt();
                    let _ = bus.emergency_shutdown();
                    state = MachineState::FaultEstop;
                    fault_reason = Some(reason);
                }
                HmiCommand::ResetFault => {
                    if state == MachineState::FaultEstop {
                        info!("Operator reset E-STOP / fault latch -> returning to IDLE");
                        let _ = bus.clear_faults();
                        let _ = bus.stop_motion();
                        state = MachineState::Idle;
                        fault_reason = None;
                        watchdog_ok = true;
                    }
                }
                HmiCommand::JogLeft { speed_mm_min } => {
                    if state == MachineState::Idle || state == MachineState::Jogging {
                        last_jog_heartbeat = Instant::now();
                        watchdog_ok = true;
                        state = MachineState::Jogging;
                        let _ = bus.command_jog(false, speed_mm_min);
                    }
                }
                HmiCommand::JogRight { speed_mm_min } => {
                    if state == MachineState::Idle || state == MachineState::Jogging {
                        last_jog_heartbeat = Instant::now();
                        watchdog_ok = true;
                        state = MachineState::Jogging;
                        let _ = bus.command_jog(true, speed_mm_min);
                    }
                }
                HmiCommand::JogHeartbeat => {
                    last_jog_heartbeat = Instant::now();
                    watchdog_ok = true;
                }
                HmiCommand::StopMotion => {
                    if state != MachineState::FaultEstop {
                        let _ = bus.stop_motion();
                        let _ = bus.set_arc_enable(false);
                        let _ = bus.set_shielding_gas(false);
                        state = MachineState::Idle;
                    }
                }
                HmiCommand::Home => {
                    if state == MachineState::Idle {
                        info!("Initiating automated carriage homing sequence");
                        let _ = bus.command_home();
                        state = MachineState::Homing;
                        state_entered_at = Instant::now();
                    }
                }
                HmiCommand::StartWeldSequence => {
                    if state == MachineState::Idle {
                        info!("Starting automated longitudinal seam weld sequence");
                        let _ = bus.set_mandrel_clamps(true, true);
                        let _ = bus.set_shielding_gas(true);
                        state = MachineState::Clamping;
                        state_entered_at = Instant::now();
                    }
                }
                HmiCommand::SetParameters {
                    travel_speed_mm_min,
                    wire_feed_mm_min,
                    target_current_amps: amps,
                } => {
                    target_travel_speed_mm_min =
                        travel_speed_mm_min.clamp(50.0, config.safety.max_travel_speed_mm_min);
                    target_wire_feed_mm_min = wire_feed_mm_min.clamp(100.0, 8000.0);
                    target_current_amps = amps.clamp(20.0, config.safety.max_weld_current_amps);
                    let _ = bus.set_weld_schedule(target_current_amps, target_wire_feed_mm_min);
                    if state == MachineState::WeldingSeam {
                        let _ = bus.command_move_velocity(target_travel_speed_mm_min);
                    }
                }
                HmiCommand::SetClamps {
                    left_locked,
                    right_locked,
                } => {
                    if state == MachineState::Idle {
                        let _ = bus.set_mandrel_clamps(left_locked, right_locked);
                    }
                }
                HmiCommand::SetShieldingGas { enabled } => {
                    if state == MachineState::Idle {
                        let _ = bus.set_shielding_gas(enabled);
                    }
                }
                HmiCommand::InjectFault(kind) => match kind {
                    FaultInjectionKind::OverCurrent => {
                        warn!("Technician Fault Injection: Simulating Primary Weld Over-Current (>375A)");
                        bus.sim_over_current_fault = true;
                    }
                    FaultInjectionKind::LimitSwitchHit => {
                        warn!("Technician Fault Injection: Simulating Hardware Over-Travel Limit Switch Hit");
                        bus.sim_limit_right = true;
                    }
                    FaultInjectionKind::Rs485CrcNoise => {
                        warn!("Technician Fault Injection: Injecting EMI burst onto RS-485 differential pair");
                        bus.inject_crc_noise_next = true;
                    }
                },
            }
        }

        // 2. Enforce 200ms Hardware Watchdog during manual Jogging
        let watchdog_elapsed = last_jog_heartbeat.elapsed();
        let watchdog_remaining_ms = if state == MachineState::Jogging {
            watchdog_timeout
                .checked_sub(watchdog_elapsed)
                .map(|d| d.as_millis() as u64)
                .unwrap_or(0)
        } else {
            config.serial.watchdog_timeout_ms
        };

        if state == MachineState::Jogging && watchdog_elapsed > watchdog_timeout {
            error!(
                "HARDWARE WATCHDOG TRIPPED: Jog heartbeat exceeded {} ms! Halting axis.",
                config.serial.watchdog_timeout_ms
            );
            watchdog_ok = false;
            let _ = bus.emergency_halt();
            let _ = bus.emergency_shutdown();
            state = MachineState::FaultEstop;
            fault_reason = Some(format!(
                "Watchdog Timeout ({}ms elapsed without GUI keep-alive)",
                watchdog_elapsed.as_millis()
            ));
        }

        // 3. Poll Servo Axis & Weld Power Supply over RS-485 Modbus RTU
        let axis_res = LinearMotionAxis::poll_telemetry(&mut bus);
        let weld_res = WeldPowerSource::poll_telemetry(&mut bus);

        if let Err(ref err) = axis_res {
            error!("RS-485 Servo Poll Fault: {}", err);
            let _ = bus.emergency_halt();
            let _ = bus.emergency_shutdown();
            state = MachineState::FaultEstop;
            fault_reason = Some(format!("Servo Bus Fault: {}", err));
        }
        if let Err(ref err) = weld_res {
            error!("RS-485 Welder Poll Fault: {}", err);
            let _ = bus.emergency_halt();
            let _ = bus.emergency_shutdown();
            state = MachineState::FaultEstop;
            fault_reason = Some(format!("Welder Bus Fault: {}", err));
        }

        let axis = axis_res.unwrap_or(AxisTelemetry {
            position_mm: bus.sim_pos_mm,
            velocity_mm_min: 0.0,
            limit_left_triggered: bus.sim_limit_left,
            limit_right_triggered: bus.sim_limit_right,
            is_homed: bus.sim_is_homed,
            drive_fault: true,
        });

        let weld = weld_res.unwrap_or(WeldPowerTelemetry {
            actual_current_amps: 0.0,
            actual_voltage_v: 0.0,
            wire_feed_speed_mm_min: 0.0,
            arc_established: false,
            gas_flow_active: false,
            left_mandrel_clamped: bus.sim_left_clamp,
            right_mandrel_clamped: bus.sim_right_clamp,
            over_current_fault: bus.sim_over_current_fault,
        });

        // 4. Evaluate Hardware Safety Interlocks (Over-Current & Hardware Limit Switches)
        if state != MachineState::FaultEstop {
            if weld.over_current_fault
                || weld.actual_current_amps > config.safety.over_current_trip_amps
            {
                let _ = bus.emergency_halt();
                let _ = bus.emergency_shutdown();
                state = MachineState::FaultEstop;
                fault_reason = Some(format!(
                    "Over-Current Trip: {:.1} A exceeded safety limit ({:.1} A)",
                    weld.actual_current_amps.max(412.5),
                    config.safety.over_current_trip_amps
                ));
            } else if axis.drive_fault {
                let _ = bus.emergency_halt();
                let _ = bus.emergency_shutdown();
                state = MachineState::FaultEstop;
                fault_reason = Some("Servo Carriage Amplifier Fault Flag (Bit 3) Tripped".to_string());
            } else if (axis.limit_left_triggered || axis.limit_right_triggered)
                && state != MachineState::Homing
            {
                let which = if axis.limit_left_triggered {
                    "LEFT (0mm)"
                } else {
                    "RIGHT (3000mm)"
                };
                let _ = bus.emergency_halt();
                let _ = bus.emergency_shutdown();
                state = MachineState::FaultEstop;
                fault_reason = Some(format!("Hardware Over-Travel Limit Switch Hit: {}", which));
            }
        }

        // 5. Step Automated Seam Welding State Machine
        match state {
            MachineState::Homing => {
                if axis.is_homed && axis.velocity_mm_min.abs() < 1.0 {
                    info!("Carriage homing complete at {:.1} mm", axis.position_mm);
                    state = MachineState::Idle;
                }
            }
            MachineState::Clamping => {
                if !weld.left_mandrel_clamped || !weld.right_mandrel_clamped {
                    let _ = bus.set_mandrel_clamps(true, true);
                }
                let clamp_wait = Duration::from_millis(
                    config.safety.clamping_dwell_ms + config.safety.min_gas_pre_flow_ms,
                );
                if state_entered_at.elapsed() >= clamp_wait {
                    info!("Mandrels clamped & Argon pre-flow complete -> Striking weld arc");
                    let _ = bus.set_weld_schedule(target_current_amps, target_wire_feed_mm_min);
                    let _ = bus.set_arc_enable(true);
                    state = MachineState::ArcStrike;
                    state_entered_at = Instant::now();
                }
            }
            MachineState::ArcStrike => {
                if state_entered_at.elapsed() >= Duration::from_millis(350) {
                    info!(
                        "Arc established ({:.1} A) -> Advancing longitudinal carriage at {:.0} mm/min",
                        weld.actual_current_amps, target_travel_speed_mm_min
                    );
                    let _ = bus.command_move_velocity(target_travel_speed_mm_min);
                    state = MachineState::WeldingSeam;
                    state_entered_at = Instant::now();
                }
            }
            MachineState::WeldingSeam => {
                if axis.position_mm >= config.machine.default_weld_end_mm {
                    info!(
                        "Reached seam weld end coordinate ({:.1} mm). Extinguishing arc & halting carriage.",
                        axis.position_mm
                    );
                    let _ = bus.stop_motion();
                    let _ = bus.set_arc_enable(false);
                    let _ = bus.set_shielding_gas(false);
                    state = MachineState::Idle;
                }
            }
            _ => {}
        }

        // 6. Publish non-blocking telemetry snapshot to GUI
        let snapshot = TelemetrySnapshot {
            timestamp_sec: bus.start_instant.elapsed().as_secs_f64(),
            state,
            active_fault_reason: fault_reason.clone(),
            bus_connected: true,
            bus_mode_label: bus.bus_mode_label(),
            bus_latency_ms: bus.last_rtt_ms,
            watchdog_ok,
            watchdog_remaining_ms,
            crc_error_count: bus.crc_errors,
            tx_frame_count: bus.tx_count,
            rx_frame_count: bus.rx_count,
            position_mm: axis.position_mm,
            velocity_mm_min: axis.velocity_mm_min,
            limit_left: axis.limit_left_triggered,
            limit_right: axis.limit_right_triggered,
            is_homed: axis.is_homed,
            actual_current_amps: weld.actual_current_amps,
            actual_voltage_v: weld.actual_voltage_v,
            wire_feed_speed_mm_min: weld.wire_feed_speed_mm_min,
            target_current_amps,
            target_travel_speed_mm_min,
            arc_active: weld.arc_established,
            gas_purging: weld.gas_flow_active,
            left_clamp_locked: weld.left_mandrel_clamped,
            right_clamp_locked: weld.right_mandrel_clamped,
            recent_frames: bus.frame_history.iter().cloned().collect(),
        };

        // Keep only latest telemetry in channel so GUI never lags behind real-time state
        let _ = telemetry_tx.try_send(snapshot);
    }
}
