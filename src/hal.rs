use crc::{Crc, CRC_16_MODBUS};
use std::fmt;

pub const MODBUS_CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

/// Modbus RTU Function Codes used on the RS-485 industrial bus
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ModbusFunction {
    ReadHoldingRegisters = 0x03,
    WriteSingleRegister = 0x06,
    WriteMultipleRegisters = 0x10,
}

impl ModbusFunction {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x03 => Some(Self::ReadHoldingRegisters),
            0x06 => Some(Self::WriteSingleRegister),
            0x10 => Some(Self::WriteMultipleRegisters),
            _ => None,
        }
    }
}

/// Register Map for Slave 0x01 (Servo Carriage Drive) & Slave 0x02 (Weld Power Supply)
#[allow(dead_code)]
pub mod registers {
    // Servo Drive Holding Registers (Slave 0x01)
    pub const SERVO_STATUS_WORD: u16 = 0x0010;
    pub const SERVO_POS_TENTHS_MM: u16 = 0x0011;
    pub const SERVO_VEL_MM_MIN: u16 = 0x0012;
    pub const SERVO_CMD_WORD: u16 = 0x0020;
    pub const SERVO_TARGET_SPEED: u16 = 0x0021;

    // Weld Power Supply Holding Registers (Slave 0x02)
    pub const WELD_STATUS_WORD: u16 = 0x0030;
    pub const WELD_CURRENT_TENTHS_A: u16 = 0x0031;
    pub const WELD_VOLTAGE_TENTHS_V: u16 = 0x0032;
    pub const WELD_CMD_WORD: u16 = 0x0040;
    pub const WELD_TARGET_CURRENT_TENTHS_A: u16 = 0x0041;
    pub const WELD_WIRE_FEED_MM_MIN: u16 = 0x0042;
}

/// Low-level deterministic RS-485 / Modbus RTU Frame Builder & Parser
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModbusRtuFrame {
    pub slave_id: u8,
    pub function: ModbusFunction,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[allow(dead_code)]
pub enum ModbusError {
    FrameTooShort(usize),
    UnsupportedFunction(u8),
    ExceptionResponse { slave_id: u8, exception_code: u8 },
    CrcMismatch { expected: u16, calculated: u16 },
    BusTimeout,
    IoError(String),
}

impl fmt::Display for ModbusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FrameTooShort(len) => write!(f, "Modbus RTU frame too short ({} bytes)", len),
            Self::UnsupportedFunction(fc) => write!(f, "Unsupported Modbus function 0x{:02X}", fc),
            Self::ExceptionResponse {
                slave_id,
                exception_code,
            } => write!(
                f,
                "Slave 0x{:02X} returned Modbus exception 0x{:02X}",
                slave_id, exception_code
            ),
            Self::CrcMismatch {
                expected,
                calculated,
            } => write!(
                f,
                "RS-485 CRC-16 error (expected 0x{:04X}, calc 0x{:04X})",
                expected, calculated
            ),
            Self::BusTimeout => write!(f, "RS-485 bus response timeout"),
            Self::IoError(msg) => write!(f, "Serial I/O fault: {}", msg),
        }
    }
}

impl ModbusRtuFrame {
    /// Construct a Read Holding Registers (0x03) request frame
    pub fn read_holding_registers(slave_id: u8, start_addr: u16, count: u16) -> Self {
        let mut payload = Vec::with_capacity(4);
        payload.extend_from_slice(&start_addr.to_be_bytes());
        payload.extend_from_slice(&count.to_be_bytes());
        Self {
            slave_id,
            function: ModbusFunction::ReadHoldingRegisters,
            payload,
        }
    }

    /// Construct a Write Single Register (0x06) request frame
    pub fn write_single_register(slave_id: u8, reg_addr: u16, value: u16) -> Self {
        let mut payload = Vec::with_capacity(4);
        payload.extend_from_slice(&reg_addr.to_be_bytes());
        payload.extend_from_slice(&value.to_be_bytes());
        Self {
            slave_id,
            function: ModbusFunction::WriteSingleRegister,
            payload,
        }
    }

    /// Encode into raw wire bytes with little-endian CRC-16/MODBUS appended
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(2 + self.payload.len() + 2);
        buf.push(self.slave_id);
        buf.push(self.function as u8);
        buf.extend_from_slice(&self.payload);
        let crc = MODBUS_CRC.checksum(&buf);
        buf.extend_from_slice(&crc.to_le_bytes());
        buf
    }

    /// Parse and validate a raw RS-485 Modbus RTU frame, verifying CRC-16 integrity
    pub fn decode(raw: &[u8]) -> Result<Self, ModbusError> {
        if raw.len() < 4 {
            return Err(ModbusError::FrameTooShort(raw.len()));
        }
        let data_len = raw.len() - 2;
        let expected_crc = u16::from_le_bytes([raw[data_len], raw[data_len + 1]]);
        let calculated_crc = MODBUS_CRC.checksum(&raw[..data_len]);

        if expected_crc != calculated_crc {
            return Err(ModbusError::CrcMismatch {
                expected: expected_crc,
                calculated: calculated_crc,
            });
        }

        let slave_id = raw[0];
        let fc_byte = raw[1];

        if (fc_byte & 0x80) != 0 {
            let exc = if raw.len() > 2 { raw[2] } else { 0xFF };
            return Err(ModbusError::ExceptionResponse {
                slave_id,
                exception_code: exc,
            });
        }

        let function =
            ModbusFunction::from_u8(fc_byte).ok_or(ModbusError::UnsupportedFunction(fc_byte))?;

        Ok(Self {
            slave_id,
            function,
            payload: raw[2..data_len].to_vec(),
        })
    }

    /// Format raw frame bytes as uppercase hex string `[01 03 00 10 00 03 04 0E]`
    pub fn format_hex(bytes: &[u8]) -> String {
        let inner: Vec<String> = bytes.iter().map(|b| format!("{:02X}", b)).collect();
        format!("[{}]", inner.join(" "))
    }
}

/// High-level state of the longitudinal linear servo carriage axis
#[derive(Debug, Clone, Copy)]
pub struct AxisTelemetry {
    pub position_mm: f64,
    pub velocity_mm_min: f64,
    pub limit_left_triggered: bool,
    pub limit_right_triggered: bool,
    pub is_homed: bool,
    pub drive_fault: bool,
}

/// High-level state of the welding power supply, mandrel clamps, and gas solenoid
#[derive(Debug, Clone, Copy)]
pub struct WeldPowerTelemetry {
    pub actual_current_amps: f64,
    pub actual_voltage_v: f64,
    pub wire_feed_speed_mm_min: f64,
    pub arc_established: bool,
    pub gas_flow_active: bool,
    pub left_mandrel_clamped: bool,
    pub right_mandrel_clamped: bool,
    pub over_current_fault: bool,
}

/// Hardware Abstraction Layer trait for the Longitudinal Linear Carriage Axis
pub trait LinearMotionAxis: Send {
    fn poll_telemetry(&mut self) -> Result<AxisTelemetry, ModbusError>;
    fn command_jog(&mut self, direction_right: bool, speed_mm_min: f64) -> Result<(), ModbusError>;
    fn command_move_velocity(&mut self, speed_mm_min: f64) -> Result<(), ModbusError>;
    fn command_home(&mut self) -> Result<(), ModbusError>;
    fn stop_motion(&mut self) -> Result<(), ModbusError>;
    fn emergency_halt(&mut self) -> Result<(), ModbusError>;
}

/// Hardware Abstraction Layer trait for the Welding Power Source & Pneumatic Auxiliaries
pub trait WeldPowerSource: Send {
    fn poll_telemetry(&mut self) -> Result<WeldPowerTelemetry, ModbusError>;
    fn set_weld_schedule(
        &mut self,
        target_current_amps: f64,
        wire_feed_mm_min: f64,
    ) -> Result<(), ModbusError>;
    fn set_shielding_gas(&mut self, enabled: bool) -> Result<(), ModbusError>;
    fn set_mandrel_clamps(&mut self, left_locked: bool, right_locked: bool)
        -> Result<(), ModbusError>;
    fn set_arc_enable(&mut self, enabled: bool) -> Result<(), ModbusError>;
    fn clear_faults(&mut self) -> Result<(), ModbusError>;
    fn emergency_shutdown(&mut self) -> Result<(), ModbusError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_modbus_rtu_crc_and_framing() {
        let req = ModbusRtuFrame::read_holding_registers(0x01, 0x0010, 0x0002);
        let raw = req.encode();
        // Slave 01, FC 03, Addr 00 10, Count 00 02 -> CRC 0xCEC5 (little-endian C5 CE)
        assert_eq!(raw, vec![0x01, 0x03, 0x00, 0x10, 0x00, 0x02, 0xC5, 0xCE]);

        let decoded = ModbusRtuFrame::decode(&raw).expect("valid frame should decode");
        assert_eq!(decoded, req);

        // Corrupt a bit and verify deterministic CRC-16 rejection
        let mut corrupted = raw.clone();
        corrupted[3] ^= 0x01;
        assert!(matches!(
            ModbusRtuFrame::decode(&corrupted),
            Err(ModbusError::CrcMismatch { .. })
        ));
    }
}
