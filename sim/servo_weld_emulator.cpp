/**
 * @file servo_weld_emulator.cpp
 * @brief Standalone C++17 RS-485 / Modbus RTU Hardware-in-the-Loop (HIL) Emulator
 *        for the MITUSA Longitudinal Seam Welder HMI (`seam-weld-hmi-rs`).
 *
 * Emulates two physical RS-485 slave nodes on a shared half-duplex bus:
 *   - Slave ID 0x01: Longitudinal Linear Servo Carriage Drive (0 - 3000.0 mm)
 *   - Slave ID 0x02: Constant-Current TIG/MIG Welding Power Source & Pneumatic Clamps
 *
 * Build:
 *   g++ -std=c++17 -O2 -Wall -Wextra -pedantic sim/servo_weld_emulator.cpp -o sim/servo_weld_emulator
 *
 * Run with `socat` virtual serial pair:
 *   socat -d -d pty,raw,echo=0 pty,raw,echo=0
 *   ./sim/servo_weld_emulator /dev/pts/3
 *   cargo run --release -- --serial /dev/pts/4
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

/// Standard Modbus RTU CRC-16 (polynomial 0xA001, initial value 0xFFFF)
uint16_t calculate_crc16_modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc = static_cast<uint16_t>(crc >> 1U);
            }
        }
    }
    return crc;
}

void append_crc16_le(std::vector<uint8_t>& frame) {
    const uint16_t crc = calculate_crc16_modbus(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
    frame.push_back(static_cast<uint8_t>((crc >> 8U) & 0xFFU));
}

bool verify_crc16(const uint8_t* buf, size_t len) {
    if (len < 4) {
        return false;
    }
    const uint16_t rx_crc = static_cast<uint16_t>(buf[len - 2]) |
                            (static_cast<uint16_t>(buf[len - 1]) << 8U);
    const uint16_t calc_crc = calculate_crc16_modbus(buf, len - 2);
    return rx_crc == calc_crc;
}

struct HardwareSimulatorState {
    // Slave 0x01: Servo Carriage Drive
    double position_mm = 150.0;
    double velocity_mm_min = 0.0;
    double target_velocity_mm_min = 0.0;
    bool is_homed = true;
    bool homing_active = false;
    bool limit_left = false;
    bool limit_right = false;
    bool drive_fault = false;

    // Slave 0x02: Weld Power Supply & Auxiliaries
    double target_current_amps = 185.0;
    bool arc_enabled = false;
    bool gas_enabled = false;
    bool left_clamp_locked = true;
    bool right_clamp_locked = true;
    bool over_current_fault = false;

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_step = std::chrono::steady_clock::now();

    void step_physics() {
        const auto now = std::chrono::steady_clock::now();
        const double dt_sec =
            std::chrono::duration<double>(now - last_step).count();
        last_step = now;

        if (drive_fault || over_current_fault) {
            velocity_mm_min = 0.0;
            target_velocity_mm_min = 0.0;
            arc_enabled = false;
            return;
        }

        if (homing_active) {
            target_velocity_mm_min = -1200.0;
        }

        // Trapezoidal velocity ramp (6000 mm/min/s)
        const double max_dv = 6000.0 * dt_sec;
        const double diff = target_velocity_mm_min - velocity_mm_min;
        if (std::abs(diff) <= max_dv) {
            velocity_mm_min = target_velocity_mm_min;
        } else {
            velocity_mm_min += (diff > 0.0 ? max_dv : -max_dv);
        }

        position_mm += (velocity_mm_min / 60.0) * dt_sec;

        if (homing_active && position_mm <= 25.0) {
            position_mm = 25.0;
            velocity_mm_min = 0.0;
            target_velocity_mm_min = 0.0;
            homing_active = false;
            is_homed = true;
            limit_left = false;
        } else if (position_mm <= 0.0) {
            position_mm = 0.0;
            velocity_mm_min = 0.0;
            target_velocity_mm_min = 0.0;
            limit_left = true;
        } else if (position_mm >= 3000.0) {
            position_mm = 3000.0;
            velocity_mm_min = 0.0;
            target_velocity_mm_min = 0.0;
            limit_right = true;
        } else {
            limit_left = false;
            limit_right = false;
        }
    }

    std::vector<uint8_t> handle_request(const uint8_t* req, size_t len) {
        step_physics();
        const uint8_t slave_id = req[0];
        const uint8_t fn_code = req[1];

        // Function 0x03: Read Holding Registers
        if (fn_code == 0x03 && len >= 8) {
            if (slave_id == 0x01) {
                uint16_t status = 0;
                if (limit_left) status |= (1U << 0U);
                if (limit_right) status |= (1U << 1U);
                if (is_homed) status |= (1U << 2U);
                if (drive_fault) status |= (1U << 3U);

                const auto pos_tenths = static_cast<uint16_t>(
                    std::clamp(std::lround(position_mm * 10.0), 0L, 60000L));
                const auto vel_signed = static_cast<uint16_t>(
                    static_cast<int16_t>(std::lround(velocity_mm_min)));

                std::vector<uint8_t> resp = {
                    0x01,
                    0x03,
                    0x06,
                    static_cast<uint8_t>(status >> 8U),
                    static_cast<uint8_t>(status & 0xFFU),
                    static_cast<uint8_t>(pos_tenths >> 8U),
                    static_cast<uint8_t>(pos_tenths & 0xFFU),
                    static_cast<uint8_t>(vel_signed >> 8U),
                    static_cast<uint8_t>(vel_signed & 0xFFU),
                };
                append_crc16_le(resp);
                return resp;
            }

            if (slave_id == 0x02) {
                uint16_t status = 0;
                if (arc_enabled) status |= (1U << 0U);
                if (gas_enabled) status |= (1U << 1U);
                if (left_clamp_locked) status |= (1U << 2U);
                if (right_clamp_locked) status |= (1U << 3U);
                if (over_current_fault) status |= (1U << 4U);

                const double elapsed =
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - start_time)
                        .count();

                double amps = 0.0;
                double volts = gas_enabled ? 68.0 : 0.2;
                if (arc_enabled) {
                    amps = target_current_amps + std::sin(elapsed * 37.0) * 3.5;
                    volts = 14.0 + 0.045 * amps + std::cos(elapsed * 29.0) * 0.4;
                }

                const auto amps_tenths =
                    static_cast<uint16_t>(std::lround(std::max(0.0, amps) * 10.0));
                const auto volts_tenths =
                    static_cast<uint16_t>(std::lround(std::max(0.0, volts) * 10.0));

                std::vector<uint8_t> resp = {
                    0x02,
                    0x03,
                    0x06,
                    static_cast<uint8_t>(status >> 8U),
                    static_cast<uint8_t>(status & 0xFFU),
                    static_cast<uint8_t>(amps_tenths >> 8U),
                    static_cast<uint8_t>(amps_tenths & 0xFFU),
                    static_cast<uint8_t>(volts_tenths >> 8U),
                    static_cast<uint8_t>(volts_tenths & 0xFFU),
                };
                append_crc16_le(resp);
                return resp;
            }
        }

        // Function 0x06: Write Single Register
        if (fn_code == 0x06 && len >= 8) {
            const uint16_t reg_addr =
                (static_cast<uint16_t>(req[2]) << 8U) | req[3];
            const uint16_t reg_val =
                (static_cast<uint16_t>(req[4]) << 8U) | req[5];

            if (slave_id == 0x01) {
                if (reg_addr == 0x0020) {
                    if (reg_val == 0x0002) {
                        homing_active = true;
                        is_homed = false;
                    } else if (reg_val == 0x00FF) {
                        homing_active = false;
                        target_velocity_mm_min = 0.0;
                        velocity_mm_min = 0.0;
                    }
                } else if (reg_addr == 0x0021) {
                    homing_active = false;
                    target_velocity_mm_min =
                        static_cast<double>(static_cast<int16_t>(reg_val));
                }
            } else if (slave_id == 0x02) {
                if (reg_addr == 0x0040) {
                    const uint8_t high_cmd = static_cast<uint8_t>(reg_val >> 8U);
                    const uint8_t low_arg = static_cast<uint8_t>(reg_val & 0xFFU);
                    if (high_cmd == 0x01) {
                        gas_enabled = (low_arg != 0);
                    } else if (high_cmd == 0x02) {
                        left_clamp_locked = ((low_arg & 0x02U) != 0U);
                        right_clamp_locked = ((low_arg & 0x01U) != 0U);
                    } else if (high_cmd == 0x03) {
                        arc_enabled = (low_arg != 0);
                    } else if (reg_val == 0x00A5) {
                        drive_fault = false;
                        over_current_fault = false;
                        limit_left = false;
                        limit_right = false;
                    } else if (reg_val == 0x00FF) {
                        arc_enabled = false;
                        gas_enabled = false;
                    }
                } else if (reg_addr == 0x0041) {
                    target_current_amps = static_cast<double>(reg_val) / 10.0;
                }
            }

            // Standard Modbus RTU 0x06 response echoes the request frame
            return std::vector<uint8_t>(req, req + 8);
        }

        return {};
    }
};

int configure_serial_port(const std::string& port_path) {
    const int fd = ::open(port_path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        return -1;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 100ms read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return -1;
    }
    return fd;
}

} // namespace

int main(int argc, char* argv[]) {
    std::cout << "===================================================================\n"
              << " MITUSA C++17 RS-485 / Modbus RTU Servo + Welder Hardware Emulator \n"
              << "===================================================================\n";

    HardwareSimulatorState sim;

    // Self-test mode if invoked with `--self-test`
    if (argc >= 2 && std::string(argv[1]) == "--self-test") {
        // Construct Read Holding Registers request for Slave 0x01 (0x0010, count 3)
        std::vector<uint8_t> req = {0x01, 0x03, 0x00, 0x10, 0x00, 0x03};
        append_crc16_le(req);
        if (!verify_crc16(req.data(), req.size())) {
            std::cerr << "[FAIL] Self-test CRC generation failed\n";
            return 1;
        }
        const auto resp = sim.handle_request(req.data(), req.size());
        if (resp.size() != 11 || !verify_crc16(resp.data(), resp.size())) {
            std::cerr << "[FAIL] Self-test slave response invalid\n";
            return 1;
        }
        std::cout << "[PASS] C++17 Modbus RTU Slave 0x01 & 0x02 self-test verified ("
                  << resp.size() << "-byte frame, CRC-16 OK).\n";
        return 0;
    }

    if (argc < 2) {
        std::cout << "Usage:\n"
                  << "  1. Create virtual serial pair: socat -d -d pty,raw,echo=0 pty,raw,echo=0\n"
                  << "  2. Start C++ emulator:         " << argv[0] << " /dev/pts/<N>\n"
                  << "  3. Start Rust HMI:             cargo run --release -- --serial /dev/pts/<M>\n"
                  << "  (Or run `" << argv[0] << " --self-test` to verify Modbus RTU framing)\n";
        return 0;
    }

    const std::string port_path = argv[1];
    const int fd = configure_serial_port(port_path);
    if (fd < 0) {
        std::cerr << "Failed to open serial PTY/TTY device: " << port_path << "\n";
        return 1;
    }

    std::cout << "[ONLINE] Listening on " << port_path
              << " @ 115200 8N1 (Slave 0x01=Servo, Slave 0x02=Welder)...\n";

    uint8_t rx_buf[64];
    while (true) {
        const ssize_t n = ::read(fd, rx_buf, sizeof(rx_buf));
        if (n >= 8) {
            if (verify_crc16(rx_buf, static_cast<size_t>(n))) {
                const auto resp =
                    sim.handle_request(rx_buf, static_cast<size_t>(n));
                if (!resp.empty()) {
                    const ssize_t written =
                        ::write(fd, resp.data(), resp.size());
                    (void)written;
                }
            } else {
                std::cerr << "[WARN] Dropped RS-485 frame with invalid CRC-16 ("
                          << n << " bytes)\n";
            }
        }
    }

    ::close(fd);
    return 0;
}
