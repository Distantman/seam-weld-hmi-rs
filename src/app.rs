use crate::comms::{FaultInjectionKind, HmiCommand, MachineState, TelemetrySnapshot};
use crate::config::MachineConfig;
use crossbeam_channel::{Receiver, Sender};
use eframe::egui::{
    self, Align2, Color32, FontFamily, FontId, Pos2, Rect, RichText, Rounding, Sense, Stroke, Vec2,
};
use egui_plot::{Legend, Line, Plot, PlotPoints};
use std::collections::VecDeque;

const MAX_PLOT_SAMPLES: usize = 360;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BottomTab {
    Oscilloscope,
    DiagnosticsAndFaults,
}

pub struct SeamWelderHmiApp {
    config: MachineConfig,
    cmd_tx: Sender<HmiCommand>,
    telemetry_rx: Receiver<TelemetrySnapshot>,
    latest: TelemetrySnapshot,
    // Operator setpoints
    set_travel_speed_mm_min: f64,
    set_wire_feed_mm_min: f64,
    set_weld_current_amps: f64,
    set_jog_speed_mm_min: f64,
    // Jog state tracking for watchdog heartbeat
    was_jogging_last_frame: bool,
    suppress_watchdog_heartbeat: bool,
    // Rolling 60 FPS time-series telemetry history
    history_current: VecDeque<[f64; 2]>,
    history_voltage: VecDeque<[f64; 2]>,
    history_velocity_scaled: VecDeque<[f64; 2]>,
    active_tab: BottomTab,
}

impl SeamWelderHmiApp {
    pub fn new(
        cc: &eframe::CreationContext<'_>,
        config: MachineConfig,
        cmd_tx: Sender<HmiCommand>,
        telemetry_rx: Receiver<TelemetrySnapshot>,
    ) -> Self {
        Self::configure_industrial_dark_theme(&cc.egui_ctx);

        let initial_snapshot = TelemetrySnapshot {
            timestamp_sec: 0.0,
            state: MachineState::Idle,
            active_fault_reason: None,
            bus_connected: true,
            bus_mode_label: "INITIALIZING RS-485 BUS...".to_string(),
            bus_latency_ms: 1.2,
            watchdog_ok: true,
            watchdog_remaining_ms: config.serial.watchdog_timeout_ms,
            crc_error_count: 0,
            tx_frame_count: 0,
            rx_frame_count: 0,
            position_mm: config.machine.default_weld_start_mm,
            velocity_mm_min: 0.0,
            limit_left: false,
            limit_right: false,
            is_homed: true,
            actual_current_amps: 0.0,
            actual_voltage_v: 0.2,
            wire_feed_speed_mm_min: 0.0,
            target_current_amps: 185.0,
            target_travel_speed_mm_min: 950.0,
            arc_active: false,
            gas_purging: false,
            left_clamp_locked: true,
            right_clamp_locked: true,
            recent_frames: Vec::new(),
        };

        Self {
            config,
            cmd_tx,
            telemetry_rx,
            latest: initial_snapshot,
            set_travel_speed_mm_min: 950.0,
            set_wire_feed_mm_min: 3200.0,
            set_weld_current_amps: 185.0,
            set_jog_speed_mm_min: 1100.0,
            was_jogging_last_frame: false,
            suppress_watchdog_heartbeat: false,
            history_current: VecDeque::with_capacity(MAX_PLOT_SAMPLES),
            history_voltage: VecDeque::with_capacity(MAX_PLOT_SAMPLES),
            history_velocity_scaled: VecDeque::with_capacity(MAX_PLOT_SAMPLES),
            active_tab: BottomTab::Oscilloscope,
        }
    }

    fn configure_industrial_dark_theme(ctx: &egui::Context) {
        let mut visuals = egui::Visuals::dark();
        visuals.panel_fill = Color32::from_rgb(13, 17, 23);
        visuals.window_fill = Color32::from_rgb(18, 24, 32);
        visuals.extreme_bg_color = Color32::from_rgb(9, 12, 16);
        visuals.widgets.noninteractive.bg_fill = Color32::from_rgb(24, 31, 42);
        visuals.widgets.inactive.bg_fill = Color32::from_rgb(30, 41, 59);
        visuals.widgets.hovered.bg_fill = Color32::from_rgb(45, 62, 88);
        visuals.widgets.active.bg_fill = Color32::from_rgb(59, 130, 246);
        visuals.widgets.noninteractive.rounding = Rounding::same(6.0);
        visuals.widgets.inactive.rounding = Rounding::same(6.0);
        visuals.widgets.hovered.rounding = Rounding::same(6.0);
        visuals.widgets.active.rounding = Rounding::same(6.0);
        ctx.set_visuals(visuals);
    }

    fn ingest_telemetry(&mut self) {
        while let Ok(snap) = self.telemetry_rx.try_recv() {
            let t = snap.timestamp_sec;
            if self.history_current.len() >= MAX_PLOT_SAMPLES {
                self.history_current.pop_front();
                self.history_voltage.pop_front();
                self.history_velocity_scaled.pop_front();
            }
            self.history_current.push_back([t, snap.actual_current_amps]);
            self.history_voltage.push_back([t, snap.actual_voltage_v]);
            // Scale velocity by 0.1 on the shared chart so 1000 mm/min plots at 100.0 units alongside Amps
            self.history_velocity_scaled
                .push_back([t, snap.velocity_mm_min * 0.1]);
            self.latest = snap;
        }
    }

    fn render_top_safety_header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.set_min_height(56.0);

            // Brand & Machine Title
            ui.vertical(|ui| {
                ui.label(
                    RichText::new("MITUSA INDUSTRIAL AUTOMATION")
                        .font(FontId::new(11.0, FontFamily::Monospace))
                        .color(Color32::from_rgb(148, 163, 184)),
                );
                ui.label(
                    RichText::new(&self.config.machine.model_name)
                        .font(FontId::new(16.5, FontFamily::Proportional))
                        .strong()
                        .color(Color32::from_rgb(241, 245, 249)),
                );
            });

            ui.add_space(12.0);
            ui.separator();
            ui.add_space(8.0);

            // RS-485 Bus Status & Watchdog LED
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    let bus_color = if self.latest.bus_connected && self.latest.crc_error_count == 0
                    {
                        Color32::from_rgb(34, 197, 94)
                    } else {
                        Color32::from_rgb(250, 204, 21)
                    };
                    ui.colored_label(bus_color, "●");
                    ui.label(
                        RichText::new(format!(
                            "RS-485 BUS {} ({:.2} ms)",
                            if self.latest.bus_connected {
                                "OK"
                            } else {
                                "FAULT"
                            },
                            self.latest.bus_latency_ms
                        ))
                        .font(FontId::new(12.0, FontFamily::Monospace))
                        .strong()
                        .color(Color32::from_rgb(226, 232, 240)),
                    );
                });

                ui.horizontal(|ui| {
                    let pulse = ((self.latest.timestamp_sec * 6.0).sin() > 0.0)
                        && self.latest.watchdog_ok;
                    let wd_color = if !self.latest.watchdog_ok {
                        Color32::from_rgb(239, 68, 68)
                    } else if pulse {
                        Color32::from_rgb(56, 189, 248)
                    } else {
                        Color32::from_rgb(2, 132, 199)
                    };
                    ui.colored_label(wd_color, "◆");
                    ui.label(
                        RichText::new(format!(
                            "WDT: {} ms | CRC ERR: {}",
                            self.latest.watchdog_remaining_ms, self.latest.crc_error_count
                        ))
                        .font(FontId::new(11.5, FontFamily::Monospace))
                        .color(Color32::from_rgb(148, 163, 184)),
                    );
                });
            });

            ui.add_space(8.0);
            ui.separator();
            ui.add_space(8.0);

            // Machine State Machine Badge
            let (badge_bg, badge_fg) = match self.latest.state {
                MachineState::Idle => (
                    Color32::from_rgb(30, 58, 138),
                    Color32::from_rgb(191, 219, 254),
                ),
                MachineState::Homing | MachineState::Jogging => (
                    Color32::from_rgb(113, 63, 18),
                    Color32::from_rgb(254, 240, 138),
                ),
                MachineState::Clamping | MachineState::ArcStrike => (
                    Color32::from_rgb(124, 45, 18),
                    Color32::from_rgb(254, 215, 170),
                ),
                MachineState::WeldingSeam => (
                    Color32::from_rgb(6, 95, 70),
                    Color32::from_rgb(167, 243, 208),
                ),
                MachineState::FaultEstop => (
                    Color32::from_rgb(153, 27, 27),
                    Color32::from_rgb(254, 202, 202),
                ),
            };

            egui::Frame::none()
                .fill(badge_bg)
                .rounding(Rounding::same(6.0))
                .stroke(Stroke::new(1.5_f32, badge_fg))
                .inner_margin(egui::Margin::symmetric(14.0, 8.0))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new(format!("STATE: {}", self.latest.state.label()))
                            .font(FontId::new(15.0, FontFamily::Monospace))
                            .strong()
                            .color(badge_fg),
                    );
                });

            // Right-aligned Giant Emergency Stop / Reset Fault Button
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if self.latest.state == MachineState::FaultEstop {
                    let reset_btn = egui::Button::new(
                        RichText::new("⟳ RESET FAULT / CLEAR E-STOP")
                            .font(FontId::new(16.0, FontFamily::Proportional))
                            .strong()
                            .color(Color32::from_rgb(15, 23, 42)),
                    )
                    .fill(Color32::from_rgb(250, 204, 21))
                    .min_size(Vec2::new(260.0, 48.0));

                    if ui.add(reset_btn).clicked() {
                        self.suppress_watchdog_heartbeat = false;
                        let _ = self.cmd_tx.send(HmiCommand::ResetFault);
                    }
                } else {
                    let estop_btn = egui::Button::new(
                        RichText::new("🛑 EMERGENCY STOP (E-STOP)")
                            .font(FontId::new(16.5, FontFamily::Proportional))
                            .strong()
                            .color(Color32::WHITE),
                    )
                    .fill(Color32::from_rgb(220, 38, 38))
                    .stroke(Stroke::new(2.0_f32, Color32::from_rgb(254, 202, 202)))
                    .min_size(Vec2::new(260.0, 48.0));

                    if ui.add(estop_btn).clicked() {
                        let _ = self.cmd_tx.send(HmiCommand::EmergencyStop {
                            reason: "Operator Touch E-STOP Activated".to_string(),
                        });
                    }
                }
            });
        });

        // Fault Banner if in FAULT_ESTOP state
        if let Some(ref reason) = self.latest.active_fault_reason {
            ui.add_space(4.0);
            egui::Frame::none()
                .fill(Color32::from_rgb(127, 29, 29))
                .stroke(Stroke::new(1.5_f32, Color32::from_rgb(248, 113, 113)))
                .rounding(Rounding::same(5.0))
                .inner_margin(egui::Margin::symmetric(10.0, 5.0))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!("⚠ ACTIVE SAFETY INTERLOCK TRIP: {}", reason))
                                .font(FontId::new(13.5, FontFamily::Monospace))
                                .strong()
                                .color(Color32::from_rgb(254, 226, 226)),
                        );
                    });
                });
        }
    }

    /// Custom-drawn 2D schematic of the 3000mm linear seam weld rail, mandrel clamps, limit switches, and torch
    fn render_carriage_schematic(&mut self, ui: &mut egui::Ui) {
        egui::Frame::none()
            .fill(Color32::from_rgb(16, 22, 31))
            .stroke(Stroke::new(1.0_f32, Color32::from_rgb(45, 58, 77)))
            .rounding(Rounding::same(8.0))
            .inner_margin(egui::Margin::same(10.0))
            .show(ui, |ui| {
                // Header row inside schematic card + Pneumatic Clamp & Shielding Gas controls
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new("LONGITUDINAL SEAM RAIL & MANDREL SCHEMATIC (0 – 3000 mm)")
                            .font(FontId::new(13.0, FontFamily::Monospace))
                            .strong()
                            .color(Color32::from_rgb(148, 163, 184)),
                    );

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        // Shielding Gas Solenoid Toggle
                        let gas_label = if self.latest.gas_purging {
                            "ARGON GAS: PURGING (25 CFH)"
                        } else {
                            "ARGON GAS: OFF"
                        };
                        let gas_color = if self.latest.gas_purging {
                            Color32::from_rgb(14, 165, 233)
                        } else {
                            Color32::from_rgb(51, 65, 85)
                        };
                        if ui
                            .add(
                                egui::Button::new(
                                    RichText::new(gas_label)
                                        .font(FontId::new(11.5, FontFamily::Monospace))
                                        .strong()
                                        .color(Color32::WHITE),
                                )
                                .fill(gas_color)
                                .min_size(Vec2::new(190.0, 28.0)),
                            )
                            .clicked()
                        {
                            let _ = self.cmd_tx.send(HmiCommand::SetShieldingGas {
                                enabled: !self.latest.gas_purging,
                            });
                        }

                        // Right Mandrel Clamp Toggle
                        let r_label = if self.latest.right_clamp_locked {
                            "R-CLAMP: LOCKED"
                        } else {
                            "R-CLAMP: OPEN"
                        };
                        let r_fill = if self.latest.right_clamp_locked {
                            Color32::from_rgb(22, 163, 74)
                        } else {
                            Color32::from_rgb(180, 83, 9)
                        };
                        if ui
                            .add(
                                egui::Button::new(
                                    RichText::new(r_label)
                                        .font(FontId::new(11.5, FontFamily::Monospace))
                                        .strong()
                                        .color(Color32::WHITE),
                                )
                                .fill(r_fill)
                                .min_size(Vec2::new(135.0, 28.0)),
                            )
                            .clicked()
                        {
                            let _ = self.cmd_tx.send(HmiCommand::SetClamps {
                                left_locked: self.latest.left_clamp_locked,
                                right_locked: !self.latest.right_clamp_locked,
                            });
                        }

                        // Left Mandrel Clamp Toggle
                        let l_label = if self.latest.left_clamp_locked {
                            "L-CLAMP: LOCKED"
                        } else {
                            "L-CLAMP: OPEN"
                        };
                        let l_fill = if self.latest.left_clamp_locked {
                            Color32::from_rgb(22, 163, 74)
                        } else {
                            Color32::from_rgb(180, 83, 9)
                        };
                        if ui
                            .add(
                                egui::Button::new(
                                    RichText::new(l_label)
                                        .font(FontId::new(11.5, FontFamily::Monospace))
                                        .strong()
                                        .color(Color32::WHITE),
                                )
                                .fill(l_fill)
                                .min_size(Vec2::new(135.0, 28.0)),
                            )
                            .clicked()
                        {
                            let _ = self.cmd_tx.send(HmiCommand::SetClamps {
                                left_locked: !self.latest.left_clamp_locked,
                                right_locked: self.latest.right_clamp_locked,
                            });
                        }
                    });
                });

                ui.add_space(6.0);

                // Allocate 2D Painter Canvas
                let desired_size = Vec2::new(ui.available_width(), 122.0);
                let (rect, _response) = ui.allocate_exact_size(desired_size, Sense::hover());
                let painter = ui.painter_at(rect);

                // Dark Blueprint Grid Background
                painter.rect_filled(rect, Rounding::same(6.0), Color32::from_rgb(10, 14, 20));
                let grid_step = 40.0;
                let mut gx = rect.left() + grid_step;
                while gx < rect.right() {
                    painter.line_segment(
                        [Pos2::new(gx, rect.top()), Pos2::new(gx, rect.bottom())],
                        Stroke::new(1.0_f32, Color32::from_rgb(18, 25, 36)),
                    );
                    gx += grid_step;
                }

                let pad_x = 46.0;
                let rail_left = rect.left() + pad_x;
                let rail_right = rect.right() - pad_x;
                let rail_width = (rail_right - rail_left).max(10.0);
                let rail_y = rect.top() + 36.0;
                let workpiece_y = rect.top() + 82.0;

                // Top Linear Precision Guideway Beam
                let beam_rect = Rect::from_min_max(
                    Pos2::new(rail_left, rail_y - 7.0),
                    Pos2::new(rail_right, rail_y + 7.0),
                );
                painter.rect_filled(beam_rect, Rounding::same(3.0), Color32::from_rgb(51, 65, 85));
                painter.rect_stroke(
                    beam_rect,
                    Rounding::same(3.0),
                    Stroke::new(1.0_f32, Color32::from_rgb(100, 116, 139)),
                );

                // Cylindrical Copper Backing Mandrel & Workpiece Seam
                let mandrel_rect = Rect::from_min_max(
                    Pos2::new(rail_left + 16.0, workpiece_y - 8.0),
                    Pos2::new(rail_right - 16.0, workpiece_y + 8.0),
                );
                painter.rect_filled(
                    mandrel_rect,
                    Rounding::same(3.0),
                    Color32::from_rgb(120, 53, 15),
                );

                // Draw Completed Weld Bead Trail along the seam
                let max_mm = self.config.machine.max_rail_length_mm;
                let start_ratio =
                    (self.config.machine.default_weld_start_mm / max_mm).clamp(0.0, 1.0) as f32;
                let curr_ratio = (self.latest.position_mm / max_mm).clamp(0.0, 1.0) as f32;
                let start_x = rail_left + start_ratio * rail_width;
                let torch_x = rail_left + curr_ratio * rail_width;

                if (self.latest.state == MachineState::WeldingSeam || self.latest.arc_active)
                    && torch_x > start_x
                {
                    painter.line_segment(
                        [
                            Pos2::new(start_x, workpiece_y),
                            Pos2::new(torch_x, workpiece_y),
                        ],
                        Stroke::new(5.0_f32, Color32::from_rgb(56, 189, 248)),
                    );
                }

                // Draw Left & Right Pneumatic Clamping Fingers
                let clamp_l_color = if self.latest.left_clamp_locked {
                    Color32::from_rgb(34, 197, 94)
                } else {
                    Color32::from_rgb(245, 158, 11)
                };
                let clamp_r_color = if self.latest.right_clamp_locked {
                    Color32::from_rgb(34, 197, 94)
                } else {
                    Color32::from_rgb(245, 158, 11)
                };
                painter.rect_filled(
                    Rect::from_min_max(
                        Pos2::new(rail_left + 20.0, workpiece_y - 16.0),
                        Pos2::new(rail_left + 120.0, workpiece_y - 9.0),
                    ),
                    Rounding::same(2.0),
                    clamp_l_color,
                );
                painter.rect_filled(
                    Rect::from_min_max(
                        Pos2::new(rail_right - 120.0, workpiece_y - 16.0),
                        Pos2::new(rail_right - 20.0, workpiece_y - 9.0),
                    ),
                    Rounding::same(2.0),
                    clamp_r_color,
                );

                // Left & Right Hardware Limit Switch Indicators
                let lim_l_color = if self.latest.limit_left {
                    Color32::from_rgb(239, 68, 68)
                } else {
                    Color32::from_rgb(34, 197, 94)
                };
                let lim_r_color = if self.latest.limit_right {
                    Color32::from_rgb(239, 68, 68)
                } else {
                    Color32::from_rgb(34, 197, 94)
                };
                painter.circle_filled(Pos2::new(rail_left - 18.0, rail_y), 8.0, lim_l_color);
                painter.text(
                    Pos2::new(rail_left - 18.0, rail_y + 18.0),
                    Align2::CENTER_CENTER,
                    "LIM-L",
                    FontId::new(10.0, FontFamily::Monospace),
                    Color32::from_rgb(148, 163, 184),
                );
                painter.circle_filled(Pos2::new(rail_right + 18.0, rail_y), 8.0, lim_r_color);
                painter.text(
                    Pos2::new(rail_right + 18.0, rail_y + 18.0),
                    Align2::CENTER_CENTER,
                    "LIM-R",
                    FontId::new(10.0, FontFamily::Monospace),
                    Color32::from_rgb(148, 163, 184),
                );

                // Ruler ticks along bottom of schematic
                for mm in (0..=3000).step_by(500) {
                    let frac = (mm as f32) / (max_mm as f32);
                    let tx = rail_left + frac * rail_width;
                    painter.line_segment(
                        [
                            Pos2::new(tx, rect.bottom() - 18.0),
                            Pos2::new(tx, rect.bottom() - 12.0),
                        ],
                        Stroke::new(1.0_f32, Color32::from_rgb(100, 116, 139)),
                    );
                    painter.text(
                        Pos2::new(tx, rect.bottom() - 6.0),
                        Align2::CENTER_CENTER,
                        format!("{}mm", mm),
                        FontId::new(10.0, FontFamily::Monospace),
                        Color32::from_rgb(148, 163, 184),
                    );
                }

                // Draw Servo Torch Carriage Assembly
                let carriage_rect = Rect::from_center_size(
                    Pos2::new(torch_x, rail_y),
                    Vec2::new(38.0, 26.0),
                );
                painter.rect_filled(
                    carriage_rect,
                    Rounding::same(4.0),
                    Color32::from_rgb(59, 130, 246),
                );
                painter.rect_stroke(
                    carriage_rect,
                    Rounding::same(4.0),
                    Stroke::new(1.5_f32, Color32::from_rgb(191, 219, 254)),
                );

                // Torch lance extending down toward copper mandrel seam
                painter.line_segment(
                    [
                        Pos2::new(torch_x, carriage_rect.bottom()),
                        Pos2::new(torch_x, workpiece_y - 6.0),
                    ],
                    Stroke::new(4.0_f32, Color32::from_rgb(203, 213, 225)),
                );

                // Shielding Gas Envelope Cone
                if self.latest.gas_purging {
                    painter.circle_stroke(
                        Pos2::new(torch_x, workpiece_y - 4.0),
                        14.0,
                        Stroke::new(1.5_f32, Color32::from_rgba_unmultiplied(56, 189, 248, 150)),
                    );
                }

                // Live Plasma Arc Glow when Welding
                if self.latest.arc_active {
                    painter.circle_filled(
                        Pos2::new(torch_x, workpiece_y - 3.0),
                        12.0,
                        Color32::from_rgba_unmultiplied(56, 189, 248, 180),
                    );
                    painter.circle_filled(
                        Pos2::new(torch_x, workpiece_y - 3.0),
                        6.0,
                        Color32::WHITE,
                    );
                }

                // Floating Digital Readout Tag above the Carriage
                painter.text(
                    Pos2::new(torch_x, rail_y - 20.0),
                    Align2::CENTER_CENTER,
                    format!(
                        "{:.1} mm {}",
                        self.latest.position_mm,
                        if self.latest.is_homed { "[HOMED]" } else { "[UNHOMED]" }
                    ),
                    FontId::new(11.5, FontFamily::Monospace),
                    Color32::from_rgb(250, 204, 21),
                );
            });
    }

    fn render_middle_operator_deck(&mut self, ui: &mut egui::Ui) {
        ui.columns(2, |cols| {
            // LEFT COLUMN: Gloved Operator Motion & Sequence Deck
            egui::Frame::none()
                .fill(Color32::from_rgb(16, 22, 31))
                .stroke(Stroke::new(1.0_f32, Color32::from_rgb(45, 58, 77)))
                .rounding(Rounding::same(8.0))
                .inner_margin(egui::Margin::same(10.0))
                .show(&mut cols[0], |ui| {
                    ui.label(
                        RichText::new("CARRIAGE MOTION & AUTOMATED SEQUENCE CONTROLS")
                            .font(FontId::new(12.5, FontFamily::Monospace))
                            .strong()
                            .color(Color32::from_rgb(148, 163, 184)),
                    );
                    ui.add_space(6.0);

                    // Primary Touch Buttons: JOG LEFT | AUTO-HOME | JOG RIGHT
                    let mut currently_jogging = false;
                    ui.horizontal(|ui| {
                        let btn_w = (ui.available_width() - 16.0) / 3.0;
                        let jog_left_resp = ui.add(
                            egui::Button::new(
                                RichText::new("◀ JOG LEFT\n(HOLD)")
                                    .font(FontId::new(14.0, FontFamily::Proportional))
                                    .strong()
                                    .color(Color32::WHITE),
                            )
                            .fill(Color32::from_rgb(30, 64, 175))
                            .min_size(Vec2::new(btn_w, 54.0)),
                        );

                        let home_resp = ui.add(
                            egui::Button::new(
                                RichText::new("⌂ AUTO-HOME\n(25.0 mm)")
                                    .font(FontId::new(14.0, FontFamily::Proportional))
                                    .strong()
                                    .color(Color32::WHITE),
                            )
                            .fill(Color32::from_rgb(51, 65, 85))
                            .min_size(Vec2::new(btn_w, 54.0)),
                        );

                        let jog_right_resp = ui.add(
                            egui::Button::new(
                                RichText::new("JOG RIGHT ▶\n(HOLD)")
                                    .font(FontId::new(14.0, FontFamily::Proportional))
                                    .strong()
                                    .color(Color32::WHITE),
                            )
                            .fill(Color32::from_rgb(30, 64, 175))
                            .min_size(Vec2::new(btn_w, 54.0)),
                        );

                        if jog_left_resp.is_pointer_button_down_on() {
                            currently_jogging = true;
                            if !self.was_jogging_last_frame {
                                let _ = self.cmd_tx.send(HmiCommand::JogLeft {
                                    speed_mm_min: self.set_jog_speed_mm_min,
                                });
                            } else if !self.suppress_watchdog_heartbeat {
                                let _ = self.cmd_tx.send(HmiCommand::JogHeartbeat);
                            }
                        } else if jog_right_resp.is_pointer_button_down_on() {
                            currently_jogging = true;
                            if !self.was_jogging_last_frame {
                                let _ = self.cmd_tx.send(HmiCommand::JogRight {
                                    speed_mm_min: self.set_jog_speed_mm_min,
                                });
                            } else if !self.suppress_watchdog_heartbeat {
                                let _ = self.cmd_tx.send(HmiCommand::JogHeartbeat);
                            }
                        }

                        if home_resp.clicked() {
                            let _ = self.cmd_tx.send(HmiCommand::Home);
                        }
                    });

                    // Automatically halt servo when operator releases the touch Jog button
                    if self.was_jogging_last_frame
                        && !currently_jogging
                        && !self.suppress_watchdog_heartbeat
                    {
                        let _ = self.cmd_tx.send(HmiCommand::StopMotion);
                    }
                    self.was_jogging_last_frame = currently_jogging;

                    ui.add_space(6.0);

                    // Start / Abort Automated Seam Weld Cycle
                    ui.horizontal(|ui| {
                        let w = (ui.available_width() - 8.0) * 0.68;
                        let start_btn = egui::Button::new(
                            RichText::new("⚡ START AUTOMATED SEAM WELD CYCLE")
                                .font(FontId::new(14.5, FontFamily::Proportional))
                                .strong()
                                .color(Color32::WHITE),
                        )
                        .fill(Color32::from_rgb(5, 150, 105))
                        .min_size(Vec2::new(w, 46.0));

                        if ui.add(start_btn).clicked() {
                            let _ = self.cmd_tx.send(HmiCommand::SetParameters {
                                travel_speed_mm_min: self.set_travel_speed_mm_min,
                                wire_feed_mm_min: self.set_wire_feed_mm_min,
                                target_current_amps: self.set_weld_current_amps,
                            });
                            let _ = self.cmd_tx.send(HmiCommand::StartWeldSequence);
                        }

                        let stop_btn = egui::Button::new(
                            RichText::new("⏹ STOP CYCLE")
                                .font(FontId::new(13.5, FontFamily::Proportional))
                                .strong()
                                .color(Color32::WHITE),
                        )
                        .fill(Color32::from_rgb(180, 83, 9))
                        .min_size(Vec2::new(ui.available_width(), 46.0));

                        if ui.add(stop_btn).clicked() {
                            let _ = self.cmd_tx.send(HmiCommand::StopMotion);
                        }
                    });
                });

            // RIGHT COLUMN: Weld Schedule Sliders & Digital Telemetry Readouts
            egui::Frame::none()
                .fill(Color32::from_rgb(16, 22, 31))
                .stroke(Stroke::new(1.0_f32, Color32::from_rgb(45, 58, 77)))
                .rounding(Rounding::same(8.0))
                .inner_margin(egui::Margin::same(10.0))
                .show(&mut cols[1], |ui| {
                    // Live Digital KPI Readout Strip
                    ui.columns(4, |kpis| {
                        Self::render_kpi_box(
                            &mut kpis[0],
                            "ACTUAL CURRENT",
                            &format!("{:.1} A", self.latest.actual_current_amps),
                            Color32::from_rgb(56, 189, 248),
                        );
                        Self::render_kpi_box(
                            &mut kpis[1],
                            "ARC VOLTAGE",
                            &format!("{:.1} V", self.latest.actual_voltage_v),
                            Color32::from_rgb(250, 204, 21),
                        );
                        Self::render_kpi_box(
                            &mut kpis[2],
                            "TRAVEL VELOCITY",
                            &format!("{:.0} mm/m", self.latest.velocity_mm_min),
                            Color32::from_rgb(52, 211, 153),
                        );
                        Self::render_kpi_box(
                            &mut kpis[3],
                            "WIRE FEED",
                            &format!("{:.0} mm/m", self.latest.wire_feed_speed_mm_min),
                            Color32::from_rgb(192, 132, 252),
                        );
                    });

                    ui.add_space(6.0);

                    // Touch Sliders for Weld Schedule Parameters
                    let mut changed = false;
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new("Weld Current (A):   ")
                                .font(FontId::new(12.0, FontFamily::Monospace)),
                        );
                        changed |= ui
                            .add(
                                egui::Slider::new(
                                    &mut self.set_weld_current_amps,
                                    30.0..=self.config.safety.max_weld_current_amps,
                                )
                                .step_by(5.0)
                                .suffix(" A"),
                            )
                            .changed();
                    });

                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new("Travel Speed (mm/m):")
                                .font(FontId::new(12.0, FontFamily::Monospace)),
                        );
                        changed |= ui
                            .add(
                                egui::Slider::new(
                                    &mut self.set_travel_speed_mm_min,
                                    150.0..=self.config.safety.max_travel_speed_mm_min,
                                )
                                .step_by(25.0)
                                .suffix(" mm/min"),
                            )
                            .changed();
                    });

                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new("Wire Feed (mm/m):   ")
                                .font(FontId::new(12.0, FontFamily::Monospace)),
                        );
                        changed |= ui
                            .add(
                                egui::Slider::new(&mut self.set_wire_feed_mm_min, 600.0..=6500.0)
                                    .step_by(50.0)
                                    .suffix(" mm/min"),
                            )
                            .changed();
                    });

                    if changed {
                        let _ = self.cmd_tx.send(HmiCommand::SetParameters {
                            travel_speed_mm_min: self.set_travel_speed_mm_min,
                            wire_feed_mm_min: self.set_wire_feed_mm_min,
                            target_current_amps: self.set_weld_current_amps,
                        });
                    }
                });
        });
    }

    fn render_kpi_box(ui: &mut egui::Ui, title: &str, value: &str, accent: Color32) {
        egui::Frame::none()
            .fill(Color32::from_rgb(10, 14, 20))
            .stroke(Stroke::new(1.0_f32, Color32::from_rgb(30, 41, 59)))
            .rounding(Rounding::same(5.0))
            .inner_margin(egui::Margin::symmetric(6.0, 4.0))
            .show(ui, |ui| {
                ui.label(
                    RichText::new(title)
                        .font(FontId::new(9.5, FontFamily::Monospace))
                        .color(Color32::from_rgb(148, 163, 184)),
                );
                ui.label(
                    RichText::new(value)
                        .font(FontId::new(15.0, FontFamily::Monospace))
                        .strong()
                        .color(accent),
                );
            });
    }

    fn render_bottom_telemetry_and_diagnostics(&mut self, ui: &mut egui::Ui) {
        egui::Frame::none()
            .fill(Color32::from_rgb(16, 22, 31))
            .stroke(Stroke::new(1.0_f32, Color32::from_rgb(45, 58, 77)))
            .rounding(Rounding::same(8.0))
            .inner_margin(egui::Margin::same(8.0))
            .show(ui, |ui| {
                // Tab Switcher + Quick Fault Injection Bar
                ui.horizontal(|ui| {
                    let osc_selected = self.active_tab == BottomTab::Oscilloscope;
                    if ui
                        .selectable_label(
                            osc_selected,
                            RichText::new("📈 60 FPS REAL-TIME TELEMETRY OSCILLOSCOPE")
                                .font(FontId::new(12.0, FontFamily::Monospace))
                                .strong(),
                        )
                        .clicked()
                    {
                        self.active_tab = BottomTab::Oscilloscope;
                    }

                    let diag_selected = self.active_tab == BottomTab::DiagnosticsAndFaults;
                    if ui
                        .selectable_label(
                            diag_selected,
                            RichText::new(format!(
                                "🛠 RS-485 HEX INSPECTOR & FAULT INJECTION (TX:{} / RX:{})",
                                self.latest.tx_frame_count, self.latest.rx_frame_count
                            ))
                            .font(FontId::new(12.0, FontFamily::Monospace))
                            .strong(),
                        )
                        .clicked()
                    {
                        self.active_tab = BottomTab::DiagnosticsAndFaults;
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .add(
                                egui::Button::new(
                                    RichText::new("⚡ Inject CRC Noise")
                                        .font(FontId::new(11.0, FontFamily::Monospace))
                                        .color(Color32::from_rgb(254, 240, 138)),
                                )
                                .fill(Color32::from_rgb(113, 63, 18)),
                            )
                            .clicked()
                        {
                            let _ = self
                                .cmd_tx
                                .send(HmiCommand::InjectFault(FaultInjectionKind::Rs485CrcNoise));
                        }

                        if ui
                            .add(
                                egui::Button::new(
                                    RichText::new("⚠ Simulate Limit Hit")
                                        .font(FontId::new(11.0, FontFamily::Monospace))
                                        .color(Color32::from_rgb(254, 215, 170)),
                                )
                                .fill(Color32::from_rgb(124, 45, 18)),
                            )
                            .clicked()
                        {
                            let _ = self
                                .cmd_tx
                                .send(HmiCommand::InjectFault(FaultInjectionKind::LimitSwitchHit));
                        }

                        if ui
                            .add(
                                egui::Button::new(
                                    RichText::new("🔥 Simulate Over-Current")
                                        .font(FontId::new(11.0, FontFamily::Monospace))
                                        .color(Color32::from_rgb(254, 202, 202)),
                                )
                                .fill(Color32::from_rgb(127, 29, 29)),
                            )
                            .clicked()
                        {
                            let _ = self
                                .cmd_tx
                                .send(HmiCommand::InjectFault(FaultInjectionKind::OverCurrent));
                        }
                    });
                });

                ui.add_space(4.0);

                match self.active_tab {
                    BottomTab::Oscilloscope => {
                        let current_pts: PlotPoints =
                            self.history_current.iter().copied().collect();
                        let voltage_pts: PlotPoints =
                            self.history_voltage.iter().copied().collect();
                        let vel_pts: PlotPoints =
                            self.history_velocity_scaled.iter().copied().collect();

                        Plot::new("weld_telemetry_oscilloscope")
                            .legend(Legend::default())
                            .height(ui.available_height().max(115.0))
                            .include_y(0.0)
                            .include_y(260.0)
                            .show(ui, |plot_ui| {
                                plot_ui.line(
                                    Line::new(current_pts)
                                        .name("Weld Current (Amps)")
                                        .color(Color32::from_rgb(56, 189, 248))
                                        .width(2.0_f32),
                                );
                                plot_ui.line(
                                    Line::new(voltage_pts)
                                        .name("Arc Voltage (V)")
                                        .color(Color32::from_rgb(250, 204, 21))
                                        .width(1.8_f32),
                                );
                                plot_ui.line(
                                    Line::new(vel_pts)
                                        .name("Carriage Velocity (×10 mm/min)")
                                        .color(Color32::from_rgb(52, 211, 153))
                                        .width(1.6_f32),
                                );
                            });
                    }
                    BottomTab::DiagnosticsAndFaults => {
                        let scroll_h = ui.available_height().max(105.0);
                        ui.columns(2, |cols| {
                            // Left side: Live scrolling RS-485 Modbus RTU hex frame log
                            cols[0].label(
                                RichText::new(format!("BUS INTERFACE: {}", self.latest.bus_mode_label))
                                    .font(FontId::new(11.0, FontFamily::Monospace))
                                    .color(Color32::from_rgb(56, 189, 248)),
                            );
                            egui::ScrollArea::vertical()
                                .max_height(scroll_h)
                                .stick_to_bottom(true)
                                .show(&mut cols[0], |ui| {
                                    for frame in &self.latest.recent_frames {
                                        let status_color = if frame.crc_ok {
                                            Color32::from_rgb(74, 222, 128)
                                        } else {
                                            Color32::from_rgb(248, 113, 113)
                                        };
                                        ui.label(
                                            RichText::new(format!(
                                                "[{:6.2}s] TX {} -> RX {} ({:.1}ms) {} | {}",
                                                frame.timestamp_sec,
                                                frame.tx_hex,
                                                frame.rx_hex,
                                                frame.rtt_ms,
                                                if frame.crc_ok {
                                                    "CRC16-OK"
                                                } else {
                                                    "CRC16-ERR!"
                                                },
                                                frame.description
                                            ))
                                            .font(FontId::new(10.5, FontFamily::Monospace))
                                            .color(status_color),
                                        );
                                    }
                                });

                            // Right side: Technician Fault Injection & 200ms Watchdog Verification
                            cols[1].vertical(|ui| {
                                ui.label(
                                    RichText::new(format!(
                                        "ACTIVE SCHEDULE: {:.0} A Target | {:.0} mm/min Travel",
                                        self.latest.target_current_amps,
                                        self.latest.target_travel_speed_mm_min
                                    ))
                                    .font(FontId::new(11.0, FontFamily::Monospace))
                                    .color(Color32::from_rgb(52, 211, 153)),
                                );
                                ui.label(
                                    RichText::new("HARDWARE-IN-THE-LOOP SAFETY VERIFICATION")
                                        .font(FontId::new(11.5, FontFamily::Monospace))
                                        .strong()
                                        .color(Color32::from_rgb(226, 232, 240)),
                                );
                                ui.add_space(4.0);
                                ui.label(
                                    RichText::new(
                                        "• 200ms Hardware Watchdog: If GUI thread stops sending keep-alive heartbeats during jogging, the 50Hz worker immediately commands a servo halt.",
                                    )
                                    .font(FontId::new(11.0, FontFamily::Proportional))
                                    .color(Color32::from_rgb(148, 163, 184)),
                                );
                                ui.add_space(6.0);
                                if ui
                                    .add(
                                        egui::Button::new(
                                            RichText::new(
                                                "⏱ TEST 200ms WATCHDOG TRIP (START JOG & DROP HEARTBEAT)",
                                            )
                                            .font(FontId::new(11.5, FontFamily::Monospace))
                                            .strong()
                                            .color(Color32::WHITE),
                                        )
                                        .fill(Color32::from_rgb(153, 27, 27))
                                        .min_size(Vec2::new(ui.available_width(), 34.0)),
                                    )
                                    .clicked()
                                {
                                    self.suppress_watchdog_heartbeat = true;
                                    let _ = self.cmd_tx.send(HmiCommand::JogRight {
                                        speed_mm_min: 900.0,
                                    });
                                }
                            });
                        });
                    }
                }
            });
    }
}

impl eframe::App for SeamWelderHmiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.ingest_telemetry();

        egui::CentralPanel::default().show(ctx, |ui| {
            self.render_top_safety_header(ui);
            ui.add_space(6.0);
            self.render_carriage_schematic(ui);
            ui.add_space(6.0);
            self.render_middle_operator_deck(ui);
            ui.add_space(6.0);
            self.render_bottom_telemetry_and_diagnostics(ui);
        });

        // Continuous 60 FPS repaint for smooth real-time oscilloscope & carriage motion
        ctx.request_repaint_after(std::time::Duration::from_millis(16));
    }
}
