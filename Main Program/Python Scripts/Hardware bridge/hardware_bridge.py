import logging
import os
import time

try:
    import serial
except Exception:
    serial = None

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None



DEFAULT_COM_PORT = "COM14"
DEFAULT_BAUD = 250000
SERIAL_TIMEOUT_S = 0.001
DEFAULT_PEDALS_COM_PORT = "COM6"
DEFAULT_PEDALS_BAUD = 115200
DEFAULT_BUTTONS_COM_PORT = "COM8"
DEFAULT_BUTTONS_BAUD = 115200




PEDAL_INPUT_MAX_TICKS = 1023.0


PEDAL_THROTTLE_RANGE_TICKS = 530.0

PEDAL_BRAKE_RANGE_TICKS = 394.0

PEDAL_DEADZONE_TICKS = 10.0

LEFT_BLINKER_BUTTON = 1
RIGHT_BLINKER_BUTTON = 4
AUTONOMY_ENGAGE_BUTTON = 0
DRIVE_REVERSE_BUTTON = 3
NAV_MAP_TOGGLE_BUTTON = 2
RESPAWN_BUTTON = 5


MAX_TICKS = 400.0
STEER_SENSITIVITY = 0.12


CENTER_KP = 0.90
CENTER_KD = 0.06
CENTER_MAX_PWM = 255
CENTER_DEADBAND_TICKS = 8
FFB_TARGET_GAIN = 1.0



MANUAL_SPRING_CURVE_EXPONENT = 1.35
MANUAL_SPRING_MAX_PWM = 255
MANUAL_RETURN_DAMPING_SCALE = 0.22

MANUAL_COUNTER_RESIST_KP_SCALE = 1.30
MANUAL_COUNTER_RESIST_KD_SCALE = 1.70
MANUAL_COUNTER_RESIST_BOOST_PWM = 22
MANUAL_IDLE_SPEED_MPS = 0.25
MANUAL_IDLE_STEER_DEADBAND = 0.01
MANUAL_IDLE_VEL_TICKS_PER_S = 25.0


MANUAL_CENTER_STICTION_PWM = 70
MANUAL_CENTER_STICTION_ERROR_TICKS = 18
MANUAL_CENTER_STICTION_MAX_VEL_TICKS_PER_S = 110.0


AUTO_STEER_LIMIT = STEER_SENSITIVITY
AUTO_TARGET_RATE_LIMIT_TICKS = 16.0
AUTO_CENTER_KP = 0.75
AUTO_CENTER_KD = 0.05
AUTO_CENTER_MAX_PWM = 255
AUTO_CENTER_ENTER_DEADBAND_TICKS = 14
AUTO_CENTER_EXIT_DEADBAND_TICKS = 8


AUTO_CENTER_STICTION_PWM = 58
AUTO_CENTER_STICTION_ERROR_TICKS = 14
AUTO_CENTER_STICTION_MAX_VEL_TICKS_PER_S = 80.0


TAKEOVER_ERROR_TICKS = 30.0
TAKEOVER_HOLD_SECONDS = 0.10
TAKEOVER_MIN_VEL_TICKS_PER_S = 140.0
TAKEOVER_MIN_PWM = 20

TAKEOVER_MAX_TARGET_RATE_TICKS_PER_S = 220.0
TAKEOVER_MIN_ERROR_GROWTH_TICKS_PER_S = 120.0

AUTO_TURN_TRIGGER_M = 25.0
AUTO_LANE_CHANGE_TRIGGER_M = 10.0
AUTO_BLINKER_OFF_HYSTERESIS_S = 1.0


class HardwareBridge:
    """Non-blocking serial bridge for wheel encoder, blinker buttons, and FFB output."""

    @staticmethod
    def _auto_detect_port(match_terms, exclude_port=None):
        if list_ports is None:
            return None
        exclude = str(exclude_port).strip().lower() if exclude_port else ""
        for port in list_ports.comports():
            device = str(getattr(port, "device", "")).strip()
            if not device:
                continue
            if exclude and device.lower() == exclude:
                continue
            haystack = " ".join(
                [
                    str(getattr(port, "device", "")),
                    str(getattr(port, "description", "")),
                    str(getattr(port, "manufacturer", "")),
                    str(getattr(port, "hwid", "")),
                ]
            ).lower()
            if any(term in haystack for term in match_terms):
                return device
        return None

    @staticmethod
    def _port_exists(port_name):
        if list_ports is None:
            return False
        target = str(port_name).strip().lower()
        if not target:
            return False
        for port in list_ports.comports():
            device = str(getattr(port, "device", "")).strip().lower()
            if device == target:
                return True
        return False

    def __init__(self, com_port=None, baud=None):
        env_steering = os.environ.get("STEERING_COM_PORT", "").strip()

        self.com_port = com_port or env_steering or DEFAULT_COM_PORT
        self.baud = int(baud or os.environ.get("STEERING_BAUD", str(DEFAULT_BAUD)))

        self.enabled = False
        self._ser = None
        self._last_pwm = 0

        self.encoder_zero = None
        self.encoder_counts = None
        self.vel_ticks = 0.0
        self._last_ticks = None
        self._last_time_s = None
        self._last_raw_encoder = None

        self.left_blinker_on = False
        self.right_blinker_on = False
        self._seen_blink_packets = False
        self._takeover_start_s = None
        self._takeover_last_eval_s = None
        self._takeover_last_target_ticks = None
        self._takeover_last_error_ticks = None
        self._auto_target_ticks_smoothed = None

        self._poll_seq = 0
        self._auto_target_cache_poll_seq = None
        self._auto_target_cache_ticks = None
        self._auto_center_correction_active = False
        self._toggle_reverse_requested = False
        self._toggle_nav_map_requested = False
        self._engage_autonomy_requested = False
        self._respawn_requested = False
        self._auto_blinker_cmd = "0"
        self._auto_blinker_off_since_s = None

        if serial is None:
            logging.warning("pyserial not available. Hardware bridge disabled.")
            return

        if not self.com_port:
            logging.info(
                "No steering hardware serial port detected. "
                "Set STEERING_COM_PORT to force a specific port."
            )
            return

        try:
            self._ser = serial.Serial(self.com_port, self.baud, timeout=SERIAL_TIMEOUT_S)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.enabled = True
            logging.info(
                "Hardware bridge connected on %s @ %d baud.",
                self.com_port,
                self.baud,
            )
        except Exception as exc:
            self._ser = None
            self.enabled = False
            logging.warning("Hardware bridge disabled (%s).", exc)

    def _decode_serial_line(self, line):
        text = str(line).strip()
        if not text:
            return ("none", None)

        if text.startswith("BL:"):
            payload = text[3:].strip()
            try:
                left_text, right_text = payload.split(",", 1)
                left_on = left_text.strip() == "1"
                right_on = right_text.strip() == "1"
                return ("blink", (left_on, right_on))
            except Exception:
                return ("none", None)

        encoder_text = text
        if "ENC:" in text:
            encoder_text = text.split("ENC:", 1)[1].strip()

        try:
            return ("encoder", float(encoder_text))
        except ValueError:
            pass

        if text.startswith("A") and text[1:].isdigit():
            return ("button", int(text[1:]))
        return ("none", None)

    def _set_vehicle_blinkers(self, vehicle):
        if vehicle is None:
            return
        try:
            import carla

            flags = carla.VehicleLightState
            bits = int(vehicle.get_light_state())
            bits &= ~(int(flags.LeftBlinker) | int(flags.RightBlinker))
            if self.left_blinker_on:
                bits |= int(flags.LeftBlinker)
            if self.right_blinker_on:
                bits |= int(flags.RightBlinker)
            vehicle.set_light_state(carla.VehicleLightState(bits))
        except Exception:
            pass

    @staticmethod
    def _road_option_name(option):
        if option is None:
            return ""
        name = getattr(option, "name", None)
        if name is not None:
            return str(name).upper()
        text = str(option).upper()
        if "." in text:
            text = text.rsplit(".", 1)[-1]
        return text

    @staticmethod
    def _find_closest_route_index(vehicle_location, route):
        closest_idx = 0
        min_dist = float("inf")
        for i, (waypoint, _) in enumerate(route):
            try:
                dist = vehicle_location.distance(waypoint.transform.location)
            except Exception:
                continue
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx

    def _compute_route_blinker_intent(self, vehicle, route, route_index=None):
        if vehicle is None or not route:
            return "0"
        if route_index is None:
            try:
                vehicle_location = vehicle.get_location()
            except Exception:
                return "0"
            closest_idx = self._find_closest_route_index(vehicle_location, route)
        else:
            try:
                closest_idx = int(route_index)
            except Exception:
                closest_idx = 0
            if closest_idx < 0:
                closest_idx = 0
            if closest_idx >= len(route):
                closest_idx = len(route) - 1

        start_idx = max(0, closest_idx - 3)
        distance_ahead = 0.0
        for i in range(start_idx, len(route)):
            if i > start_idx:
                try:
                    prev_loc = route[i - 1][0].transform.location
                    curr_loc = route[i][0].transform.location
                    distance_ahead += prev_loc.distance(curr_loc)
                except Exception:
                    pass

            option_name = self._road_option_name(route[i][1])
            if option_name == "CHANGELANELEFT" and distance_ahead <= AUTO_LANE_CHANGE_TRIGGER_M:
                return "L"
            if option_name == "CHANGELANERIGHT" and distance_ahead <= AUTO_LANE_CHANGE_TRIGGER_M:
                return "R"
            if option_name == "LEFT" and distance_ahead <= AUTO_TURN_TRIGGER_M:
                return "L"
            if option_name == "RIGHT" and distance_ahead <= AUTO_TURN_TRIGGER_M:
                return "R"

            if distance_ahead > AUTO_TURN_TRIGGER_M:
                break
        return "0"

    def _write_blinker_command(self, cmd):
        if not self.enabled or self._ser is None:
            return
        try:
            self._ser.write(f"BLCMD:{cmd}\n".encode())
        except Exception:
            pass

    def _set_auto_blinker_command(self, cmd):
        cmd = cmd if cmd in ("L", "R", "0") else "0"
        if cmd == self._auto_blinker_cmd:
            return
        self._auto_blinker_cmd = cmd
        self._write_blinker_command(cmd)

    def update_auto_blinker(self, vehicle, route, autonomy_active, route_index=None):
        if not autonomy_active:
            self._auto_blinker_off_since_s = None
            self._set_auto_blinker_command("0")
            return

        intent = self._compute_route_blinker_intent(vehicle, route, route_index=route_index)
        if intent in ("L", "R"):
            self._auto_blinker_off_since_s = None
            self._set_auto_blinker_command(intent)
            return

        if self._auto_blinker_cmd in ("L", "R"):
            now_s = time.monotonic()
            if self._auto_blinker_off_since_s is None:
                self._auto_blinker_off_since_s = now_s
                return
            if (now_s - self._auto_blinker_off_since_s) < AUTO_BLINKER_OFF_HYSTERESIS_S:
                return

        self._auto_blinker_off_since_s = None
        self._set_auto_blinker_command("0")

    def poll(self, vehicle):
        """Read wheel + button serial input and update internal state."""
        if not self.enabled or self._ser is None:
            return
        self._poll_seq += 1

        raw = None
        while self._ser.in_waiting:
            try:
                line = self._ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue

            line_type, value = self._decode_serial_line(line)
            if line_type == "encoder":
                raw = value
            elif line_type == "blink":
                self._seen_blink_packets = True
                self.left_blinker_on, self.right_blinker_on = bool(value[0]), bool(value[1])
            elif line_type == "button":
                if value == LEFT_BLINKER_BUTTON:

                    if not self._seen_blink_packets:
                        if self.left_blinker_on:
                            self.left_blinker_on = False
                            self.right_blinker_on = False
                        else:
                            self.left_blinker_on = True
                            self.right_blinker_on = False
                elif value == RIGHT_BLINKER_BUTTON:

                    if not self._seen_blink_packets:
                        if self.right_blinker_on:
                            self.right_blinker_on = False
                            self.left_blinker_on = False
                        else:
                            self.right_blinker_on = True
                            self.left_blinker_on = False
                elif value == DRIVE_REVERSE_BUTTON:
                    self._toggle_reverse_requested = True
                elif value == NAV_MAP_TOGGLE_BUTTON:
                    self._toggle_nav_map_requested = True
                elif value == AUTONOMY_ENGAGE_BUTTON:
                    self._engage_autonomy_requested = True
                elif value == RESPAWN_BUTTON:
                    self._respawn_requested = True


        self._set_vehicle_blinkers(vehicle)

        if raw is None:
            return

        self._last_raw_encoder = float(raw)
        if self.encoder_zero is None:
            self.encoder_zero = raw
            self.encoder_counts = 0.0
            self.vel_ticks = 0.0
            self._last_ticks = self.encoder_counts
            self._last_time_s = time.monotonic()
            logging.info("Wheel encoder zeroed at raw=%.3f", self.encoder_zero)
            return

        self.encoder_counts = -(raw - self.encoder_zero)
        now_s = time.monotonic()
        if self._last_ticks is not None and self._last_time_s is not None:
            dt = now_s - self._last_time_s
            if dt > 0.0:
                inst_vel = (self.encoder_counts - self._last_ticks) / dt
                self.vel_ticks = 0.7 * self.vel_ticks + 0.3 * inst_vel
        self._last_ticks = self.encoder_counts
        self._last_time_s = now_s

    def get_manual_steer(self):
        """Return normalized steering from wheel encoder for manual drive mode."""
        if self.encoder_counts is None:
            return None
        steer = (self.encoder_counts / MAX_TICKS) * STEER_SENSITIVITY
        steer = max(-1.0, min(1.0, steer))
        if abs(steer) < 0.02:
            steer = 0.0
        return float(steer)

    def rezero_encoder(self):
        """Set current wheel position as the new encoder center."""
        if self.encoder_zero is None:
            return False

        if self._last_raw_encoder is not None:
            self.encoder_zero = float(self._last_raw_encoder)
        elif self.encoder_counts is not None:

            self.encoder_zero = float(self.encoder_zero) - float(self.encoder_counts)
        else:
            return False

        self.encoder_counts = 0.0
        self.vel_ticks = 0.0
        self._last_ticks = 0.0
        self._last_time_s = time.monotonic()
        self._takeover_start_s = None
        self._takeover_last_eval_s = None
        self._takeover_last_target_ticks = None
        self._takeover_last_error_ticks = None
        self._auto_target_ticks_smoothed = None
        self._auto_target_cache_ticks = None
        self._auto_target_cache_poll_seq = None
        self._auto_center_correction_active = False
        self._write_pwm(0)
        return True

    def _compute_target_ticks(self, vehicle, auto_mode=False):
        if vehicle is None:
            return 0.0
        if auto_mode:
            if (
                self._auto_target_cache_poll_seq == self._poll_seq
                and self._auto_target_cache_ticks is not None
            ):
                return self._auto_target_cache_ticks

            try:
                target_steer = float(vehicle.get_control().steer)
            except Exception:
                target_steer = 0.0
            target_steer = max(-AUTO_STEER_LIMIT, min(AUTO_STEER_LIMIT, target_steer))
            target_ticks = (target_steer / AUTO_STEER_LIMIT) * MAX_TICKS if AUTO_STEER_LIMIT > 0 else 0.0

            if self._auto_target_ticks_smoothed is None:
                self._auto_target_ticks_smoothed = target_ticks
            else:
                delta = target_ticks - self._auto_target_ticks_smoothed
                if abs(delta) > AUTO_TARGET_RATE_LIMIT_TICKS:
                    self._auto_target_ticks_smoothed += AUTO_TARGET_RATE_LIMIT_TICKS * self._sign(delta)
                else:
                    self._auto_target_ticks_smoothed = target_ticks
            self._auto_target_cache_ticks = self._auto_target_ticks_smoothed
            self._auto_target_cache_poll_seq = self._poll_seq
            return self._auto_target_cache_ticks

        self._auto_target_ticks_smoothed = None
        self._auto_target_cache_ticks = None
        self._auto_target_cache_poll_seq = None
        try:


            target_steer = float(vehicle.get_control().steer)
        except Exception:
            target_steer = 0.0

        return target_steer * MAX_TICKS * FFB_TARGET_GAIN

    def _compute_center_pwm(self, vehicle, auto_mode=False):
        if self.encoder_counts is None or vehicle is None:
            self._auto_center_correction_active = False
            return 0

        target_ticks = self._compute_target_ticks(vehicle, auto_mode=auto_mode)
        error_ticks = self.encoder_counts - target_ticks
        abs_error_ticks = abs(error_ticks)
        if auto_mode:
            kp = AUTO_CENTER_KP
            kd = AUTO_CENTER_KD
            pwm_limit = AUTO_CENTER_MAX_PWM
            if not self._auto_center_correction_active:
                if abs_error_ticks < AUTO_CENTER_ENTER_DEADBAND_TICKS:
                    return 0
                self._auto_center_correction_active = True
            elif abs_error_ticks <= AUTO_CENTER_EXIT_DEADBAND_TICKS:
                self._auto_center_correction_active = False
                return 0
        else:
            self._auto_center_correction_active = False
            deadband = CENTER_DEADBAND_TICKS
            kp = CENTER_KP
            kd = CENTER_KD
            pwm_limit = CENTER_MAX_PWM


            try:
                velocity = vehicle.get_velocity()
                speed_mps = float((velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z) ** 0.5)
            except Exception:
                speed_mps = 0.0
            try:
                command_steer = float(vehicle.get_control().steer)
            except Exception:
                command_steer = 0.0
            if (
                speed_mps <= MANUAL_IDLE_SPEED_MPS
                and abs(command_steer) <= MANUAL_IDLE_STEER_DEADBAND
                and abs(self.vel_ticks) <= MANUAL_IDLE_VEL_TICKS_PER_S
                and abs_error_ticks <= (deadband * 1.5)
            ):
                return 0
            if abs_error_ticks <= deadband:
                return 0

        moving_toward_center = (
            not auto_mode
            and self._sign(error_ticks) != 0
            and self._sign(self.vel_ticks) == -self._sign(error_ticks)
        )
        moving_away_from_center = (
            not auto_mode
            and self._sign(error_ticks) != 0
            and self._sign(self.vel_ticks) == self._sign(error_ticks)
        )

        kp_effective = kp
        kd_effective = kd
        if moving_toward_center:
            kd_effective = kd * MANUAL_RETURN_DAMPING_SCALE
        elif moving_away_from_center:
            kp_effective = kp * MANUAL_COUNTER_RESIST_KP_SCALE
            kd_effective = kd * MANUAL_COUNTER_RESIST_KD_SCALE

        pwm = int(-kp_effective * error_ticks - kd_effective * self.vel_ticks)
        if not auto_mode:
            spring_sign = -self._sign(error_ticks)
            spring_pwm = self._compute_manual_spring_pwm(abs_error_ticks)
            if spring_sign != 0 and spring_pwm > 0:
                pwm = spring_sign * max(abs(pwm), spring_pwm)
            if moving_away_from_center and MANUAL_COUNTER_RESIST_BOOST_PWM > 0 and pwm != 0:
                pwm = self._sign(pwm) * (
                    abs(pwm) + int(MANUAL_COUNTER_RESIST_BOOST_PWM)
                )
        if (
            not auto_mode
            and abs_error_ticks >= MANUAL_CENTER_STICTION_ERROR_TICKS
            and abs(self.vel_ticks) <= MANUAL_CENTER_STICTION_MAX_VEL_TICKS_PER_S
        ):
            pwm = self._with_stiction_floor(pwm, MANUAL_CENTER_STICTION_PWM)
        return max(-pwm_limit, min(pwm_limit, pwm))

    @staticmethod
    def _sign(value):
        if value > 0:
            return 1
        if value < 0:
            return -1
        return 0

    @staticmethod
    def _with_stiction_floor(pwm, stiction_pwm):
        if pwm == 0 or stiction_pwm <= 0:
            return int(pwm)
        if abs(pwm) >= stiction_pwm:
            return int(pwm)
        return int(stiction_pwm if pwm > 0 else -stiction_pwm)

    @staticmethod
    def _compute_manual_spring_pwm(abs_error_ticks):
        if abs_error_ticks <= CENTER_DEADBAND_TICKS:
            return 0
        if MAX_TICKS <= 0:
            return 0
        normalized = max(0.0, min(1.0, float(abs_error_ticks) / float(MAX_TICKS)))
        curved = normalized ** MANUAL_SPRING_CURVE_EXPONENT
        return int(round(curved * MANUAL_SPRING_MAX_PWM))

    def reset_takeover_detection(self):
        """Clear takeover detection state, e.g. on autonomy mode transitions."""
        self._takeover_start_s = None
        self._takeover_last_eval_s = None
        self._takeover_last_target_ticks = None
        self._takeover_last_error_ticks = None

    def detect_manual_takeover(self, vehicle):
        """Detect strong human steering override against autonomy torque."""
        if self.encoder_counts is None or vehicle is None:
            self.reset_takeover_detection()
            return False

        now_s = time.monotonic()
        target_ticks = self._compute_target_ticks(vehicle, auto_mode=True)
        error_ticks = target_ticks - self.encoder_counts
        pwm = self._compute_center_pwm(vehicle, auto_mode=True)
        target_rate_ticks_s = 0.0
        error_rate_ticks_s = 0.0

        if self._takeover_last_eval_s is not None:
            dt_s = now_s - self._takeover_last_eval_s
            if dt_s > 1e-4:
                if self._takeover_last_target_ticks is not None:
                    target_rate_ticks_s = (target_ticks - self._takeover_last_target_ticks) / dt_s
                if self._takeover_last_error_ticks is not None:
                    error_rate_ticks_s = (error_ticks - self._takeover_last_error_ticks) / dt_s

        self._takeover_last_eval_s = now_s
        self._takeover_last_target_ticks = target_ticks
        self._takeover_last_error_ticks = error_ticks

        target_is_stable = abs(target_rate_ticks_s) <= TAKEOVER_MAX_TARGET_RATE_TICKS_PER_S
        error_growing_away = (
            self._sign(error_ticks) != 0
            and self._sign(error_rate_ticks_s) == self._sign(error_ticks)
            and abs(error_rate_ticks_s) >= TAKEOVER_MIN_ERROR_GROWTH_TICKS_PER_S
        )

        diverging_from_controller = (
            abs(pwm) >= TAKEOVER_MIN_PWM
            and abs(self.vel_ticks) >= TAKEOVER_MIN_VEL_TICKS_PER_S
            and self._sign(self.vel_ticks) == -self._sign(pwm)
        )

        if (
            target_is_stable
            and abs(error_ticks) >= TAKEOVER_ERROR_TICKS
            and diverging_from_controller
            and error_growing_away
        ):
            if self._takeover_start_s is None:
                self._takeover_start_s = now_s
            elif (now_s - self._takeover_start_s) >= TAKEOVER_HOLD_SECONDS:
                self._takeover_start_s = None
                return True
        else:
            self._takeover_start_s = None

        return False

    def _write_pwm(self, pwm):
        if not self.enabled or self._ser is None:
            return
        pwm_limit = max(CENTER_MAX_PWM, AUTO_CENTER_MAX_PWM)
        pwm = int(max(-pwm_limit, min(pwm_limit, pwm)))
        if pwm == self._last_pwm:
            return
        try:
            self._ser.write(f"{pwm}\n".encode())
            self._last_pwm = pwm
        except Exception:
            pass

    def apply_force_feedback(self, vehicle, auto_mode=False):
        pwm = self._compute_center_pwm(vehicle, auto_mode=auto_mode)
        if auto_mode and pwm != 0 and self.encoder_counts is not None and vehicle is not None:
            target_ticks = self._compute_target_ticks(vehicle, auto_mode=True)
            error_ticks = target_ticks - self.encoder_counts
            if (
                abs(error_ticks) >= AUTO_CENTER_STICTION_ERROR_TICKS
                and abs(self.vel_ticks) <= AUTO_CENTER_STICTION_MAX_VEL_TICKS_PER_S
            ):
                pwm = self._with_stiction_floor(pwm, AUTO_CENTER_STICTION_PWM)
        if auto_mode:
            pwm = max(-AUTO_CENTER_MAX_PWM, min(AUTO_CENTER_MAX_PWM, pwm))
        self._write_pwm(pwm)

    def consume_toggle_reverse_request(self):
        requested = self._toggle_reverse_requested
        self._toggle_reverse_requested = False
        return requested

    def consume_toggle_nav_map_request(self):
        requested = self._toggle_nav_map_requested
        self._toggle_nav_map_requested = False
        return requested

    def consume_engage_autonomy_request(self):
        requested = self._engage_autonomy_requested
        self._engage_autonomy_requested = False
        return requested

    def consume_respawn_request(self):
        requested = self._respawn_requested
        self._respawn_requested = False
        return requested

    def close(self):
        if self._ser is not None:
            try:
                self._ser.write(b"BLCMD:0\n")
            except Exception:
                pass
            try:
                self._ser.write(b"0\n")
            except Exception:
                pass
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.enabled = False


class PedalsBridge:
    """Non-blocking serial bridge for pedal potentiometer telemetry."""

    def __init__(self, com_port=None, baud=None, exclude_port=None):
        env_port = os.environ.get("PEDALS_COM_PORT", "").strip()
        env_baud = os.environ.get("PEDALS_BAUD", "").strip()
        _ = exclude_port  

        self.com_port = com_port or env_port or DEFAULT_PEDALS_COM_PORT
        self.baud = int(baud or env_baud or DEFAULT_PEDALS_BAUD)

        self.enabled = False
        self._ser = None
        self.throttle = None  
        self.brake = None  
        self.last_update_s = None
        self._throttle_zero_raw = None
        self._brake_zero_raw = None
        self.throttle_range_ticks = float(
            os.environ.get("PEDAL_THROTTLE_RANGE_TICKS", str(PEDAL_THROTTLE_RANGE_TICKS))
        )
        self.brake_range_ticks = float(
            os.environ.get("PEDAL_BRAKE_RANGE_TICKS", str(PEDAL_BRAKE_RANGE_TICKS))
        )

        if serial is None:
            logging.warning("pyserial not available. Pedals bridge disabled.")
            return

        try:
            self._ser = serial.Serial(self.com_port, self.baud, timeout=SERIAL_TIMEOUT_S)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.enabled = True
            logging.info(
                "Pedals bridge connected on %s @ %d baud.",
                self.com_port,
                self.baud,
            )
        except Exception as exc:
            self._ser = None
            self.enabled = False
            logging.warning("Pedals bridge disabled (%s).", exc)

    @staticmethod
    def _parse_pedal_line(line):
        text = str(line).strip()
        if not text.startswith("T:") or ",B:" not in text:
            return None
        try:
            throttle_text, brake_text = text.split(",B:", 1)
            throttle = float(throttle_text[2:].strip())
            brake = float(brake_text.strip())
        except Exception:
            return None


        if max(abs(throttle), abs(brake)) <= 1.5:
            throttle *= PEDAL_INPUT_MAX_TICKS
            brake *= PEDAL_INPUT_MAX_TICKS
        throttle = max(0.0, throttle)
        brake = max(0.0, brake)
        return throttle, brake

    @staticmethod
    def _to_delta_ticks(raw, zero):
        if raw is None or zero is None:
            return 0.0
        delta = abs(float(raw) - float(zero))
        return max(0.0, delta - PEDAL_DEADZONE_TICKS)

    def poll(self):
        if not self.enabled or self._ser is None:
            return

        while self._ser.in_waiting:
            try:
                line = self._ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue

            parsed = self._parse_pedal_line(line)
            if parsed is None:
                continue
            raw_throttle, raw_brake = parsed
            if self._throttle_zero_raw is None:
                self._throttle_zero_raw = float(raw_throttle)
                logging.info("Throttle zeroed at raw ticks=%.1f", self._throttle_zero_raw)
            if self._brake_zero_raw is None:
                self._brake_zero_raw = float(raw_brake)
                logging.info("Brake zeroed at raw ticks=%.1f", self._brake_zero_raw)

            throttle_ticks = self._to_delta_ticks(raw_throttle, self._throttle_zero_raw)
            brake_ticks = self._to_delta_ticks(raw_brake, self._brake_zero_raw)

            self.throttle = max(0.0, float(throttle_ticks))
            self.brake = max(0.0, float(brake_ticks))
            self.last_update_s = time.monotonic()

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.enabled = False


class ButtonsBridge:
    """Non-blocking serial bridge for action buttons on a dedicated Uno."""

    def __init__(self, com_port=None, baud=None, exclude_port=None):
        env_port = os.environ.get("BUTTONS_COM_PORT", "").strip()
        env_baud = os.environ.get("BUTTONS_BAUD", "").strip()
        _ = exclude_port  

        self.com_port = com_port or env_port or DEFAULT_BUTTONS_COM_PORT
        self.baud = int(baud or env_baud or DEFAULT_BUTTONS_BAUD)

        self.enabled = False
        self._ser = None
        self._toggle_reverse_requested = False
        self._toggle_nav_map_requested = False
        self._engage_autonomy_requested = False
        self._respawn_requested = False
        self.joystick_vrx = None
        self.joystick_vry = None
        self.joystick_sw_pressed = None

        if serial is None:
            logging.warning("pyserial not available. Buttons bridge disabled.")
            return

        try:
            self._ser = serial.Serial(self.com_port, self.baud, timeout=SERIAL_TIMEOUT_S)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.enabled = True
            logging.info(
                "Buttons bridge connected on %s @ %d baud.",
                self.com_port,
                self.baud,
            )
        except Exception as exc:
            self._ser = None
            self.enabled = False
            logging.warning("Buttons bridge disabled (%s).", exc)

    @staticmethod
    def _parse_button_line(line):
        text = str(line).strip().upper()
        if not text:
            return None
        if text.startswith("A") and text[1:].isdigit():
            return int(text[1:])
        if text.startswith("BTN:"):
            payload = text[4:].strip()
            if payload.isdigit():
                return int(payload)
        return None

    @staticmethod
    def _parse_joystick_line(line):
        text = str(line).strip().upper()
        if not text.startswith("JOY:"):
            return None
        payload = text[4:].strip()
        parts = [part.strip() for part in payload.split(",")]
        if len(parts) != 3:
            return None
        try:
            vrx = int(parts[0])
            vry = int(parts[1])
            sw = int(parts[2])
        except Exception:
            return None
        return vrx, vry, bool(sw)

    def _handle_button_event(self, value):
        if value == DRIVE_REVERSE_BUTTON:
            self._toggle_reverse_requested = True
        elif value == NAV_MAP_TOGGLE_BUTTON:
            self._toggle_nav_map_requested = True
        elif value == AUTONOMY_ENGAGE_BUTTON:
            self._engage_autonomy_requested = True
        elif value == RESPAWN_BUTTON:
            self._respawn_requested = True

    def poll(self):
        if not self.enabled or self._ser is None:
            return

        while self._ser.in_waiting:
            try:
                line = self._ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            value = self._parse_button_line(line)
            if value is not None:
                self._handle_button_event(value)
                continue
            joystick_state = self._parse_joystick_line(line)
            if joystick_state is None:
                continue
            self.joystick_vrx, self.joystick_vry, self.joystick_sw_pressed = joystick_state

    def consume_toggle_reverse_request(self):
        requested = self._toggle_reverse_requested
        self._toggle_reverse_requested = False
        return requested

    def consume_toggle_nav_map_request(self):
        requested = self._toggle_nav_map_requested
        self._toggle_nav_map_requested = False
        return requested

    def consume_engage_autonomy_request(self):
        requested = self._engage_autonomy_requested
        self._engage_autonomy_requested = False
        return requested

    def consume_respawn_request(self):
        requested = self._respawn_requested
        self._respawn_requested = False
        return requested

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.enabled = False
