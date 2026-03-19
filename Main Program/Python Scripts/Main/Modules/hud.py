
import pygame

from Modules.constants import (
    COLOR_BLACK,
    COLOR_ALUMINIUM_5,
    COLOR_ALUMINIUM_4,
    COLOR_ALUMINIUM_3,
    COLOR_ALUMINIUM_2,
    COLOR_WHITE,
    COLOR_ORANGE_0,
    COLOR_SCARLET_RED_0,
)


class HUD:
    """
    HUD container with a simple Pure Pursuit tuning panel.
    Allows live editing of controller parameters via click + type.
    """

    def __init__(self, name, width, height):
        self.name = name
        self.dim = (width, height)

        self.show_info = False
        self.show_pp_tuning = False
        self._info_text = {}

        self._header_font = pygame.font.SysFont("Arial", 14, True)
        self._body_font = pygame.font.SysFont("Arial", 13, False)
        self._mono_font = pygame.font.SysFont("Consolas", 13, False)

        self._autonomy_controller = None

        self._pp_fields = [
            ("target_speed_kmh", "Target speed (km/h)", "float"),
            ("lookahead_distance", "Lookahead base (m)", "float"),
            ("adaptive_lookahead", "Adaptive lookahead", "bool"),
            ("lookahead_speed_gain", "Lookahead speed gain", "float"),
            ("lookahead_min", "Lookahead min (m)", "float"),
            ("lookahead_max", "Lookahead max (m)", "float"),
            ("steer_gain", "Steer gain", "float"),
            ("throttle_low", "Throttle low", "float"),
            ("throttle_high", "Throttle high", "float"),
            ("wheelbase_override", "Wheelbase override (m)", "float_or_auto"),
        ]
        self._pp_values = {
            "target_speed_kmh": 30.0,
            "lookahead_distance": 4.0,
            "adaptive_lookahead": True,
            "lookahead_speed_gain": 0.25,
            "lookahead_min": 2.0,
            "lookahead_max": 12.0,
            "steer_gain": 1.0,
            "throttle_low": 0.2,
            "throttle_high": 0.5,
            "wheelbase_override": None,
        }

        self._editing_key = None
        self._edit_buffer = ""
        self._pp_value_rects = {}
        self._pp_status = ""
        self._pp_status_until = 0
        self._drive_status = {
            "mode": "MANUAL",
            "drivetrain": "DRIVE",
            "ticks": None,
            "left_blinker": False,
            "right_blinker": False,
            "pedal_throttle": None,
            "pedal_brake": None,
            "pedal_throttle_max": None,
            "pedal_brake_max": None,
            "nav_mode_active": False,
            "joystick_vrx": None,
            "joystick_vry": None,
            "joystick_sw_pressed": None,
        }
        self._map_mode_hint_lines = [
            "Joystick: Move map crosshair",
            "Joystick SW: Set destination",
        ]
        self._keybind_lines = [
            "F1: Toggle keybind list",
            "F2: Toggle PP tuning panel",
            "F8: Recenter steering wheel",
            "TAB: Toggle navigation map",
            "V: Toggle camera mode (first/third person view)",
            "B: Toggle YOLO view",
            "L: Toggle 3D route line",
            "C: Toggle custom autonomy",
            "P: Toggle CARLA autopilot",
            "Q: Toggle drive/reverse",
            "W/UP: Throttle",
            "S/DOWN: Brake",
            "A,D or LEFT,RIGHT: Steering",
            "ESC or CTRL+Q: Quit",
        ]
        self._keybind_panel_cache = None
        self._keybind_panel_cache_rect = None
        self._keybind_panel_cache_key = None
        self._map_mode_panel_cache = None
        self._map_mode_panel_cache_rect = None
        self._map_mode_panel_cache_key = None
        self._pp_layout_cache = None
        self._pp_layout_cache_key = None
        self._pp_static_panel_cache = None
        self._pp_static_panel_cache_key = None
        self._pp_values_layer_cache = None
        self._pp_values_layer_signature = None
        self._pp_drive_layer_cache = None
        self._pp_drive_line_cache = {}
        self._pp_drive_start_y = 0
        self._pp_drive_row_h = 18
        self._pp_status_surface_cache = None
        self._pp_status_surface_signature = None

    def start(self):
        pass

    def tick(self, clock):
        pass

    def set_autonomy_controller(self, controller):
        self._autonomy_controller = controller
        if controller is not None:
            self._sync_from_controller()

    def get_pp_params(self):
        return dict(self._pp_values)

    def set_pp_tuning_visible(self, visible):
        self.show_pp_tuning = bool(visible)
        if not self.show_pp_tuning:
            self._editing_key = None

    def set_drive_status(
        self,
        mode=None,
        drivetrain=None,
        ticks=None,
        left_blinker=None,
        right_blinker=None,
        pedal_throttle=None,
        pedal_brake=None,
        pedal_throttle_max=None,
        pedal_brake_max=None,
        nav_mode_active=None,
        joystick_vrx=None,
        joystick_vry=None,
        joystick_sw_pressed=None,
    ):
        if mode is not None:
            self._drive_status["mode"] = str(mode).upper()
        if drivetrain is not None:
            self._drive_status["drivetrain"] = str(drivetrain).upper()
        self._drive_status["ticks"] = ticks
        if left_blinker is not None:
            self._drive_status["left_blinker"] = bool(left_blinker)
        if right_blinker is not None:
            self._drive_status["right_blinker"] = bool(right_blinker)
        self._drive_status["pedal_throttle"] = pedal_throttle
        self._drive_status["pedal_brake"] = pedal_brake
        self._drive_status["pedal_throttle_max"] = pedal_throttle_max
        self._drive_status["pedal_brake_max"] = pedal_brake_max
        if nav_mode_active is not None:
            self._drive_status["nav_mode_active"] = bool(nav_mode_active)
        self._drive_status["joystick_vrx"] = joystick_vrx
        self._drive_status["joystick_vry"] = joystick_vry
        self._drive_status["joystick_sw_pressed"] = (
            None if joystick_sw_pressed is None else bool(joystick_sw_pressed)
        )

    def handle_event(self, event):
        if not self.show_pp_tuning:
            return False

        panel_rect, value_rects, _ = self._get_cached_layout()
        self._pp_value_rects = value_rects

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self._editing_key is not None and not panel_rect.collidepoint(event.pos):
                self._commit_edit()
                self._editing_key = None
                return False

            for key, rect in value_rects.items():
                if rect.collidepoint(event.pos):
                    if self._editing_key is not None and self._editing_key != key:
                        self._commit_edit()
                    self._editing_key = key
                    self._edit_buffer = self._format_value_for_edit(key)
                    return True

            if self._editing_key is not None:
                self._commit_edit()
                self._editing_key = None
            return panel_rect.collidepoint(event.pos)

        if event.type == pygame.KEYDOWN and self._editing_key is not None:
            if event.key == pygame.K_RETURN:
                self._commit_edit()
                self._editing_key = None
                return True
            if event.key == pygame.K_ESCAPE:
                self._editing_key = None
                return True
            if event.key == pygame.K_BACKSPACE:
                self._edit_buffer = self._edit_buffer[:-1]
                return True
            if event.unicode and event.unicode.isprintable():
                if len(self._edit_buffer) < 32:
                    self._edit_buffer += event.unicode
                return True
            return True

        if self._editing_key is not None and event.type == pygame.KEYUP:
            return True

        return False

    def add_info(self, title, info):
        self._info_text[title] = info

    def notification(self, text, seconds=2.0):

        pass

    def render(self, display):
        if self._drive_status.get("nav_mode_active", False):
            return
        if self.show_info:
            self._render_keybinds_panel(display)
        if not self.show_pp_tuning:
            return

        panel_rect, value_rects, row_height = self._get_cached_layout()
        self._pp_value_rects = value_rects

        static_panel = self._get_pp_static_panel(panel_rect, row_height)
        values_layer = self._get_pp_values_layer(panel_rect, value_rects)
        drive_layer = self._get_pp_drive_status_layer(panel_rect)

        display.blit(static_panel, panel_rect.topleft)
        display.blit(values_layer, panel_rect.topleft)
        display.blit(drive_layer, (panel_rect.x, panel_rect.y + self._pp_drive_start_y))

        now_ms = pygame.time.get_ticks()
        if self._pp_status and now_ms < self._pp_status_until:
            signature = self._pp_status
            if signature != self._pp_status_surface_signature:
                self._pp_status_surface_cache = self._body_font.render(
                    self._pp_status, True, COLOR_SCARLET_RED_0
                )
                self._pp_status_surface_signature = signature
            if self._pp_status_surface_cache is not None:
                display.blit(
                    self._pp_status_surface_cache,
                    (panel_rect.x + 8, panel_rect.y + panel_rect.height - 20),
                )
        else:
            self._pp_status_surface_cache = None
            self._pp_status_surface_signature = None

    def _render_map_mode_hints(self, display):
        cache_key = (self.dim, tuple(self._map_mode_hint_lines))
        if self._map_mode_panel_cache is None or self._map_mode_panel_cache_key != cache_key:
            margin = 10
            row_h = 18
            title_h = 24
            panel_w = max(300, max(self._mono_font.size(line)[0] for line in self._map_mode_hint_lines) + 16)
            panel_h = title_h + (len(self._map_mode_hint_lines) * row_h) + 10
            panel_rect = pygame.Rect(
                self.dim[0] - panel_w - margin,
                self.dim[1] - panel_h - margin,
                panel_w,
                panel_h,
            )

            panel = pygame.Surface((panel_rect.width, panel_rect.height), pygame.SRCALPHA)
            panel.fill((0, 0, 0, 145))
            pygame.draw.rect(panel, COLOR_ALUMINIUM_4, panel.get_rect(), 1)

            title = self._header_font.render("Map Mode", True, COLOR_WHITE)
            panel.blit(title, (8, 6))

            y = title_h
            for line in self._map_mode_hint_lines:
                text = self._mono_font.render(line, True, COLOR_ALUMINIUM_2)
                panel.blit(text, (8, y))
                y += row_h

            self._map_mode_panel_cache = panel
            self._map_mode_panel_cache_rect = panel_rect
            self._map_mode_panel_cache_key = cache_key

        display.blit(self._map_mode_panel_cache, self._map_mode_panel_cache_rect.topleft)

    def _render_keybinds_panel(self, display):
        cache_key = (self.dim, tuple(self._keybind_lines))
        if self._keybind_panel_cache is None or self._keybind_panel_cache_key != cache_key:
            margin = 10
            panel_w = min(460, max(280, self.dim[0] - 20))
            row_h = 18
            title_h = 24
            panel_h = title_h + (len(self._keybind_lines) * row_h) + 12
            panel_rect = pygame.Rect(self.dim[0] - panel_w - margin, margin, panel_w, panel_h)

            panel = pygame.Surface((panel_rect.width, panel_rect.height), pygame.SRCALPHA)
            panel.fill((0, 0, 0, 145))
            pygame.draw.rect(panel, COLOR_ALUMINIUM_4, panel.get_rect(), 1)

            title = self._header_font.render("Keybind Directory", True, COLOR_WHITE)
            panel.blit(title, (8, 6))

            y = title_h
            for line in self._keybind_lines:
                text = self._mono_font.render(line, True, COLOR_ALUMINIUM_2)
                panel.blit(text, (8, y))
                y += row_h

            self._keybind_panel_cache = panel
            self._keybind_panel_cache_rect = panel_rect
            self._keybind_panel_cache_key = cache_key

        display.blit(self._keybind_panel_cache, self._keybind_panel_cache_rect.topleft)

    def _invalidate_pp_panel_caches(self):
        self._pp_static_panel_cache = None
        self._pp_static_panel_cache_key = None
        self._pp_values_layer_cache = None
        self._pp_values_layer_signature = None
        self._pp_drive_layer_cache = None
        self._pp_drive_line_cache = {}
        self._pp_status_surface_cache = None
        self._pp_status_surface_signature = None

    def _get_cached_layout(self):
        layout_key = (self.dim, len(self._pp_fields))
        if self._pp_layout_cache is None or self._pp_layout_cache_key != layout_key:
            self._pp_layout_cache = self._compute_layout()
            self._pp_layout_cache_key = layout_key
            self._invalidate_pp_panel_caches()
        return self._pp_layout_cache

    def _get_pp_static_panel(self, panel_rect, row_height):
        cache_key = (panel_rect.size, row_height, tuple(self._pp_fields))
        if self._pp_static_panel_cache is not None and self._pp_static_panel_cache_key == cache_key:
            return self._pp_static_panel_cache

        panel = pygame.Surface((panel_rect.width, panel_rect.height), pygame.SRCALPHA)
        panel.fill((0, 0, 0, 140))
        pygame.draw.rect(panel, COLOR_ALUMINIUM_4, panel.get_rect(), 1)

        title = self._header_font.render("Pure Pursuit Tuning", True, COLOR_WHITE)
        panel.blit(title, (8, 6))

        y = 26
        for _, label, _ in self._pp_fields:
            label_surf = self._body_font.render(label, True, COLOR_ALUMINIUM_2)
            panel.blit(label_surf, (8, y + 4))
            y += row_height

        y += 4
        pygame.draw.line(panel, COLOR_ALUMINIUM_3, (8, y), (panel_rect.width - 8, y), 1)
        y += 8

        status_title = self._header_font.render("Driving Status", True, COLOR_WHITE)
        panel.blit(status_title, (8, y))
        y += 20

        self._pp_drive_start_y = y
        self._pp_drive_row_h = 18
        self._pp_drive_layer_cache = None
        self._pp_drive_line_cache = {}

        self._pp_static_panel_cache = panel
        self._pp_static_panel_cache_key = cache_key
        return self._pp_static_panel_cache

    def _get_pp_values_layer(self, panel_rect, value_rects):
        signature_items = []
        for key, _, _ in self._pp_fields:
            is_editing = self._editing_key == key
            value_text = (self._edit_buffer + "_") if is_editing else self._format_value(key)
            signature_items.append((key, is_editing, value_text))
        signature = tuple(signature_items)

        if self._pp_values_layer_cache is not None and signature == self._pp_values_layer_signature:
            return self._pp_values_layer_cache

        layer = pygame.Surface((panel_rect.width, panel_rect.height), pygame.SRCALPHA)
        for key, _, _ in self._pp_fields:
            value_rect = pygame.Rect(value_rects[key])
            value_rect.x -= panel_rect.x
            value_rect.y -= panel_rect.y
            is_editing = self._editing_key == key
            bg = COLOR_ORANGE_0 if is_editing else COLOR_ALUMINIUM_5
            pygame.draw.rect(layer, bg, value_rect)
            pygame.draw.rect(layer, COLOR_ALUMINIUM_3, value_rect, 1)

            value_text = (self._edit_buffer + "_") if is_editing else self._format_value(key)
            value_surf = self._mono_font.render(value_text, True, COLOR_WHITE)
            layer.blit(value_surf, (value_rect.x + 6, value_rect.y + 4))

        self._pp_values_layer_cache = layer
        self._pp_values_layer_signature = signature
        return self._pp_values_layer_cache

    def _get_drive_status_lines(self):
        mode_text = self._drive_status.get("mode", "MANUAL")
        drivetrain_text = self._drive_status.get("drivetrain", "DRIVE")
        ticks = self._drive_status.get("ticks")
        ticks_text = "--" if ticks is None else f"{float(ticks):.0f}"
        left_on = self._drive_status.get("left_blinker", False)
        right_on = self._drive_status.get("right_blinker", False)
        pedal_throttle = self._drive_status.get("pedal_throttle")
        pedal_brake = self._drive_status.get("pedal_brake")
        pedal_throttle_max = self._drive_status.get("pedal_throttle_max")
        pedal_brake_max = self._drive_status.get("pedal_brake_max")
        joystick_vrx = self._drive_status.get("joystick_vrx")
        joystick_vry = self._drive_status.get("joystick_vry")
        joystick_sw_pressed = self._drive_status.get("joystick_sw_pressed")
        throttle_text = "--" if pedal_throttle is None else f"{float(pedal_throttle):.0f}"
        brake_text = "--" if pedal_brake is None else f"{float(pedal_brake):.0f}"
        throttle_max_text = "--" if pedal_throttle_max is None else f"{float(pedal_throttle_max):.0f}"
        brake_max_text = "--" if pedal_brake_max is None else f"{float(pedal_brake_max):.0f}"
        joystick_vrx_text = "--" if joystick_vrx is None else f"{int(joystick_vrx)}"
        joystick_vry_text = "--" if joystick_vry is None else f"{int(joystick_vry)}"
        if joystick_sw_pressed is None:
            joystick_sw_text = "--"
        else:
            joystick_sw_text = "PRESSED" if joystick_sw_pressed else "RELEASED"
        return [
            f"Mode: {mode_text}",
            f"Gear: {drivetrain_text}",
            f"Ticks: {ticks_text}",
            f"Blinkers: L {'ON' if left_on else 'OFF'}  R {'ON' if right_on else 'OFF'}",
            f"Pedals ticks: Throttle {throttle_text}/{throttle_max_text}  Brake {brake_text}/{brake_max_text}",
            f"Joystick: SW {joystick_sw_text}  VRX {joystick_vrx_text}  VRY {joystick_vry_text}",
        ]

    def _get_pp_drive_status_layer(self, panel_rect):
        lines = self._get_drive_status_lines()
        layer_h = self._pp_drive_row_h * len(lines)
        if (
            self._pp_drive_layer_cache is None
            or self._pp_drive_layer_cache.get_width() != panel_rect.width
            or self._pp_drive_layer_cache.get_height() != layer_h
        ):
            self._pp_drive_layer_cache = pygame.Surface((panel_rect.width, layer_h), pygame.SRCALPHA)
            self._pp_drive_line_cache = {}

        for idx, line in enumerate(lines):
            if self._pp_drive_line_cache.get(idx) == line:
                continue
            row_rect = pygame.Rect(0, idx * self._pp_drive_row_h, panel_rect.width, self._pp_drive_row_h)
            self._pp_drive_layer_cache.fill((0, 0, 0, 0), row_rect)
            text_surf = self._mono_font.render(line, True, COLOR_WHITE)
            self._pp_drive_layer_cache.blit(text_surf, (8, idx * self._pp_drive_row_h))
            self._pp_drive_line_cache[idx] = line

        return self._pp_drive_layer_cache

    def _compute_layout(self):
        margin = 10
        panel_w = 380
        row_h = 22
        header_h = 24
        status_h = 18
        drive_h = 112
        panel_h = header_h + row_h * len(self._pp_fields) + drive_h + status_h + 10
        panel_rect = pygame.Rect(margin, margin, panel_w, panel_h)

        value_rects = {}
        y = panel_rect.y + header_h + 2
        for key, _, _ in self._pp_fields:
            value_rects[key] = pygame.Rect(panel_rect.x + 210, y, 150, row_h - 4)
            y += row_h

        return panel_rect, value_rects, row_h

    def _sync_from_controller(self):
        c = self._autonomy_controller
        if c is None:
            return

        self._pp_values["target_speed_kmh"] = getattr(c, "target_speed_kmh", c.target_speed * 3.6)
        self._pp_values["lookahead_distance"] = c.lookahead_distance
        self._pp_values["adaptive_lookahead"] = getattr(c, "adaptive_lookahead", True)
        self._pp_values["lookahead_speed_gain"] = getattr(c, "lookahead_speed_gain", 0.25)
        self._pp_values["lookahead_min"] = getattr(c, "lookahead_min", 2.0)
        self._pp_values["lookahead_max"] = getattr(c, "lookahead_max", 12.0)
        self._pp_values["steer_gain"] = getattr(c, "steer_gain", 1.0)
        self._pp_values["throttle_low"] = getattr(c, "throttle_low", 0.2)
        self._pp_values["throttle_high"] = getattr(c, "throttle_high", 0.5)
        self._pp_values["wheelbase_override"] = getattr(c, "_wheelbase_override", None)

    def _format_value_for_edit(self, key):
        value = self._pp_values.get(key)
        if key == "wheelbase_override":
            return "" if value is None else f"{value:.3f}"
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, (int, float)):
            return f"{value:.3f}"
        return str(value)

    def _format_value(self, key):
        value = self._pp_values.get(key)
        if key == "wheelbase_override":
            return "auto" if value is None else f"{value:.2f}"
        if isinstance(value, bool):
            return "on" if value else "off"
        if isinstance(value, (int, float)):
            return f"{value:.2f}"
        return str(value)

    def _commit_edit(self):
        key = self._editing_key
        if key is None:
            return

        try:
            value = self._parse_value(key, self._edit_buffer.strip())
            value = self._sanitize_value(key, value)
            self._pp_values[key] = value
            self._apply_value_to_controller(key, value)

            if key in ("lookahead_min", "lookahead_max"):
                self._enforce_lookahead_bounds()
        except ValueError as exc:
            self._pp_status = str(exc)
            self._pp_status_until = pygame.time.get_ticks() + 1500

    def _parse_value(self, key, text):
        if key == "adaptive_lookahead":
            val = text.lower()
            if val in ("true", "1", "on", "yes"):
                return True
            if val in ("false", "0", "off", "no"):
                return False
            raise ValueError("adaptive_lookahead expects true/false")

        if key == "wheelbase_override":
            if text == "" or text.lower() in ("auto", "none"):
                return None
            return float(text)

        return float(text)

    def _sanitize_value(self, key, value):
        if key in ("lookahead_distance", "lookahead_min", "lookahead_max"):
            return max(0.1, float(value))
        if key in ("lookahead_speed_gain", "steer_gain", "target_speed_kmh"):
            return max(0.0, float(value))
        if key in ("throttle_low", "throttle_high"):
            return max(0.0, min(1.0, float(value)))
        if key == "wheelbase_override" and value is not None:
            return max(0.5, float(value))
        return value

    def _enforce_lookahead_bounds(self):
        min_val = self._pp_values["lookahead_min"]
        max_val = self._pp_values["lookahead_max"]
        if min_val > max_val:
            self._pp_values["lookahead_max"] = min_val
            self._apply_value_to_controller("lookahead_max", min_val)

    def _apply_value_to_controller(self, key, value):
        c = self._autonomy_controller
        if c is None:
            return

        if key == "target_speed_kmh":
            c.target_speed_kmh = value
            c.target_speed = value / 3.6
            return
        if key == "wheelbase_override":
            if hasattr(c, "set_wheelbase_override"):
                c.set_wheelbase_override(value)
            else:
                c._wheelbase = c._compute_wheelbase_m() if value is None else float(value)
            return

        setattr(c, key, value)



