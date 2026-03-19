
import pygame
import carla
import sys

from pygame.locals import (
    KMOD_CTRL, KMOD_SHIFT,
    K_DOWN, K_ESCAPE, K_F1, K_F2, K_LEFT, K_RIGHT,
    K_SLASH, K_TAB, K_UP,
    K_a, K_b, K_c, K_d, K_h, K_l, K_p, K_q, K_s, K_w, K_v,
)
from Modules.actuation import SimpleRouteFollower

DRIVE_ACTUATION_ENABLED = True
AUTONOMY_ENABLED = True

PEDAL_THROTTLE_DEADZONE = 0.04
PEDAL_BRAKE_DEADZONE = 0.06
PEDAL_DOMINANCE_MARGIN = 0.02


class InputControl:
    """Handles input received such as keyboard and mouse."""

    def __init__(self, name):
        self.name = name
        self.mouse_pos = (0, 0)
        self.mouse_offset = [0.0, 0.0]
        self.wheel_offset = 0.1
        self.wheel_amount = 0.025
        self._steer_cache = 0.0
        self.control = None
        self._autopilot_enabled = False
        self._external_steer_provider = None
        self._external_pedal_provider = None

        self._hud = None
        self._world = None

    def start(self, hud, world):
        self._hud = hud
        self._world = world
        self._hud.notification("Press F1 for keybinds.", seconds=4.0)

    def render(self, display):
        pass

    def tick(self, clock):
        self.parse_input(clock)

    def set_external_steer_provider(self, provider):
        """Set a callback returning normalized steering [-1, 1] or None."""
        self._external_steer_provider = provider

    def set_external_pedal_provider(self, provider):
        """Set a callback returning (throttle, brake), each in [0, 1], or None values."""
        self._external_pedal_provider = provider

    def is_builtin_autopilot_enabled(self):
        return bool(self._autopilot_enabled)

    def clear_user_route_state(self):
        self._clear_user_route_state()

    def on_hero_respawned(self):
        """Reset local autopilot state when hero actor is replaced."""
        self._autopilot_enabled = False

    def disable_builtin_autopilot(self):
        self._disable_builtin_autopilot()

    def toggle_drive_reverse(self):
        """Toggle manual drivetrain state. Returns True when reverse is active."""
        if not isinstance(self.control, carla.VehicleControl):
            return False
        self.control.gear = 1 if (self.control.reverse or self.control.gear < 0) else -1
        self.control.reverse = self.control.gear < 0
        return self.control.reverse

    def _clear_user_route_state(self):
        if self._world is None:
            return
        if hasattr(self._world, "set_current_route"):
            self._world.set_current_route(None)
        else:
            self._world.current_route = None
        self._world.last_destination = None
        self._world.clicked_points = []

    def _disable_builtin_autopilot(self):
        if self._autopilot_enabled and self._world is not None and self._world.hero_actor is not None:
            self._autopilot_enabled = False
            self._world.hero_actor.set_autopilot(False)

    def _build_autonomy_controller(self, route):
        if not AUTONOMY_ENABLED:
            return None
        params = self._hud.get_pp_params() if self._hud is not None else {}
        return SimpleRouteFollower(
            vehicle=self._world.hero_actor,
            route=route,
            target_speed_kmh=params.get("target_speed_kmh", 30.0),
            lookahead_distance=params.get("lookahead_distance", 1.0),
            steer_gain=params.get("steer_gain", 1.0),
            throttle_low=params.get("throttle_low", 0.2),
            throttle_high=params.get("throttle_high", 0.5),
            adaptive_lookahead=params.get("adaptive_lookahead", True),
            lookahead_speed_gain=params.get("lookahead_speed_gain", 0.25),
            lookahead_min=params.get("lookahead_min", 2.0),
            lookahead_max=params.get("lookahead_max", 12.0),
            wheelbase_override=params.get("wheelbase_override", None),
            tl_state_provider=lambda: self._world.latest_tl_cluster_state,
            reroute_callback=self._reroute_to_destination,
        )

    def _reroute_to_destination(self, destination):
        if self._world is None or self._world.hero_actor is None or destination is None:
            return None
        try:
            current_loc = self._world.hero_actor.get_location()
            route = self._world.compute_route(current_loc, destination)
        except Exception:
            return None
        if not route:
            return None
        self._world.set_current_route(route)
        return route

    def engage_autonomy_from_last_destination(self):
        """Enable autonomy only when currently manual and a destination already exists."""
        if not AUTONOMY_ENABLED:
            if self._hud is not None:
                self._hud.notification("Autonomy disabled")
            return False
        if self._world is None or self._world.hero_actor is None:
            return False
        if self._world.autonomy_controller is not None:
            return False

        destination = self._world.last_destination
        if destination is None and self._world.current_route:
            try:
                destination = self._world.current_route[-1][0].transform.location
                self._world.last_destination = destination
            except Exception:
                destination = None

        if destination is None:
            if self._hud is not None:
                self._hud.notification("No destination set")
            return False

        self._disable_builtin_autopilot()
        current_loc = self._world.hero_actor.get_location()
        route = self._world.compute_route(current_loc, destination)
        if not route:
            if self._hud is not None:
                self._hud.notification("Route recalculation failed")
            return False
        self._world.set_current_route(route)
        self._world.autonomy_controller = self._build_autonomy_controller(route)
        if self._hud is not None:
            self._hud.set_autonomy_controller(self._world.autonomy_controller)
            self._hud.notification("Autonomy On")
        return True

    def set_navigation_destination_screen(self, sx, sy):
        if self._world is None or not self._world.show_navigation:
            return False

        world_loc = self._world.screen_to_world(int(sx), int(sy))
        self._world.clicked_points = [world_loc]

        if self._world.hero_actor:


            self._disable_builtin_autopilot()

            current_loc = self._world.hero_actor.get_location()
            route = self._world.compute_route(current_loc, world_loc)
            self._world.set_current_route(route)
            self._world.last_destination = world_loc

            if AUTONOMY_ENABLED:
                self._world.autonomy_controller = self._build_autonomy_controller(route)
                if self._hud is not None:
                    self._hud.set_autonomy_controller(self._world.autonomy_controller)

        return True

    def _parse_events(self):
        self.mouse_pos = pygame.mouse.get_pos()
        for event in pygame.event.get():
            if self._hud is not None and self._hud.handle_event(event):
                continue
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    pygame.quit()
                    sys.exit()


                elif event.key == K_F1:
                    self._hud.show_info = not self._hud.show_info

                elif event.key == K_F2:
                    self._hud.set_pp_tuning_visible(not self._hud.show_pp_tuning)
                    self._hud.notification("PP Tuning ON" if self._hud.show_pp_tuning else "PP Tuning OFF")

                elif event.key == K_TAB:
                    self._world.show_navigation = not self._world.show_navigation
                    self._world._sync_camera_surface_conversion()

                    if self._world.show_navigation:

                        if hasattr(self._world, "center_map_on_map_center"):
                            self._world.center_map_on_map_center()
                        if hasattr(self._world, "reset_nav_crosshair"):
                            self._world.reset_nav_crosshair()


                        self._world._recenter_nav_next_tick = True

                    self._hud.notification("Navigation ON" if self._world.show_navigation else "Navigation OFF")

                elif event.key == K_v:
                    show_rgb = self._world.toggle_camera_mode()
                    self._hud.notification("RGB Camera" if show_rgb else "Third-Person Camera")

                elif event.key == K_b:
                    show_boxes = self._world.toggle_yolo_boxes()
                    self._hud.notification("YOLO Boxes ON" if show_boxes else "YOLO Boxes OFF")

                elif event.key == K_l:
                    show_route = self._world.toggle_route_debug_3d()
                    self._hud.notification("3D Route Line ON" if show_route else "3D Route Line OFF")

                elif event.key == K_c:
                    if not AUTONOMY_ENABLED:
                        self._hud.notification("Autonomy disabled")
                        return
                    if self._world.hero_actor is None:
                        return

                    if self._world.autonomy_controller is not None:
                        self._world.autonomy_controller = None
                        if self._hud is not None:
                            self._hud.set_autonomy_controller(None)
                        self._hud.notification("Autonomy Off")
                        return

                    self.engage_autonomy_from_last_destination()
                    return

                elif isinstance(self.control, carla.VehicleControl):
                    if event.key == K_q:
                        self.control.gear = 1 if self.control.reverse else -1

                    elif event.key == K_p:
                        if not AUTONOMY_ENABLED:
                            self._hud.notification("Autonomy disabled")
                            continue
                        if self._world.hero_actor is not None:
                            self._autopilot_enabled = not self._autopilot_enabled
                            if self._autopilot_enabled:

                                self._world.autonomy_controller = None
                                if self._hud is not None:
                                    self._hud.set_autonomy_controller(None)
                                self.clear_user_route_state()
                            self._world.hero_actor.set_autopilot(self._autopilot_enabled)
                            self._hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if self._world.show_navigation and event.button in (1, 2, 4, 5):

                    continue

    def _parse_keys(self, milliseconds):
        keys = pygame.key.get_pressed()
        keyboard_throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        keyboard_brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self.control.throttle = keyboard_throttle
        self.control.brake = keyboard_brake
        keyboard_steer_left = bool(keys[K_LEFT] or keys[K_a])
        keyboard_steer_right = bool(keys[K_RIGHT] or keys[K_d])
        keyboard_steer_active = keyboard_steer_left or keyboard_steer_right

        external_steer = None
        if self._external_steer_provider is not None:
            try:
                external_steer = self._external_steer_provider()
            except Exception:
                external_steer = None

        if external_steer is None or keyboard_steer_active:
            steer_increment = 5e-4 * milliseconds
            if keyboard_steer_left:
                self._steer_cache -= steer_increment
            elif keyboard_steer_right:
                self._steer_cache += steer_increment
            else:
                self._steer_cache = 0.0

            self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
            self.control.steer = round(self._steer_cache, 1)
        else:
            self.control.steer = max(-1.0, min(1.0, float(external_steer)))
            self._steer_cache = self.control.steer

        pedal_throttle = None
        pedal_brake = None
        if self._external_pedal_provider is not None:
            try:
                pedal_values = self._external_pedal_provider()
                if pedal_values is not None:
                    pedal_throttle, pedal_brake = pedal_values
            except Exception:
                pedal_throttle, pedal_brake = None, None

        if pedal_throttle is not None:
            throttle_value = max(0.0, min(1.0, float(pedal_throttle)))
            self.control.throttle = throttle_value if throttle_value >= PEDAL_THROTTLE_DEADZONE else 0.0
        if pedal_brake is not None:
            brake_value = max(0.0, min(1.0, float(pedal_brake)))
            self.control.brake = brake_value if brake_value >= PEDAL_BRAKE_DEADZONE else 0.0

        if self.control.throttle > 0.0 and self.control.brake > 0.0:
            if self.control.throttle >= self.control.brake:
                self.control.brake = 0.0
            else:
                self.control.throttle = 0.0

        if keyboard_throttle > 0.0:
            self.control.throttle = 1.0
        if keyboard_brake > 0.0:
            self.control.brake = 1.0

        if self.control.brake > 0.05:
            self.control.throttle = 0.0
        self.control.hand_brake = False

    def _parse_mouse(self):

        return

    def parse_input(self, clock):
        self._parse_events()
        self._parse_mouse()

        if not DRIVE_ACTUATION_ENABLED:
            return


        if self._world.autonomy_controller is not None:
            return

        if not self._autopilot_enabled:
            if isinstance(self.control, carla.VehicleControl):
                self._parse_keys(clock.get_time())
                self.control.reverse = self.control.gear < 0
            if self._world.hero_actor is not None:
                self._world.hero_actor.apply_control(self.control)

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)




