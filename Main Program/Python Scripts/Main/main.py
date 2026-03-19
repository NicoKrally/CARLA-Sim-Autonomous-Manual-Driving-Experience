import os
import sys
import glob
import time
import random
import argparse
import logging
import threading
import pygame

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON_SCRIPTS_DIR = os.path.dirname(PROJECT_DIR)
HARDWARE_BRIDGE_DIR = os.path.join(PYTHON_SCRIPTS_DIR, "Hardware bridge")


def find_carla_root(start_dir):
    """Walk upward until we find CARLA's PythonAPI/carla/dist folder."""
    current = os.path.abspath(start_dir)
    while True:
        candidate = os.path.join(current, "PythonAPI", "carla", "dist")
        if os.path.isdir(candidate):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            raise RuntimeError(
                f"Could not locate CARLA root from {start_dir}. Expected PythonAPI/carla/dist in a parent directory."
            )
        current = parent


CARLA_ROOT = find_carla_root(PROJECT_DIR)
MAIN_PROGRAM_DIR = os.path.dirname(PROJECT_DIR)
MEDIA_CONTENT_DIR = os.path.join(MAIN_PROGRAM_DIR, "Main supplementary")
LOADING_GIF_DIR = os.path.join(MEDIA_CONTENT_DIR, "Loading gifs")

dist_dir = os.path.join(CARLA_ROOT, "PythonAPI", "carla", "dist")
whls = glob.glob(os.path.join(dist_dir, "carla-*-cp*-win_amd64.whl"))

if not whls:
    raise RuntimeError(
        f"CARLA .whl not found in {dist_dir}. Available files: {os.listdir(dist_dir) if os.path.isdir(dist_dir) else 'dist dir missing'}"
    )

sys.path.append(whls[0])

sys.path.append(os.path.join(CARLA_ROOT, "PythonAPI", "carla"))

sys.path.insert(0, PROJECT_DIR)
sys.path.insert(0, PYTHON_SCRIPTS_DIR)
sys.path.insert(0, HARDWARE_BRIDGE_DIR)

from Modules.world import World
from Modules.hud import HUD
from Modules.input_control import InputControl
from hardware_bridge import HardwareBridge, PedalsBridge, ButtonsBridge
from Modules.constants import (
    TITLE_WORLD,
    TITLE_HUD,
    TITLE_INPUT,
    COLOR_WHITE,
    COLOR_ALUMINIUM_4,
)

PEDAL_STALE_TIMEOUT_S = 0.2
REZERO_KEY = pygame.K_F8
REZERO_KEY_LABEL = "F8"
CARLA_AUTOPILOT_TAKEOVER_GRACE_S = 1.0
MANUAL_TAKEOVER_KEYS = (pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d)
PEDAL_TAKEOVER_MIN_COMMAND = 0.35
PEDAL_TAKEOVER_HOLD_S = 0.35

def _load_random_loading_gif_frames(display_size):
    gif_paths = glob.glob(os.path.join(LOADING_GIF_DIR, "*.gif"))
    if not gif_paths:
        return [], [], None

    chosen_path = random.choice(gif_paths)
    max_dim = int(min(display_size) * 0.45)

    try:
        from PIL import Image, ImageSequence

        img = Image.open(chosen_path)
        frames = []
        durations = []
        for frame in ImageSequence.Iterator(img):
            rgba = frame.convert("RGBA")
            surface = pygame.image.fromstring(rgba.tobytes(), rgba.size, "RGBA").convert_alpha()

            w, h = surface.get_size()
            scale = min(1.0, float(max_dim) / float(max(w, h)))
            if scale < 1.0:
                surface = pygame.transform.smoothscale(surface, (max(1, int(w * scale)), max(1, int(h * scale))))

            frames.append(surface)
            durations.append(max(0.03, float(frame.info.get("duration", 100)) / 1000.0))

        if not frames:
            raise RuntimeError("GIF had no frames")

        return frames, durations, chosen_path
    except Exception:
        surface = pygame.image.load(chosen_path).convert_alpha()
        w, h = surface.get_size()
        scale = min(1.0, float(max_dim) / float(max(w, h)))
        if scale < 1.0:
            surface = pygame.transform.smoothscale(surface, (max(1, int(w * scale)), max(1, int(h * scale))))
        return [surface], [0.2], chosen_path


def _run_loading_animation_until_done(display, font, done_event, done_payload):
    frames, frame_durations, chosen_path = _load_random_loading_gif_frames(display.get_size())
    if not chosen_path:
        logging.warning("No GIF files found in loading folder: %s", LOADING_GIF_DIR)

    clock = pygame.time.Clock()
    frame_idx = 0
    last_frame_switch = time.time()

    while not done_event.is_set():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done_payload["error"] = KeyboardInterrupt()
                done_event.set()
                return

        now = time.time()
        if frames and len(frames) > 1:
            frame_time = frame_durations[frame_idx]
            if (now - last_frame_switch) >= frame_time:
                frame_idx = (frame_idx + 1) % len(frames)
                last_frame_switch = now

        display.fill((0, 0, 0))

        if frames:
            icon = frames[frame_idx]
            icon_rect = icon.get_rect(center=(display.get_width() // 2, display.get_height() // 2))
            display.blit(icon, icon_rect)
            text_top = icon_rect.bottom + 16
        else:
            text_top = (display.get_height() // 2) + 16

        loading_frames = ("Loading", "Loading.", "Loading..", "Loading...")
        loading_frame_interval = 0.65
        text_value = loading_frames[int((now / loading_frame_interval) % len(loading_frames))]
        text = font.render(text_value, True, COLOR_WHITE)
        text_rect = text.get_rect(midtop=(display.get_width() // 2, text_top))
        display.blit(text, text_rect)

        pygame.display.flip()
        clock.tick(60)


def game_loop(args):
    try:
        import ctypes
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

    pygame.init()
    pygame.font.init()

    screen_w, screen_h = 1920, 1200

    display = pygame.display.set_mode(
        (screen_w, screen_h),
        pygame.DOUBLEBUF
    )
    pygame.mouse.set_visible(False)

    render_w, render_h = 1920, 1200

    args.width, args.height = render_w, render_h
    white_icon = pygame.Surface((32, 32))
    white_icon.fill((255, 255, 255))
    pygame.display.set_icon(white_icon)
    pygame.display.set_caption("")

    font = pygame.font.Font(pygame.font.get_default_font(), 20)
    world = None
    hud = None
    input_control = None
    hardware_bridge = None
    pedals_bridge = None
    buttons_bridge = None
    init_done = threading.Event()
    init_result = {"error": None}

    def _get_safe_pedal_ticks():
        if pedals_bridge is None:
            return 0.0, 0.0

        throttle = float(pedals_bridge.throttle) if pedals_bridge.throttle is not None else 0.0
        brake = float(pedals_bridge.brake) if pedals_bridge.brake is not None else 0.0
        throttle = max(0.0, throttle)
        brake = max(0.0, brake)

        if pedals_bridge.last_update_s is None:
            return 0.0, 0.0
        if (time.monotonic() - pedals_bridge.last_update_s) > PEDAL_STALE_TIMEOUT_S:
            return 0.0, 0.0

        return throttle, brake

    def _get_safe_pedal_commands():
        throttle_ticks, brake_ticks = _get_safe_pedal_ticks()
        if pedals_bridge is None:
            return 0.0, 0.0

        throttle_max = float(getattr(pedals_bridge, "throttle_range_ticks", 0.0) or 0.0)
        brake_max = float(getattr(pedals_bridge, "brake_range_ticks", 0.0) or 0.0)
        throttle_cmd = (throttle_ticks / throttle_max) if throttle_max > 0.0 else 0.0
        brake_cmd = (brake_ticks / brake_max) if brake_max > 0.0 else 0.0
        throttle_cmd = max(0.0, min(1.0, throttle_cmd))
        brake_cmd = max(0.0, min(1.0, brake_cmd))
        return throttle_cmd, brake_cmd
    def _cleanup_finished_custom_autonomy():
        if world is None:
            return False
        controller = getattr(world, "autonomy_controller", None)
        if controller is None:
            return False
        if not bool(getattr(controller, "finished", False)):
            return False

        world.autonomy_controller = None
        input_control.clear_user_route_state()
        hud.set_autonomy_controller(None)


        if world.hero_actor is not None:
            try:
                completion_control = world.hero_actor.get_control()
                completion_control.throttle = 0.0
                completion_control.brake = 1.0
                completion_control.steer = 0.0
                completion_control.hand_brake = False
                completion_control.reverse = False
                world.hero_actor.apply_control(completion_control)
            except Exception:
                pass

        if hardware_bridge is not None:
            hardware_bridge.reset_takeover_detection()

        hud.notification("Autonomy complete")
        logging.info("Custom autonomy completed: controller cleared and route state reset.")
        return True

    def _initialize_runtime():
        nonlocal world, hud, input_control, hardware_bridge, pedals_bridge, buttons_bridge
        try:
            input_control = InputControl(TITLE_INPUT)
            hud = HUD(TITLE_HUD, args.width, args.height)
            world = World(TITLE_WORLD, args, timeout=args.timeout)

            input_control.start(hud, world)
            hud.start()
            world.start(hud, input_control)
            hardware_bridge = HardwareBridge()
            buttons_bridge = ButtonsBridge(
                exclude_port=hardware_bridge.com_port if hardware_bridge is not None else None
            )
            pedals_bridge = PedalsBridge(
                exclude_port=hardware_bridge.com_port if hardware_bridge is not None else None
            )
            input_control.set_external_steer_provider(hardware_bridge.get_manual_steer)
            input_control.set_external_pedal_provider(_get_safe_pedal_commands)
        except Exception as exc:
            init_result["error"] = exc
        finally:
            init_done.set()

    init_thread = threading.Thread(target=_initialize_runtime, daemon=True)
    init_thread.start()

    _run_loading_animation_until_done(display, font, init_done, init_result)

    try:
        if init_result["error"] is not None:
            raise init_result["error"]
        if world is None or hud is None or input_control is None:
            raise RuntimeError("Initialization did not complete successfully.")

        clock = pygame.time.Clock()
        rezero_key_was_down = False
        prev_built_in_autopilot_active = False
        built_in_autopilot_engaged_s = None
        pedal_takeover_start_s = None
        joystick_sw_was_pressed = False
        while True:
            clock.tick(60)

            world.tick(clock)
            if _cleanup_finished_custom_autonomy():
                joystick_sw_was_pressed = False
            now_s = time.monotonic()
            built_in_autopilot_active = (
                input_control.is_builtin_autopilot_enabled()
                if input_control is not None
                else False
            )
            if built_in_autopilot_active and not prev_built_in_autopilot_active:
                built_in_autopilot_engaged_s = now_s
                if hardware_bridge is not None:
                    hardware_bridge.reset_takeover_detection()
            elif not built_in_autopilot_active and prev_built_in_autopilot_active:
                built_in_autopilot_engaged_s = None
            if built_in_autopilot_active:
                if world.autonomy_controller is not None:
                    world.autonomy_controller = None
                    hud.set_autonomy_controller(None)
                if (
                    world.current_route
                    or world.last_destination is not None
                    or bool(world.clicked_points)
                ):
                    input_control.clear_user_route_state()

            if world.autonomy_controller is not None:
                drive_mode = "AUTO (CUSTOM)"
            elif built_in_autopilot_active:
                drive_mode = "AUTO (CARLA)"
            else:
                drive_mode = "MANUAL"

            keys_now = pygame.key.get_pressed()
            keyboard_takeover_requested = any(bool(keys_now[key]) for key in MANUAL_TAKEOVER_KEYS)
            if keyboard_takeover_requested and (
                world.autonomy_controller is not None or built_in_autopilot_active
            ):
                world.hide_route_debug_until_autonomy_reactivated()
                if world.autonomy_controller is not None:
                    world.autonomy_controller = None
                    hud.set_autonomy_controller(None)
                if built_in_autopilot_active:
                    input_control.disable_builtin_autopilot()
                    built_in_autopilot_active = False
                    built_in_autopilot_engaged_s = None
                pedal_takeover_start_s = None
                drive_mode = "MANUAL"
                logging.info("Manual takeover source=keyboard: switching autonomy off.")

            autonomy_active = (world.autonomy_controller is not None) or built_in_autopilot_active
            if autonomy_active:
                pedal_throttle_cmd, pedal_brake_cmd = _get_safe_pedal_commands()
                pedal_takeover_cmd = max(float(pedal_throttle_cmd), float(pedal_brake_cmd))
                if pedal_takeover_cmd >= PEDAL_TAKEOVER_MIN_COMMAND:
                    if pedal_takeover_start_s is None:
                        pedal_takeover_start_s = now_s
                    elif (now_s - pedal_takeover_start_s) >= PEDAL_TAKEOVER_HOLD_S:
                        world.hide_route_debug_until_autonomy_reactivated()
                        if world.autonomy_controller is not None:
                            world.autonomy_controller = None
                            hud.set_autonomy_controller(None)
                        if built_in_autopilot_active:
                            input_control.disable_builtin_autopilot()
                            built_in_autopilot_active = False
                            built_in_autopilot_engaged_s = None
                        pedal_takeover_start_s = None
                        drive_mode = "MANUAL"
                        logging.info(
                            "Manual takeover source=pedal (throttle=%.2f brake=%.2f): switching autonomy off.",
                            pedal_throttle_cmd,
                            pedal_brake_cmd,
                        )
                else:
                    pedal_takeover_start_s = None
            else:
                pedal_takeover_start_s = None

            drivetrain_mode = "DRIVE"
            if world.hero_actor is not None:
                try:
                    current_control = world.hero_actor.get_control()
                    if current_control.reverse or current_control.gear < 0:
                        drivetrain_mode = "REVERSE"
                except Exception:
                    drivetrain_mode = "DRIVE"
            if hardware_bridge is not None and world.hero_actor is not None:
                hardware_bridge.poll(world.hero_actor)
                if pedals_bridge is not None:
                    pedals_bridge.poll()
                if buttons_bridge is not None:
                    buttons_bridge.poll()
                joystick_vrx = getattr(buttons_bridge, "joystick_vrx", None) if buttons_bridge is not None else None
                joystick_vry = getattr(buttons_bridge, "joystick_vry", None) if buttons_bridge is not None else None
                joystick_sw_pressed = (
                    getattr(buttons_bridge, "joystick_sw_pressed", None) if buttons_bridge is not None else None
                )
                joystick_sw_down = bool(joystick_sw_pressed) if joystick_sw_pressed is not None else False
                if world.show_navigation:
                    world.update_nav_crosshair_from_joystick(
                        joystick_vrx, joystick_vry, clock.get_time() / 1000.0
                    )
                    if joystick_sw_down and not joystick_sw_was_pressed:
                        cross_x, cross_y = world.get_nav_crosshair_screen_pos()
                        if input_control.set_navigation_destination_screen(cross_x, cross_y):
                            logging.info(
                                "Joystick destination set at crosshair (%d, %d).",
                                cross_x,
                                cross_y,
                            )
                joystick_sw_was_pressed = joystick_sw_down
                rezero_now = bool(pygame.key.get_pressed()[REZERO_KEY])
                if rezero_now and not rezero_key_was_down:
                    if hardware_bridge.rezero_encoder():
                        hud.notification(f"Wheel recentered ({REZERO_KEY_LABEL})")
                        logging.info(
                            "Wheel encoder re-zeroed from keyboard (%s).",
                            REZERO_KEY_LABEL,
                        )
                    else:
                        hud.notification("Wheel recenter unavailable")
                rezero_key_was_down = rezero_now
                button_bridge = buttons_bridge if buttons_bridge is not None else hardware_bridge
                if button_bridge is not None and button_bridge.consume_toggle_nav_map_request():
                    world.show_navigation = not world.show_navigation
                    world._sync_camera_surface_conversion()
                    if world.show_navigation:
                        if hasattr(world, "center_map_on_map_center"):
                            world.center_map_on_map_center()
                        if hasattr(world, "reset_nav_crosshair"):
                            world.reset_nav_crosshair()
                        world._recenter_nav_next_tick = True
                    hud.notification(
                        "Navigation ON" if world.show_navigation else "Navigation OFF"
                    )
                    logging.info(
                        "Hardware nav toggle (A2): %s",
                        "ON" if world.show_navigation else "OFF",
                    )
                if button_bridge is not None and button_bridge.consume_toggle_reverse_request():
                    reverse_on = input_control.toggle_drive_reverse()
                    logging.info(
                        "Hardware gear toggle (A3): %s",
                        "REVERSE" if reverse_on else "DRIVE",
                    )
                if button_bridge is not None and button_bridge.consume_engage_autonomy_request():
                    if world.autonomy_controller is None:
                        engaged = input_control.engage_autonomy_from_last_destination()
                        if engaged:
                            logging.info("Hardware autonomy engage (A0): ON")
                    else:
                        logging.info("Hardware autonomy engage (A0): ignored (already AUTO)")
                if button_bridge is not None and button_bridge.consume_respawn_request():
                    respawned = world.respawn_hero_random()
                    input_control.on_hero_respawned()
                    hud.set_autonomy_controller(None)
                    if respawned:
                        hud.notification("Vehicle respawned")
                        logging.info("Hardware respawn (A5): hero respawned.")
                    else:
                        logging.warning("Hardware respawn (A5) failed.")
                if (
                    (
                        world.autonomy_controller is not None
                        or (
                            built_in_autopilot_active
                            and built_in_autopilot_engaged_s is not None
                            and (now_s - built_in_autopilot_engaged_s) >= CARLA_AUTOPILOT_TAKEOVER_GRACE_S
                        )
                    )
                    and hardware_bridge.detect_manual_takeover(world.hero_actor)
                ):
                    world.hide_route_debug_until_autonomy_reactivated()
                    if world.autonomy_controller is not None:
                        world.autonomy_controller = None
                        hud.set_autonomy_controller(None)
                    if built_in_autopilot_active:
                        input_control.disable_builtin_autopilot()
                        built_in_autopilot_active = False
                        built_in_autopilot_engaged_s = None
                    if world.autonomy_controller is None and not built_in_autopilot_active:
                        drive_mode = "MANUAL"
                    logging.info("Manual takeover source=wheel: switching autonomy off.")
                route_index = None
                if world.autonomy_controller is not None:
                    route_index = getattr(world.autonomy_controller, "current_wp_index", None)
                hardware_bridge.update_auto_blinker(
                    vehicle=world.hero_actor,
                    route=world.current_route,
                    autonomy_active=(world.autonomy_controller is not None),
                    route_index=route_index,
                )
                safe_throttle, safe_brake = _get_safe_pedal_ticks()
                hud.set_drive_status(
                    mode=drive_mode,
                    drivetrain=drivetrain_mode,
                    ticks=hardware_bridge.encoder_counts,
                    left_blinker=hardware_bridge.left_blinker_on,
                    right_blinker=hardware_bridge.right_blinker_on,
                    pedal_throttle=safe_throttle,
                    pedal_brake=safe_brake,
                    pedal_throttle_max=getattr(pedals_bridge, "throttle_range_ticks", None),
                    pedal_brake_max=getattr(pedals_bridge, "brake_range_ticks", None),
                    nav_mode_active=world.show_navigation,
                    joystick_vrx=joystick_vrx,
                    joystick_vry=joystick_vry,
                    joystick_sw_pressed=joystick_sw_pressed,
                )
            else:
                if pedals_bridge is not None:
                    pedals_bridge.poll()
                rezero_key_was_down = bool(pygame.key.get_pressed()[REZERO_KEY])
                safe_throttle, safe_brake = _get_safe_pedal_ticks()
                hud.set_drive_status(
                    mode=drive_mode,
                    drivetrain=drivetrain_mode,
                    ticks=None,
                    left_blinker=False,
                    right_blinker=False,
                    pedal_throttle=safe_throttle,
                    pedal_brake=safe_brake,
                    pedal_throttle_max=getattr(pedals_bridge, "throttle_range_ticks", None),
                    pedal_brake_max=getattr(pedals_bridge, "brake_range_ticks", None),
                    nav_mode_active=world.show_navigation,
                    joystick_vrx=None,
                    joystick_vry=None,
                    joystick_sw_pressed=None,
                )
            prev_built_in_autopilot_active = built_in_autopilot_active
            hud.tick(clock)
            input_control.tick(clock)
            if hardware_bridge is not None and world.hero_actor is not None:
                hardware_bridge.apply_force_feedback(
                    world.hero_actor,
                    auto_mode=(world.autonomy_controller is not None or built_in_autopilot_active),
                )

            display.fill(COLOR_ALUMINIUM_4)
            world.render(display)
            hud.render(display)
            input_control.render(display)
            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    finally:
        if buttons_bridge is not None:
            buttons_bridge.close()
        if pedals_bridge is not None:
            pedals_bridge.close()
        if hardware_bridge is not None:
            hardware_bridge.close()
        if world is not None:
            world.destroy()
        pygame.quit()
        sys.exit()

def main():
    argparser = argparse.ArgumentParser(
        description="CARLA Phase 1 control (refactored)"
    )
    argparser.add_argument("--host", default="127.0.0.1")
    argparser.add_argument("--port", default=2000, type=int)
    argparser.add_argument("--width", default=1920, type=int)
    argparser.add_argument("--height", default=1080, type=int)
    argparser.add_argument("--map", default="Town10HD_Opt")
    # argparser.add_argument("--map", default="Town01") #small town
    # argparser.add_argument("--map", default="Town02") #small town
    # argparser.add_argument("--map", default="Town03") #big city
    # argparser.add_argument("--map", default="Town05") #big city

    argparser.add_argument("--timeout", default=30.0, type=float)

    argparser.add_argument("--show-triggers", action="store_true", default=False)
    argparser.add_argument("--show-connections", action="store_true", default=False)
    argparser.add_argument("--show-spawn-points", action="store_true", default=False)
    argparser.add_argument("--tl-red", type=float, default=5.0, help="Traffic-light red duration (seconds)")
    argparser.add_argument("--tl-yellow", type=float, default=2.0, help="Traffic-light yellow duration (seconds)")
    argparser.add_argument("--tl-green", type=float, default=5.0, help="Traffic-light green duration (seconds)")

    argparser.add_argument("-v", "--verbose", action="store_true")

    args = argparser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    game_loop(args)


if __name__ == "__main__":
    main()




















