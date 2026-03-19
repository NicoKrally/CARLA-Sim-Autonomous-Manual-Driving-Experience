import pygame



MAX_TICKS = 400
DEADZONE = 0.02
STEER_SENSITIVITY = 0.12
THROTTLE_STEP = 0.05
CENTER_KP = 0.28
CENTER_KD = 0.035
CENTER_MAX_PWM = 230
CENTER_DEADBAND_TICKS = 30
USE_CONSTANT_THROTTLE = True
CONSTANT_THROTTLE = 0.5


AUTO_TOGGLE_KEY = pygame.K_a
REVERSE_TOGGLE_KEY = pygame.K_q


def create_manual_state():
    return {
        "throttle_value": 0.0,
        "brake_value": 0.0,
        "reverse_enabled": False,
    }


def to_manual_steer(encoder_counts):
    steer_value = (encoder_counts / MAX_TICKS) * STEER_SENSITIVITY
    steer_value = max(-1.0, min(1.0, steer_value))
    if abs(steer_value) < DEADZONE:
        steer_value = 0.0
    return steer_value


def handle_manual_event(event, state):
    """Returns True if mode should switch to AUTO."""
    if event.type == pygame.KEYUP and event.key == REVERSE_TOGGLE_KEY:
        state["reverse_enabled"] = not state["reverse_enabled"]
    return event.type == pygame.KEYUP and event.key == AUTO_TOGGLE_KEY


def run_manual_step(*, state, control, vehicle, encoder_counts, vel_ticks, ser):
    steer_value = to_manual_steer(encoder_counts)

    keys = pygame.key.get_pressed()
    if USE_CONSTANT_THROTTLE:
        state["throttle_value"] = CONSTANT_THROTTLE
    else:
        if keys[pygame.K_w]:
            state["throttle_value"] = min(state["throttle_value"] + THROTTLE_STEP, 1.0)
        else:
            state["throttle_value"] = 0.0

    if keys[pygame.K_s]:
        state["brake_value"] = min(state["brake_value"] + 0.2, 1.0)
    else:
        state["brake_value"] = 0.0

    control.steer = steer_value
    control.throttle = state["throttle_value"]
    control.brake = state["brake_value"]
    control.reverse = state["reverse_enabled"]
    control.gear = -1 if state["reverse_enabled"] else 1
    vehicle.apply_control(control)

    try:
        physics = vehicle.get_physics_control()
        max_steer = max(w.max_steer_angle for w in physics.wheels)
        steer_angles = [vehicle.get_wheel_steer_angle(i) for i in range(len(physics.wheels))]
        front_angles = steer_angles[:2] if len(steer_angles) >= 2 else steer_angles
        current_steer = sum(front_angles) / max(1, len(front_angles))
        target_steer = max(-1.0, min(1.0, current_steer / max_steer)) if max_steer else 0.0
    except Exception:
        target_steer = control.steer

    target_ticks = target_steer * MAX_TICKS
    error_ticks = encoder_counts - target_ticks
    if abs(error_ticks) <= CENTER_DEADBAND_TICKS:
        center_pwm = 0
    else:
        center_pwm = int(-CENTER_KP * error_ticks - CENTER_KD * vel_ticks)
        center_pwm = max(-CENTER_MAX_PWM, min(CENTER_MAX_PWM, center_pwm))
    ser.write(f"{center_pwm}\n".encode())

    return steer_value


def main():

    import os
    import runpy

    python_scripts_dir = os.path.dirname(os.path.abspath(__file__))
    main_path = os.path.join(os.path.dirname(python_scripts_dir), "Main", "main.py")
    runpy.run_path(main_path, run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
