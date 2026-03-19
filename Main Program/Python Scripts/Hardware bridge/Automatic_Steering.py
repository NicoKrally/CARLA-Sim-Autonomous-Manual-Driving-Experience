import random
import time

import numpy as np



AUTO_STEER_SENSITIVITY = 0.1
TARGET_RATE_LIMIT_COUNTS = 10
ERROR_ENTER_DEADBAND = 22
ERROR_EXIT_DEADBAND = 8
KP = 140.0
PWM_DEADBAND = 2


TAKEOVER_ERROR_COUNTS = 30
TAKEOVER_HOLD_SECONDS = 0.1
TAKEOVER_MIN_VEL_TICKS_PER_S = 140
TAKEOVER_MIN_PWM = 20

def create_auto_state(initial_encoder_counts=0.0):
    return {
        "target_counts_smoothed": float(initial_encoder_counts),
        "correction_active": False,
        "takeover_start_time": None,
    }


def reset_auto_state(state, initial_encoder_counts):
    state["target_counts_smoothed"] = float(initial_encoder_counts)
    state["correction_active"] = False
    state["takeover_start_time"] = None


def wants_manual_from_key(event):
    return False


def run_auto_step(
    *,
    state,
    control,
    agent,
    vehicle,
    spawn_points,
    encoder_counts,
    vel_ticks,
    max_ticks,
    ser,
):
    """Run one AUTO control step. Returns True if we should switch to MANUAL."""
    control = agent.run_step()
    control.manual_gear_shift = False
    vehicle.apply_control(control)

    if agent.done():
        agent.set_destination(random.choice(spawn_points).location)

    target = max(-AUTO_STEER_SENSITIVITY, min(AUTO_STEER_SENSITIVITY, control.steer))
    target_counts = (target / AUTO_STEER_SENSITIVITY) * max_ticks

    delta_target = target_counts - state["target_counts_smoothed"]
    if abs(delta_target) > TARGET_RATE_LIMIT_COUNTS:
        state["target_counts_smoothed"] += TARGET_RATE_LIMIT_COUNTS * float(np.sign(delta_target))
    else:
        state["target_counts_smoothed"] = target_counts

    ideal_error_counts = target_counts - encoder_counts
    error_counts = state["target_counts_smoothed"] - encoder_counts

    pwm = 0
    if not state["correction_active"]:
        if abs(error_counts) >= ERROR_ENTER_DEADBAND:
            state["correction_active"] = True
    if state["correction_active"]:
        if abs(error_counts) <= ERROR_EXIT_DEADBAND:
            state["correction_active"] = False
            pwm = 0
        else:
            pwm = int(KP * (error_counts / max_ticks))
            pwm = max(-255, min(255, pwm))
            if abs(pwm) < PWM_DEADBAND:
                pwm = 0

    diverging_from_controller = (
        abs(pwm) >= TAKEOVER_MIN_PWM
        and abs(vel_ticks) >= TAKEOVER_MIN_VEL_TICKS_PER_S
        and np.sign(vel_ticks) == -np.sign(pwm)
    )

    if abs(ideal_error_counts) >= TAKEOVER_ERROR_COUNTS and diverging_from_controller:
        if state["takeover_start_time"] is None:
            state["takeover_start_time"] = time.monotonic()
        elif (time.monotonic() - state["takeover_start_time"]) >= TAKEOVER_HOLD_SECONDS:
            state["takeover_start_time"] = None
            state["correction_active"] = False
            ser.write(b"0\n")
            return True
    else:
        state["takeover_start_time"] = None

    ser.write(f"{pwm}\n".encode())
    return False


def main():

    import os
    import runpy

    python_scripts_dir = os.path.dirname(os.path.abspath(__file__))
    main_path = os.path.join(os.path.dirname(python_scripts_dir), "Main", "main.py")
    runpy.run_path(main_path, run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
