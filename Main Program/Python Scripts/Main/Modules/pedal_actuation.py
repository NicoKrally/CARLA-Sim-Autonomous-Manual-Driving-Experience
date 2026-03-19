import carla


def clamp(x, lo, hi):
    x = float(x)
    return lo if x < lo else hi if x > hi else x


def clamp01(x):
    return clamp(x, 0.0, 1.0)


def dominant_pedal(throttle, brake):
    throttle = clamp01(throttle)
    brake = clamp01(brake)
    if throttle >= brake:
        return throttle, 0.0
    return 0.0, brake


def apply_pedals_to_carla(vehicle, throttle, brake, steer=0.0):
    control = vehicle.get_control()
    control.throttle = clamp01(throttle)
    control.brake = clamp01(brake)
    control.steer = clamp(steer, -1.0, 1.0)
    vehicle.apply_control(control)

