import logging
import math
import time
from typing import Callable, List, Optional

import carla
import numpy as np


class SimpleRouteFollower:
    def __init__(
        self,
        vehicle,
        route,
        target_speed_kmh=30.0,
        lookahead_distance=8.0,
        steer_gain=1.0,
        throttle_low=0.2,
        throttle_high=0.5,
        adaptive_lookahead=True,
        lookahead_speed_gain=0.25,
        lookahead_min=2.0,
        lookahead_max=12.0,
        wheelbase_override=None,
        tl_state_provider: Optional[Callable[[], str]] = None,
        reroute_callback: Optional[Callable[[carla.Location], Optional[list]]] = None,
    ):
        self.vehicle = vehicle
        self.route = route
        self.lookahead_distance = lookahead_distance
        self.target_speed_kmh = target_speed_kmh
        self.target_speed = target_speed_kmh / 3.6
        self.steer_gain = steer_gain
        self.throttle_low = throttle_low
        self.throttle_high = throttle_high
        self.adaptive_lookahead = adaptive_lookahead
        self.lookahead_speed_gain = lookahead_speed_gain
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max

        self.current_wp_index = 0
        self.finished = False
        self._wheelbase_override = wheelbase_override
        self._wheelbase = (
            float(wheelbase_override)
            if wheelbase_override is not None
            else self._compute_wheelbase_m()
        )
        self._max_steer_rad = self._get_max_steer_rad()
        self._carla_map = self.vehicle.get_world().get_map()
        self._tl_state_provider = tl_state_provider
        self._reroute_callback = reroute_callback
        self._route_destination = self._extract_route_destination(route)

        self._turnaround_active = False
        self._turnaround_frames = 0
        self._behind_frames = 0
        self._divergence_time_s = 0.0
        self._last_route_error_m = None
        self._last_reroute_time_s = -1e9

        self.stop_lookahead_m = 100.0
        self.stop_step_m = 1.5
        self.stop_buffer_m = 0.0
        self.stop_hold_distance_m = 1.0
        self.stop_hold_speed_mps = 0.2
        self.stop_brake_hold = 0.85

        self.turnaround_enter_heading_deg = 115.0
        self.turnaround_exit_heading_deg = 45.0
        self.turnaround_trigger_frames = 8
        self.turnaround_min_local_x_m = 0.5
        self.turnaround_steer = 0.9
        self.turnaround_throttle = 0.22
        self.turnaround_speed_cap_mps = 4.0

        self.waypoint_heading_penalty_m = 8.0
        self.waypoint_search_back = 35
        self.waypoint_search_ahead = 120
        self.max_backtrack_when_stable = 5

        self.divergence_enter_error_m = 6.0
        self.divergence_growth_epsilon_m = 0.25
        self.divergence_trigger_s = 1.0
        self.reroute_cooldown_s = 4.0

        self.latest_bumper_goal = None
        self.latest_actor_goal = None
        self.latest_bumper_now = None

    def tick(self, dt):
        if self.finished or not self.route:
            return

        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        speed = self._get_speed_mps()

        self.current_wp_index, closest_route_error_m = self._find_closest_waypoint(vehicle_transform)
        self._update_divergence_state(dt, closest_route_error_m)
        if self._maybe_refresh_route():
            vehicle_transform = self.vehicle.get_transform()
            vehicle_location = vehicle_transform.location
            speed = self._get_speed_mps()
            self.current_wp_index, _ = self._find_closest_waypoint(vehicle_transform)

        target_wp = self._get_lookahead_waypoint(vehicle_location)

        if target_wp is None:
            self.finished = True
            self._apply_brake()
            return

        route_forward = self._route_forward_vector(self.current_wp_index)
        local_x, _local_y = self._to_vehicle_local_xy(vehicle_transform, target_wp.transform.location)
        heading_error_deg = self._heading_error_deg(vehicle_transform, route_forward)
        self._update_turnaround_state(local_x, heading_error_deg)

        if self._turnaround_active:
            steer, throttle, brake = self._compute_turnaround_command(
                vehicle_transform=vehicle_transform,
                target_location=target_wp.transform.location,
                route_forward=route_forward,
                speed=speed,
            )
        else:
            should_stop_for_red = self._is_red_light()
            stop_solution = self._compute_stop_targets() if should_stop_for_red else None

            if stop_solution is not None:
                bumper_goal, actor_goal, _f_stop, s_actor_goal, path_points, path_s = stop_solution
                self.latest_bumper_goal = bumper_goal
                self.latest_actor_goal = actor_goal
                self.latest_bumper_now = self._compute_front_bumper_point(vehicle_transform)

                steer = self._compute_steering(vehicle_transform, actor_goal)
                d_along = self._distance_along_path_to_goal(path_points, path_s, s_actor_goal, vehicle_location)
                throttle, brake = self._compute_stop_longitudinal_command(d_along=d_along, speed=speed)
            else:
                self.latest_bumper_goal = None
                self.latest_actor_goal = None
                self.latest_bumper_now = None
                steer = self._compute_steering(vehicle_transform, target_wp.transform.location)
                throttle = self._compute_throttle()
                brake = 0.0

        control = carla.VehicleControl()
        control.throttle = throttle
        control.steer = steer
        control.brake = brake
        control.hand_brake = False
        control.reverse = False
        self.vehicle.apply_control(control)

    def _find_closest_waypoint(self, vehicle_transform):
        if not self.route:
            return 0, float("inf")

        location = vehicle_transform.location
        route_len = len(self.route)
        current_idx = int(np.clip(self.current_wp_index, 0, route_len - 1))
        search_start = max(0, current_idx - self.waypoint_search_back)
        search_end = min(route_len - 1, current_idx + self.waypoint_search_ahead)
        if self._turnaround_active:
            search_start = 0
            search_end = route_len - 1

        best_idx = current_idx
        best_dist = float("inf")
        best_score = float("inf")

        for i in range(search_start, search_end + 1):
            wp = self.route[i][0]
            dist = location.distance(wp.transform.location)

            segment_forward = self._route_forward_vector(i)
            heading_penalty = 0.0
            if segment_forward is not None:
                heading_penalty = self.waypoint_heading_penalty_m * (
                    self._heading_error_deg(vehicle_transform, segment_forward) / 180.0
                )

            backtrack_penalty = 0.0
            if (not self._turnaround_active) and i < current_idx:
                backtrack = current_idx - i
                if backtrack > self.max_backtrack_when_stable:
                    backtrack_penalty = 3.0 + (backtrack - self.max_backtrack_when_stable) * 0.3

            score = dist + heading_penalty + backtrack_penalty
            if score < best_score:
                best_score = score
                best_dist = dist
                best_idx = i

        return best_idx, best_dist

    def _get_lookahead_waypoint(self, location):
        lookahead = self._get_lookahead_distance()
        total_dist = 0.0
        for i in range(self.current_wp_index, len(self.route) - 1):
            wp1 = self.route[i][0].transform.location
            wp2 = self.route[i + 1][0].transform.location
            total_dist += wp1.distance(wp2)
            if total_dist >= lookahead:
                return self.route[i + 1][0]
        return None

    def _compute_steering(self, vehicle_transform, target_location):
        local_x, local_y = self._to_vehicle_local_xy(vehicle_transform, target_location)

        if local_x <= 0.001:
            turn_sign = 1.0 if local_y >= 0.0 else -1.0
            return float(turn_sign)

        curvature = 2.0 * local_y / (local_x**2 + local_y**2)
        desired_steer_rad = math.atan(self._wheelbase * curvature)
        steer = desired_steer_rad / self._max_steer_rad
        steer *= self.steer_gain
        steer = np.clip(steer, -1.0, 1.0)
        return float(steer)

    @staticmethod
    def _extract_route_destination(route):
        if not route:
            return None
        try:
            loc = route[-1][0].transform.location
            return carla.Location(x=float(loc.x), y=float(loc.y), z=float(loc.z))
        except Exception:
            return None

    @staticmethod
    def _to_vehicle_local_xy(vehicle_transform, target_location):
        v_loc = vehicle_transform.location
        v_yaw = math.radians(vehicle_transform.rotation.yaw)

        dx = target_location.x - v_loc.x
        dy = target_location.y - v_loc.y
        local_x = math.cos(-v_yaw) * dx - math.sin(-v_yaw) * dy
        local_y = math.sin(-v_yaw) * dx + math.cos(-v_yaw) * dy
        return local_x, local_y

    @staticmethod
    def _angle_between_vectors_deg(vec_a, vec_b):
        if vec_a is None or vec_b is None:
            return 0.0
        dot = (vec_a.x * vec_b.x) + (vec_a.y * vec_b.y) + (vec_a.z * vec_b.z)
        norm_a = math.sqrt((vec_a.x * vec_a.x) + (vec_a.y * vec_a.y) + (vec_a.z * vec_a.z))
        norm_b = math.sqrt((vec_b.x * vec_b.x) + (vec_b.y * vec_b.y) + (vec_b.z * vec_b.z))
        if norm_a <= 1e-6 or norm_b <= 1e-6:
            return 0.0
        cos_theta = float(np.clip(dot / (norm_a * norm_b), -1.0, 1.0))
        return math.degrees(math.acos(cos_theta))

    def _route_forward_vector(self, index):
        if not self.route or len(self.route) < 2:
            return None

        idx = int(np.clip(index, 0, len(self.route) - 1))
        if idx < (len(self.route) - 1):
            p0 = self.route[idx][0].transform.location
            p1 = self.route[idx + 1][0].transform.location
        elif idx > 0:
            p0 = self.route[idx - 1][0].transform.location
            p1 = self.route[idx][0].transform.location
        else:
            return None

        dx = p1.x - p0.x
        dy = p1.y - p0.y
        dz = p1.z - p0.z
        norm = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        if norm <= 1e-6:
            return None
        return carla.Vector3D(dx / norm, dy / norm, dz / norm)

    def _heading_error_deg(self, vehicle_transform, route_forward):
        if route_forward is None:
            return 0.0
        vehicle_forward = vehicle_transform.get_forward_vector()
        return self._angle_between_vectors_deg(vehicle_forward, route_forward)

    def _signed_heading_error_deg(self, vehicle_transform, route_forward):
        if route_forward is None:
            return 0.0
        vehicle_forward = vehicle_transform.get_forward_vector()
        angle_deg = self._angle_between_vectors_deg(vehicle_forward, route_forward)
        cross_z = (vehicle_forward.x * route_forward.y) - (vehicle_forward.y * route_forward.x)
        return angle_deg if cross_z >= 0.0 else -angle_deg

    def _update_turnaround_state(self, local_x, heading_error_deg):
        should_request_turnaround = (
            local_x <= self.turnaround_min_local_x_m
            and heading_error_deg >= self.turnaround_enter_heading_deg
        )

        if not self._turnaround_active:
            if should_request_turnaround:
                self._behind_frames += 1
            else:
                self._behind_frames = max(0, self._behind_frames - 1)
            if self._behind_frames >= self.turnaround_trigger_frames:
                self._turnaround_active = True
                self._turnaround_frames = 0
        else:
            self._turnaround_frames += 1
            aligned = (
                heading_error_deg <= self.turnaround_exit_heading_deg
                and local_x > (self.turnaround_min_local_x_m + 0.5)
            )
            if self._turnaround_frames >= self.turnaround_trigger_frames and aligned:
                self._turnaround_active = False
                self._turnaround_frames = 0
                self._behind_frames = 0

    def _compute_turnaround_command(self, vehicle_transform, target_location, route_forward, speed):
        local_x, local_y = self._to_vehicle_local_xy(vehicle_transform, target_location)
        if abs(local_y) > 0.15:
            turn_sign = 1.0 if local_y > 0.0 else -1.0
        else:
            signed_heading = self._signed_heading_error_deg(vehicle_transform, route_forward)
            turn_sign = 1.0 if signed_heading >= 0.0 else -1.0

        steer = float(np.clip(turn_sign * self.turnaround_steer, -1.0, 1.0))
        brake = float(np.clip((speed - self.turnaround_speed_cap_mps) / 2.0, 0.0, 1.0))

        if speed < self.turnaround_speed_cap_mps and local_x < 3.0 and brake < 0.1:
            throttle = self.turnaround_throttle
        else:
            throttle = 0.0

        self.latest_bumper_goal = None
        self.latest_actor_goal = None
        self.latest_bumper_now = None
        return steer, float(np.clip(throttle, 0.0, 1.0)), brake

    def _update_divergence_state(self, dt, route_error_m):
        if route_error_m is None or not math.isfinite(route_error_m):
            self._divergence_time_s = 0.0
            self._last_route_error_m = None
            return

        route_error_m = float(route_error_m)
        dt_s = max(0.0, float(dt) if dt is not None else 0.0)

        if route_error_m < self.divergence_enter_error_m:
            self._divergence_time_s = 0.0
            self._last_route_error_m = route_error_m
            return

        if self._last_route_error_m is None:
            self._last_route_error_m = route_error_m
            self._divergence_time_s = 0.0
            return

        if route_error_m > (self._last_route_error_m + self.divergence_growth_epsilon_m):
            self._divergence_time_s += dt_s
        else:
            self._divergence_time_s = max(0.0, self._divergence_time_s - (dt_s * 0.5))

        self._last_route_error_m = route_error_m

    def _maybe_refresh_route(self):
        if self._reroute_callback is None or self._route_destination is None:
            return False
        if self._divergence_time_s < self.divergence_trigger_s:
            return False

        now_s = time.monotonic()
        if (now_s - self._last_reroute_time_s) < self.reroute_cooldown_s:
            return False

        self._last_reroute_time_s = now_s
        self._divergence_time_s = 0.0
        self._last_route_error_m = None

        try:
            refreshed_route = self._reroute_callback(self._route_destination)
        except Exception as exc:
            logging.debug("Route refresh callback failed: %s", exc)
            return False

        if not refreshed_route:
            return False

        self.route = refreshed_route
        self.current_wp_index = 0
        destination = self._extract_route_destination(refreshed_route)
        if destination is not None:
            self._route_destination = destination

        self._turnaround_active = False
        self._turnaround_frames = 0
        self._behind_frames = 0
        logging.info("Custom autonomy route refreshed after divergence.")
        return True

    def _compute_throttle(self):
        speed = self._get_speed_mps()
        if speed < self.target_speed:
            return self.throttle_high
        return self.throttle_low

    def _apply_brake(self):
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = 1.0
        self.vehicle.apply_control(control)

    def set_wheelbase_override(self, override):
        self._wheelbase_override = override
        if override is None:
            self._wheelbase = self._compute_wheelbase_m()
        else:
            self._wheelbase = float(override)

    def _get_lookahead_distance(self):
        if not self.adaptive_lookahead:
            return self.lookahead_distance
        speed = self._get_speed_mps()
        dynamic = self.lookahead_distance + self.lookahead_speed_gain * speed
        return float(np.clip(dynamic, self.lookahead_min, self.lookahead_max))

    def _get_speed_mps(self):
        vel = self.vehicle.get_velocity()
        return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

    def _is_red_light(self):
        if self._tl_state_provider is None:
            return False
        try:
            return str(self._tl_state_provider()).upper() == "RED"
        except Exception:
            return False

    def _compute_stop_targets(self):
        vehicle_transform = self.vehicle.get_transform()
        p_ego = vehicle_transform.location

        path_points, path_s, path_wps = self._build_lane_polyline_ahead(
            p_ego=p_ego,
            lookahead_m=self.stop_lookahead_m,
            step_m=self.stop_step_m,
        )
        if len(path_points) < 2:
            return None

        junction_idx = None
        for i, wp in enumerate(path_wps):
            if wp.is_junction:
                junction_idx = i
                break
        if junction_idx is None:
            return None

        s_entry = path_s[junction_idx]
        s_stop = s_entry - self.stop_buffer_m
        if s_stop <= 0.0:
            return None

        bumper_goal, seg_idx, _seg_alpha = self._interp_polyline(path_points, path_s, s_stop)
        if bumper_goal is None:
            return None

        f_stop = self._segment_forward(path_points, seg_idx)
        if f_stop is None:
            return None

        l_half = self._front_bumper_offset_m()
        actor_goal = bumper_goal - (f_stop * l_half)
        s_actor_goal = max(0.0, s_stop - l_half)
        return bumper_goal, actor_goal, f_stop, s_actor_goal, path_points, path_s

    def _build_lane_polyline_ahead(self, p_ego, lookahead_m, step_m):
        wp = self._carla_map.get_waypoint(
            p_ego,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if wp is None:
            return [], [], []

        points = [wp.transform.location]
        s_vals = [0.0]
        wps = [wp]

        traveled = 0.0
        current_wp = wp
        while traveled < lookahead_m:
            nxt = current_wp.next(step_m)
            if not nxt:
                break
            next_wp = nxt[0]
            next_loc = next_wp.transform.location
            seg_len = points[-1].distance(next_loc)
            if seg_len <= 1e-3:
                break
            traveled += seg_len
            points.append(next_loc)
            s_vals.append(traveled)
            wps.append(next_wp)
            current_wp = next_wp

        return points, s_vals, wps

    def _interp_polyline(self, points, s_vals, s_target):
        if not points or len(points) != len(s_vals):
            return None, 0, 0.0

        s_target = float(np.clip(s_target, s_vals[0], s_vals[-1]))
        for i in range(len(s_vals) - 1):
            s0 = s_vals[i]
            s1 = s_vals[i + 1]
            if s0 <= s_target <= s1:
                ds = max(1e-6, s1 - s0)
                alpha = (s_target - s0) / ds
                p0 = points[i]
                p1 = points[i + 1]
                p = carla.Location(
                    x=p0.x + (p1.x - p0.x) * alpha,
                    y=p0.y + (p1.y - p0.y) * alpha,
                    z=p0.z + (p1.z - p0.z) * alpha,
                )
                return p, i, alpha

        return points[-1], max(0, len(points) - 2), 1.0

    def _segment_forward(self, points, seg_idx):
        i = int(np.clip(seg_idx, 0, max(0, len(points) - 2)))
        p0 = points[i]
        p1 = points[i + 1]
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        dz = p1.z - p0.z
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm <= 1e-6:
            return None
        return carla.Vector3D(dx / norm, dy / norm, dz / norm)

    def _front_bumper_offset_m(self):
        try:
            return max(0.1, float(self.vehicle.bounding_box.extent.x))
        except Exception:
            return 1.2

    def _distance_along_path_to_goal(self, points: List[carla.Location], s_vals: List[float], s_goal: float, p_ego):
        if len(points) < 2:
            return 0.0

        best_dist_sq = float("inf")
        best_s = s_vals[0]
        for i in range(len(points) - 1):
            p0 = points[i]
            p1 = points[i + 1]
            vx = p1.x - p0.x
            vy = p1.y - p0.y
            vz = p1.z - p0.z
            wx = p_ego.x - p0.x
            wy = p_ego.y - p0.y
            wz = p_ego.z - p0.z

            vv = vx * vx + vy * vy + vz * vz
            if vv <= 1e-6:
                t = 0.0
            else:
                t = (wx * vx + wy * vy + wz * vz) / vv
                t = float(np.clip(t, 0.0, 1.0))

            proj_x = p0.x + t * vx
            proj_y = p0.y + t * vy
            proj_z = p0.z + t * vz
            dx = p_ego.x - proj_x
            dy = p_ego.y - proj_y
            dz = p_ego.z - proj_z
            dist_sq = dx * dx + dy * dy + dz * dz
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_s = s_vals[i] + (s_vals[i + 1] - s_vals[i]) * t

        return s_goal - best_s

    def _compute_stop_longitudinal_command(self, d_along, speed):
        if d_along < self.stop_hold_distance_m and speed < self.stop_hold_speed_mps:
            return 0.0, self.stop_brake_hold

        if d_along > 10.0:
            target = self.target_speed
        elif d_along > 0.0:
            target = self.target_speed * (d_along / 10.0)
        else:
            target = 0.0

        speed_err = speed - target
        if speed_err > 0.0:
            brake = float(np.clip(speed_err / 2.0, 0.0, 1.0))
            return 0.0, brake

        throttle = self.throttle_high if speed < target else self.throttle_low
        return float(np.clip(throttle, 0.0, 1.0)), 0.0

    def _compute_front_bumper_point(self, vehicle_transform):
        loc = vehicle_transform.location
        fwd = vehicle_transform.get_forward_vector()
        l_half = self._front_bumper_offset_m()
        return loc + carla.Location(fwd.x * l_half, fwd.y * l_half, fwd.z * l_half)

    def _compute_wheelbase_m(self):
        try:
            physics = self.vehicle.get_physics_control()
            wheels = physics.wheels
            front_x = [w.position.x for w in wheels if w.position.x > 0.0]
            rear_x = [w.position.x for w in wheels if w.position.x < 0.0]
            if front_x and rear_x:
                return (sum(front_x) / len(front_x)) - (sum(rear_x) / len(rear_x))
        except Exception:
            pass

        try:
            bb = self.vehicle.bounding_box
            length = 2.0 * bb.extent.x
            return max(1.0, length * 0.8)
        except Exception:
            return 2.7

    def _get_max_steer_rad(self):
        try:
            physics = self.vehicle.get_physics_control()
            max_deg = max(w.max_steer_angle for w in physics.wheels)
            if max_deg > 0.0:
                return math.radians(max_deg)
        except Exception:
            pass
        return math.radians(70.0)
