
import os
import glob
import time
import math
import random
import logging
import datetime
import weakref
import hashlib
import sys

import pygame
import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from Modules.detections import (
    TLClusterConfig,
    draw_cluster_debug_overlay_pygame,
    select_traffic_light_cluster_single_frame,
)
from Modules.yolo_perception import YoloPerception
from Modules.sensing import TrafficLightSurfaces, ThirdPersonCamera, RGBCamera
from Modules.constants import (
    PIXELS_PER_METER,
    MAP_DEFAULT_SCALE,
    COLOR_BLACK,
    COLOR_ALUMINIUM_4,
    COLOR_ALUMINIUM_5,
    COLOR_ALUMINIUM_4_5,
    COLOR_ALUMINIUM_1,
    COLOR_ALUMINIUM_3,
    COLOR_ALUMINIUM_2,
    COLOR_BUTTER_0,
    COLOR_ORANGE_0,
    COLOR_SCARLET_RED_0,
    COLOR_SCARLET_RED_1,
    COLOR_ORANGE_1,
    COLOR_CHAMELEON_0,
    COLOR_SKY_BLUE_1,
    COLOR_CHOCOLATE_0,
    COLOR_PLUM_0,
    COLOR_WHITE,
)

tls = carla.TrafficLightState


MEDIA_CONTENT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "Main supplementary")
AERIAL_MAP_DIR = os.path.join(MEDIA_CONTENT_DIR, "Aerial map imgs")

SAT_OVERLAY_PATHS = {
    "Town10HD_Opt": os.path.join(AERIAL_MAP_DIR, "town10aerial.png"),
    "Town10HD": os.path.join(AERIAL_MAP_DIR, "town10aerial.png"),
}


SAT_OVERLAY_TUNING = {
    "Town10HD_Opt": {"alpha": 255, "scale": 0.82, "rot_deg": 0.0, "dx": 15, "dy": -69},
    "Town10HD": {"alpha": 255, "scale": 0.82, "rot_deg": 0.0, "dx": 15, "dy": -69},
}

YOLO_MODEL_CANDIDATES = [
    os.path.join(MEDIA_CONTENT_DIR, "my_yolo_model.pt"),
]


def _blits(destination_surface, source_surfaces, rect=None, blend_mode=0):
    for surface in source_surfaces:
        destination_surface.blit(surface[0], surface[1], rect, blend_mode)


def _get_actor_trigger_bounding_box(actor):
    bb = actor.trigger_volume.extent
    corners = [
        carla.Location(x=-bb.x, y=-bb.y),
        carla.Location(x=bb.x, y=-bb.y),
        carla.Location(x=bb.x, y=bb.y),
        carla.Location(x=-bb.x, y=bb.y),
        carla.Location(x=-bb.x, y=-bb.y),
    ]
    corners = [x + actor.trigger_volume.location for x in corners]
    t = actor.get_transform()
    t.transform(corners)
    return corners

class MapImage:
    """Renders a 2D top-view map image, cached by OpenDrive hash."""

    def __init__(self, carla_world, carla_map, pixels_per_meter,
                 show_triggers, show_connections, show_spawn_points):
        self._pixels_per_meter = pixels_per_meter
        self.scale = 1.0
        self.show_triggers = show_triggers
        self.show_connections = show_connections
        self.show_spawn_points = show_spawn_points

        waypoints = carla_map.generate_waypoints(2)
        margin = 50
        max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
        max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
        min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
        min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin

        self.width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)

        width_in_pixels = (1 << 14) - 1
        surface_pixel_per_meter = int(width_in_pixels / self.width)
        if surface_pixel_per_meter > PIXELS_PER_METER:
            surface_pixel_per_meter = PIXELS_PER_METER

        self._pixels_per_meter = surface_pixel_per_meter
        width_in_pixels = int(self._pixels_per_meter * self.width)

        self.big_map_surface = pygame.Surface((width_in_pixels, width_in_pixels)).convert()

        opendrive_content = carla_map.to_opendrive()
        hash_func = hashlib.sha1()
        hash_func.update(opendrive_content.encode("UTF-8"))
        opendrive_hash = str(hash_func.hexdigest())

        filename = carla_map.name.split("/")[-1] + "_" + opendrive_hash + ".tga"
        dirname = os.path.join("cache", "no_rendering_mode")
        full_path = str(os.path.join(dirname, filename))

        if os.path.isfile(full_path):
            self.big_map_surface = pygame.image.load(full_path)
        else:
            self.draw_road_map(
                self.big_map_surface,
                carla_world,
                carla_map,
                self.world_to_pixel,
                self.world_to_pixel_width,
            )

            if not os.path.exists(dirname):
                os.makedirs(dirname)

            list_filenames = glob.glob(os.path.join(dirname, carla_map.name) + "*")
            for town_filename in list_filenames:
                os.remove(town_filename)

            pygame.image.save(self.big_map_surface, full_path)



        self._apply_satellite_overlay(carla_map)
        self.surface = self.big_map_surface

    def _apply_satellite_overlay(self, carla_map):
        """Blend an external aerial image onto the generated top-down map."""
        map_name = carla_map.name.split("/")[-1]
        overlay_path = SAT_OVERLAY_PATHS.get(map_name)
        if not overlay_path:
            return

        if not os.path.isfile(overlay_path):
            logging.warning("Satellite overlay not found: %s", overlay_path)
            return

        tuning = SAT_OVERLAY_TUNING.get(map_name, {})
        alpha = int(tuning.get("alpha", 120))
        scale = float(tuning.get("scale", 1.0))
        rot_deg = float(tuning.get("rot_deg", 0.0))
        dx = int(tuning.get("dx", 0))
        dy = int(tuning.get("dy", 0))

        try:
            sat = pygame.image.load(overlay_path).convert()
        except Exception as exc:
            logging.warning("Failed loading satellite overlay %s: %s", overlay_path, exc)
            return

        target_w, target_h = self.big_map_surface.get_size()
        sat = pygame.transform.smoothscale(sat, (target_w, target_h))

        if abs(scale - 1.0) > 1e-6:
            sat = pygame.transform.smoothscale(
                sat,
                (
                    max(1, int(sat.get_width() * scale)),
                    max(1, int(sat.get_height() * scale)),
                ),
            )

        if abs(rot_deg) > 1e-6:
            sat = pygame.transform.rotate(sat, rot_deg)

        sat.set_alpha(max(0, min(255, alpha)))


        x = (target_w - sat.get_width()) // 2 + dx
        y = (target_h - sat.get_height()) // 2 + dy

        self.big_map_surface.blit(sat, (x, y))

    def draw_road_map(self, map_surface, carla_world, carla_map, world_to_pixel, world_to_pixel_width):
        map_surface.fill(COLOR_ALUMINIUM_4)
        precision = 0.05

        def lane_marking_color_to_tango(lane_marking_color):
            tango_color = COLOR_BLACK
            if lane_marking_color == carla.LaneMarkingColor.White:
                tango_color = carla.Color(186, 189, 182)
            elif lane_marking_color == carla.LaneMarkingColor.Blue:
                tango_color = carla.Color(114, 159, 207)
            elif lane_marking_color == carla.LaneMarkingColor.Green:
                tango_color = carla.Color(138, 226, 52)
            elif lane_marking_color == carla.LaneMarkingColor.Red:
                tango_color = carla.Color(239, 41, 41)
            elif lane_marking_color == carla.LaneMarkingColor.Yellow:
                tango_color = carla.Color(252, 175, 62)
            return pygame.Color(tango_color.r, tango_color.g, tango_color.b)

        def draw_solid_line(surface, color, closed, points, width):
            if len(points) >= 2:
                pygame.draw.lines(surface, color, closed, points, width)

        def draw_broken_line(surface, color, closed, points, width):
            broken_lines = [x for n, x in enumerate(zip(*(iter(points),) * 20)) if n % 3 == 0]
            for line in broken_lines:
                pygame.draw.lines(surface, color, closed, line, width)

        def lateral_shift(transform, shift):
            transform.rotation.yaw += 90
            return transform.location + shift * transform.get_forward_vector()

        def get_lane_markings(lane_marking_type, lane_marking_color, waypoints, sign):
            margin = 0.25
            marking_1 = [world_to_pixel(lateral_shift(w.transform, sign * w.lane_width * 0.5)) for w in waypoints]
            if lane_marking_type in (carla.LaneMarkingType.Broken, carla.LaneMarkingType.Solid):
                return [(lane_marking_type, lane_marking_color, marking_1)]
            marking_2 = [world_to_pixel(lateral_shift(
                w.transform, sign * (w.lane_width * 0.5 + margin * 2)
            )) for w in waypoints]

            if lane_marking_type == carla.LaneMarkingType.SolidBroken:
                return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1),
                        (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]
            if lane_marking_type == carla.LaneMarkingType.BrokenSolid:
                return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1),
                        (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
            if lane_marking_type == carla.LaneMarkingType.BrokenBroken:
                return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1),
                        (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
            if lane_marking_type == carla.LaneMarkingType.SolidSolid:
                return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1),
                        (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]
            return [(carla.LaneMarkingType.NONE, pygame.Color(0, 0, 0), [])]

        def draw_lane(surface, lane, color):
            for side in lane:
                lane_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in side]
                lane_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in side]
                polygon = lane_left_side + [x for x in reversed(lane_right_side)]
                polygon = [world_to_pixel(x) for x in polygon]
                if len(polygon) > 2:
                    pygame.draw.polygon(surface, color, polygon, 5)
                    pygame.draw.polygon(surface, color, polygon)

        def draw_lane_marking_single_side(surface, waypoints, sign):
            marking_type = carla.LaneMarkingType.NONE
            previous_marking_type = carla.LaneMarkingType.NONE

            marking_color = carla.LaneMarkingColor.Other
            previous_marking_color = carla.LaneMarkingColor.Other

            markings_list = []
            temp_waypoints = []
            current_lane_marking = carla.LaneMarkingType.NONE

            for sample in waypoints:
                lane_marking = sample.left_lane_marking if sign < 0 else sample.right_lane_marking
                if lane_marking is None:
                    continue

                marking_type = lane_marking.type
                marking_color = lane_marking.color

                if current_lane_marking != marking_type:
                    markings = get_lane_markings(
                        previous_marking_type,
                        lane_marking_color_to_tango(previous_marking_color),
                        temp_waypoints,
                        sign,
                    )
                    current_lane_marking = marking_type
                    for marking in markings:
                        markings_list.append(marking)
                    temp_waypoints = temp_waypoints[-1:]
                else:
                    temp_waypoints.append(sample)
                    previous_marking_type = marking_type
                    previous_marking_color = marking_color

            last_markings = get_lane_markings(
                previous_marking_type,
                lane_marking_color_to_tango(previous_marking_color),
                temp_waypoints,
                sign,
            )
            for marking in last_markings:
                markings_list.append(marking)

            for markings in markings_list:
                if markings[0] == carla.LaneMarkingType.Solid:
                    draw_solid_line(surface, markings[1], False, markings[2], 2)
                elif markings[0] == carla.LaneMarkingType.Broken:
                    draw_broken_line(surface, markings[1], False, markings[2], 2)

        def draw_lane_marking(surface, waypoints):
            draw_lane_marking_single_side(surface, waypoints[0], -1)
            draw_lane_marking_single_side(surface, waypoints[1], 1)

        def draw_arrow(surface, transform, color=COLOR_ALUMINIUM_2):
            transform.rotation.yaw += 180
            forward = transform.get_forward_vector()
            transform.rotation.yaw += 90
            right_dir = transform.get_forward_vector()
            end = transform.location
            start = end - 2.0 * forward
            right = start + 0.8 * forward + 0.4 * right_dir
            left = start + 0.8 * forward - 0.4 * right_dir
            pygame.draw.lines(surface, color, False, [world_to_pixel(x) for x in [start, end]], 4)
            pygame.draw.lines(surface, color, False, [world_to_pixel(x) for x in [left, start, right]], 4)

        def draw_traffic_signs(surface, font_surface, actor, color=COLOR_ALUMINIUM_2, trigger_color=COLOR_PLUM_0):
            transform = actor.get_transform()
            waypoint = carla_map.get_waypoint(transform.location)
            angle = -waypoint.transform.rotation.yaw - 90.0
            font_surface = pygame.transform.rotate(font_surface, angle)
            pixel_pos = world_to_pixel(waypoint.transform.location)
            offset = font_surface.get_rect(center=(pixel_pos[0], pixel_pos[1]))
            surface.blit(font_surface, offset)

            forward_vector = carla.Location(waypoint.transform.get_forward_vector())
            left_vector = carla.Location(-forward_vector.y, forward_vector.x, forward_vector.z) * waypoint.lane_width / 2 * 0.7
            line = [
                (waypoint.transform.location + (forward_vector * 1.5) + (left_vector)),
                (waypoint.transform.location + (forward_vector * 1.5) - (left_vector)),
            ]
            line_pixel = [world_to_pixel(p) for p in line]
            pygame.draw.lines(surface, color, True, line_pixel, 2)

            if self.show_triggers:
                corners = _get_actor_trigger_bounding_box(actor)
                corners = [world_to_pixel(p) for p in corners]
                pygame.draw.lines(surface, trigger_color, True, corners, 2)

        def draw_topology(carla_topology, index):
            topology = [x[index] for x in carla_topology]
            topology = sorted(topology, key=lambda w: w.transform.location.z)
            set_waypoints = []

            for waypoint in topology:
                waypoints = [waypoint]
                nxt = waypoint.next(precision)
                if len(nxt) > 0:
                    nxt = nxt[0]
                    while nxt.road_id == waypoint.road_id:
                        waypoints.append(nxt)
                        nxt = nxt.next(precision)
                        if len(nxt) > 0:
                            nxt = nxt[0]
                        else:
                            break
                set_waypoints.append(waypoints)

                PARKING_COLOR = COLOR_ALUMINIUM_4_5
                SHOULDER_COLOR = COLOR_ALUMINIUM_5
                SIDEWALK_COLOR = COLOR_ALUMINIUM_3

                shoulder = [[], []]
                parking = [[], []]
                sidewalk = [[], []]

                for w in waypoints:
                    l = w.get_left_lane()
                    while l and l.lane_type != carla.LaneType.Driving:
                        if l.lane_type == carla.LaneType.Shoulder:
                            shoulder[0].append(l)
                        if l.lane_type == carla.LaneType.Parking:
                            parking[0].append(l)
                        if l.lane_type == carla.LaneType.Sidewalk:
                            sidewalk[0].append(l)
                        l = l.get_left_lane()

                    r = w.get_right_lane()
                    while r and r.lane_type != carla.LaneType.Driving:
                        if r.lane_type == carla.LaneType.Shoulder:
                            shoulder[1].append(r)
                        if r.lane_type == carla.LaneType.Parking:
                            parking[1].append(r)
                        if r.lane_type == carla.LaneType.Sidewalk:
                            sidewalk[1].append(r)
                        r = r.get_right_lane()

                draw_lane(map_surface, shoulder, SHOULDER_COLOR)
                draw_lane(map_surface, parking, PARKING_COLOR)
                draw_lane(map_surface, sidewalk, SIDEWALK_COLOR)

            for waypoints in set_waypoints:
                waypoint = waypoints[0]
                road_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
                road_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]
                polygon = road_left_side + [x for x in reversed(road_right_side)]
                polygon = [world_to_pixel(x) for x in polygon]
                if len(polygon) > 2:
                    pygame.draw.polygon(map_surface, COLOR_ALUMINIUM_5, polygon, 5)
                    pygame.draw.polygon(map_surface, COLOR_ALUMINIUM_5, polygon)

                if not waypoint.is_junction:
                    draw_lane_marking(map_surface, [waypoints, waypoints])
                    for n, wp in enumerate(waypoints):
                        if ((n + 1) % 400) == 0:
                            draw_arrow(map_surface, wp.transform)

        topology = carla_map.get_topology()
        draw_topology(topology, 0)

        if self.show_spawn_points:
            for sp in carla_map.get_spawn_points():
                draw_arrow(map_surface, sp, color=COLOR_CHOCOLATE_0)

        if self.show_connections:
            dist = 1.5

            def to_pixel(wp): return world_to_pixel(wp.transform.location)
            for wp in carla_map.generate_waypoints(dist):
                col = (0, 255, 255) if wp.is_junction else (0, 255, 0)
                for nxt in wp.next(dist):
                    pygame.draw.line(map_surface, col, to_pixel(wp), to_pixel(nxt), 2)
                if wp.lane_change & carla.LaneChange.Right:
                    r = wp.get_right_lane()
                    if r and r.lane_type == carla.LaneType.Driving:
                        pygame.draw.line(map_surface, col, to_pixel(wp), to_pixel(r), 2)
                if wp.lane_change & carla.LaneChange.Left:
                    l = wp.get_left_lane()
                    if l and l.lane_type == carla.LaneType.Driving:
                        pygame.draw.line(map_surface, col, to_pixel(wp), to_pixel(l), 2)

        actors = carla_world.get_actors()

        font_size = world_to_pixel_width(1)
        font = pygame.font.SysFont("Arial", font_size, True)

        stops = [actor for actor in actors if "stop" in actor.type_id]
        yields = [actor for actor in actors if "yield" in actor.type_id]

        stop_font_surface = font.render("STOP", False, COLOR_ALUMINIUM_2)
        stop_font_surface = pygame.transform.scale(
            stop_font_surface, (stop_font_surface.get_width(), stop_font_surface.get_height() * 2)
        )

        yield_font_surface = font.render("YIELD", False, COLOR_ALUMINIUM_2)
        yield_font_surface = pygame.transform.scale(
            yield_font_surface, (yield_font_surface.get_width(), yield_font_surface.get_height() * 2)
        )

        for ts_stop in stops:
            draw_traffic_signs(map_surface, stop_font_surface, ts_stop, trigger_color=COLOR_SCARLET_RED_1)
        for ts_yield in yields:
            draw_traffic_signs(map_surface, yield_font_surface, ts_yield, trigger_color=COLOR_ORANGE_1)

    def world_to_pixel(self, location, offset=(0, 0)):
        x = self.scale * self._pixels_per_meter * (location.x - self._world_offset[0])
        y = self.scale * self._pixels_per_meter * (location.y - self._world_offset[1])
        return [int(x - offset[0]), int(y - offset[1])]

    def world_to_pixel_width(self, width):
        return int(self.scale * self._pixels_per_meter * width)

    def get_world_center(self):
        return carla.Location(
            x=float(self._world_offset[0] + (self.width * 0.5)),
            y=float(self._world_offset[1] + (self.width * 0.5)),
            z=0.0,
        )

    def scale_map(self, scale):
        if scale != self.scale:
            self.scale = scale
            width = int(self.big_map_surface.get_width() * self.scale)
            self.surface = pygame.transform.smoothscale(self.big_map_surface, (width, width))






class World:
    def __init__(self, name, args, timeout):
        self.show_navigation = False  
        self.current_route = None

        self.client = None
        self.name = name
        self.args = args
        self.timeout = timeout
        self.server_fps = 0.0
        self.simulation_time = 0
        self.server_clock = pygame.time.Clock()

        self.world = None
        self.town_map = None

        self._hud = None
        self._input = None

        self.surface_size = [0, 0]
        self.prev_scaled_size = 0
        self.scaled_size = 0

        self.hero_actor = None
        self.spawned_hero = None
        self.hero_transform = None

        self.scale_offset = [0, 0]
        self.nav_crosshair_screen = None
        self.map_center_world = None
        self.lock_navigation_to_map_center = True

        self.result_surface = None

        self.traffic_light_surfaces = TrafficLightSurfaces()
        self.affected_traffic_light = None

        self.map_image = None
        self.border_round_surface = None
        self.original_surface_size = None
        self.hero_surface = None
        self.actors_surface = None

        self.clicked_points = []
        self.last_destination = None
        self.camera = None
        self.rgb_camera = None
        self.show_rgb = False

        self.yolo = None
        self.show_yolo_boxes = True
        self.latest_yolo_detections = []
        self.latest_tl_cluster_result = None
        self.latest_tl_cluster_state = "UNKNOWN"
        self._last_clustered_yolo_batch_id = -1
        self.tl_cluster_cfg = TLClusterConfig(
            conf_min=0.6,
            y_roi_min_ratio=0.10,
            y_roi_max_ratio=0.60,
            y_thresh_ratio=0.06,
            x_gap_max_ratio=0.25,
            score_k=0.2,
            min_cluster_size=2,
        )
        self._yolo_font = None

        self.autonomy_controller = None
        self._recenter_nav_next_tick = False

        self._route_debug_life_time = 0.24
        self._route_debug_refresh_s = 0.12
        self._route_debug_horizon_m = 50.0
        self._last_route_debug_draw_time = 0.0
        self.route_debug_3d_enabled = True
        self._route_debug_hidden_until_autonomy_reactivated = False



    def _get_data_from_carla(self):
        try:
            self.client = carla.Client(self.args.host, self.args.port)
            self.client.set_timeout(self.timeout)

            world = None
            for i in range(3):
                try:
                    if self.args.map is None:
                        world = self.client.get_world()
                    else:
                        world = self.client.load_world(self.args.map)
                    break
                except RuntimeError as e:
                    if i < 2:
                        print(f"Retry {i+1}/3 connecting to CARLA...")
                        time.sleep(2)
                    else:
                        raise e

            if world is None:
                raise RuntimeError("Failed to get world from CARLA")

            town_map = world.get_map()
            return (world, town_map)

        except RuntimeError as ex:
            logging.error(ex)
            pygame.quit()
            sys.exit()



    def start(self, hud, input_control):
        self.world, self.town_map = self._get_data_from_carla()

        sampling_resolution = 2.0
        self.route_planner = GlobalRoutePlanner(self.town_map, sampling_resolution)

        settings = self.world.get_settings()
        settings.no_rendering_mode = True
        self.world.apply_settings(settings)
        self._apply_traffic_light_timings()

        self.map_image = MapImage(
            carla_world=self.world,
            carla_map=self.town_map,
            pixels_per_meter=PIXELS_PER_METER,
            show_triggers=self.args.show_triggers,
            show_connections=self.args.show_connections,
            show_spawn_points=self.args.show_spawn_points,
        )

        self._hud = hud
        self._input = input_control
        self.reset_nav_crosshair()
        self.map_center_world = self.map_image.get_world_center()
        logging.info(
            "Navigation map center locked at x=%.2f y=%.2f z=%.2f",
            self.map_center_world.x,
            self.map_center_world.y,
            self.map_center_world.z,
        )

        self.original_surface_size = min(self._hud.dim[0], self._hud.dim[1])
        self.surface_size = self.map_image.big_map_surface.get_width()

        self.scaled_size = int(self.surface_size)
        self.prev_scaled_size = int(self.surface_size)

        self.actors_surface = pygame.Surface((self.map_image.surface.get_width(), self.map_image.surface.get_height()))
        self.actors_surface.set_colorkey(COLOR_BLACK)

        self.result_surface = pygame.Surface((self.surface_size, self.surface_size)).convert()
        self.result_surface.set_colorkey(COLOR_BLACK)


        self.select_hero_actor()
        self.hero_actor.set_autopilot(False)
        self._input.wheel_offset = MAP_DEFAULT_SCALE
        self._input.control = carla.VehicleControl()

        self._start_yolo()
        self._activate_third_person_camera()

        weak_self = weakref.ref(self)
        self.world.on_tick(lambda timestamp: World.on_world_tick(weak_self, timestamp))



    def _apply_traffic_light_timings(self):
        red_s = getattr(self.args, "tl_red", None)
        yellow_s = getattr(self.args, "tl_yellow", None)
        green_s = getattr(self.args, "tl_green", None)

        if red_s is None and yellow_s is None and green_s is None:
            return

        def _valid_seconds(value, name):
            if value is None:
                return None
            if value <= 0.0:
                logging.warning("Ignoring %s=%s (must be > 0)", name, value)
                return None
            return float(value)

        red_s = _valid_seconds(red_s, "tl_red")
        yellow_s = _valid_seconds(yellow_s, "tl_yellow")
        green_s = _valid_seconds(green_s, "tl_green")

        if red_s is None and yellow_s is None and green_s is None:
            return

        tls_actors = self.world.get_actors().filter("traffic.traffic_light*")
        if not tls_actors:
            logging.info("No traffic lights found to retime.")
            return

        applied = 0
        for tl in tls_actors:
            try:
                if red_s is not None:
                    tl.set_red_time(red_s)
                if yellow_s is not None:
                    tl.set_yellow_time(yellow_s)
                if green_s is not None:
                    tl.set_green_time(green_s)
                applied += 1
            except Exception as exc:
                logging.warning("Failed to retime traffic light %s: %s", tl.id, exc)

        logging.info(
            "Traffic-light timing override applied to %d lights (red=%s, yellow=%s, green=%s).",
            applied,
            red_s if red_s is not None else "map default",
            yellow_s if yellow_s is not None else "map default",
            green_s if green_s is not None else "map default",
        )



    def _start_yolo(self):
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        explicit_model = os.environ.get("YOLO_MODEL")

        conf_threshold = float(os.environ.get("YOLO_CONF", "0.4"))
        infer_hz = float(os.environ.get("YOLO_INFER_HZ", "10.0"))

        try:
            model_path = YoloPerception.resolve_model_path(
                explicit_model=explicit_model,
                script_dir=script_dir,
                candidates=YOLO_MODEL_CANDIDATES,
            )
        except Exception as exc:
            logging.warning("YOLO disabled: %s", exc)
            self.yolo = None
            return

        try:
            self.yolo = YoloPerception(
                model_path=model_path,
                conf_threshold=conf_threshold,
                infer_hz=infer_hz,
            )
            logging.info("YOLO enabled: %s", model_path)
        except Exception as exc:
            logging.warning("Failed to start YOLO perception: %s", exc)
            self.yolo = None

    def toggle_yolo_boxes(self):

        self.show_yolo_boxes = not self.show_yolo_boxes
        return self.show_yolo_boxes

    def toggle_route_debug_3d(self):
        self.route_debug_3d_enabled = not self.route_debug_3d_enabled
        self._last_route_debug_draw_time = 0.0
        if self.route_debug_3d_enabled and self.current_route:
            self._draw_route_3d_once(self.current_route, self._route_debug_life_time)
        return self.route_debug_3d_enabled

    def _activate_third_person_camera(self):
        if self.hero_actor is None:
            return
        if self.rgb_camera is not None:
            self.rgb_camera.destroy()
            self.rgb_camera = None
        if self.camera is None:
            self.camera = ThirdPersonCamera(self.world, self.hero_actor, self._hud.dim[0], self._hud.dim[1])
        self.show_rgb = False
        self._sync_camera_surface_conversion()

    def _activate_rgb_camera(self):
        if self.hero_actor is None:
            return
        if self.camera is not None:
            self.camera.destroy()
            self.camera = None
        if self.rgb_camera is None:
            self.rgb_camera = RGBCamera(self.world, self.hero_actor, self._hud.dim[0], self._hud.dim[1])
        self.show_rgb = True
        self._sync_camera_surface_conversion()

    def _sync_camera_surface_conversion(self):
        showing_camera_feed = not self.show_navigation
        if self.camera is not None:
            self.camera.set_display_enabled(showing_camera_feed and (not self.show_rgb))
        if self.rgb_camera is not None:
            self.rgb_camera.set_display_enabled(showing_camera_feed and self.show_rgb)

    def toggle_camera_mode(self):
        if self.hero_actor is None:
            return self.show_rgb
        if self.show_rgb:
            self._activate_third_person_camera()
        else:
            self._activate_rgb_camera()
        return self.show_rgb

    def select_hero_actor(self):
        hero_vehicles = [
            actor for actor in self.world.get_actors()
            if "vehicle" in actor.type_id and actor.attributes.get("role_name", "") == "hero"
        ]
        if len(hero_vehicles) > 0:
            self.hero_actor = random.choice(hero_vehicles)
            self.hero_transform = self.hero_actor.get_transform()
        else:
            self._spawn_hero()

    def _spawn_hero(self):
        bp_lib = self.world.get_blueprint_library()
        blueprint = bp_lib.find("vehicle.dodge.charger_2020")
        blueprint.set_attribute("role_name", "hero")
        if blueprint.has_attribute("color"):
            color = random.choice(blueprint.get_attribute("color").recommended_values)
            blueprint.set_attribute("color", color)

        while self.hero_actor is None:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.hero_actor = self.world.try_spawn_actor(blueprint, spawn_point)

        self.hero_transform = self.hero_actor.get_transform()
        self.spawned_hero = self.hero_actor

    def respawn_hero_random(self):
        """Destroy current hero and respawn at a random lane-aligned spawn point."""
        if self.world is None:
            return False

        self.autonomy_controller = None
        self.current_route = None
        self.clicked_points = []

        if self.camera is not None:
            self.camera.destroy()
            self.camera = None
        if self.rgb_camera is not None:
            self.rgb_camera.destroy()
            self.rgb_camera = None

        old_hero = self.hero_actor
        self.hero_actor = None
        self.spawned_hero = None
        self.hero_transform = None

        if old_hero is not None:
            try:
                old_hero.destroy()
            except Exception as exc:
                logging.warning("Failed to destroy old hero actor: %s", exc)

        self._spawn_hero()
        if self.hero_actor is None:
            return False

        try:
            self.hero_actor.set_autopilot(False)
        except Exception:
            pass

        self.hero_transform = self.hero_actor.get_transform()
        if self._input is not None:
            self._input.control = carla.VehicleControl()

        if self.show_rgb:
            self._activate_rgb_camera()
        else:
            self._activate_third_person_camera()

        if self.show_navigation:
            self.center_map_on_map_center()
            self._recenter_nav_next_tick = False
        return True



    def tick(self, clock):

        if getattr(self, "_recenter_nav_next_tick", False) and self.show_navigation:
            self.center_map_on_map_center()
            self._recenter_nav_next_tick = False

        active_camera = self.rgb_camera if self.show_rgb else self.camera
        if self.yolo is not None and active_camera is not None:
            frame_bgr = active_camera.get_latest_bgr_frame()
            if frame_bgr is not None:
                self.yolo.update_frame(frame_bgr)
            detections, batch_id = self.yolo.get_latest_detections_with_batch_id()
            self.latest_yolo_detections = detections
            if batch_id != self._last_clustered_yolo_batch_id:
                self.latest_tl_cluster_result = select_traffic_light_cluster_single_frame(
                    detections=self.latest_yolo_detections,
                    image_width=self._hud.dim[0],
                    image_height=self._hud.dim[1],
                    config=self.tl_cluster_cfg,
                )
                self.latest_tl_cluster_state = self.latest_tl_cluster_result.selected_state
                self._last_clustered_yolo_batch_id = batch_id
        else:
            self.latest_yolo_detections = []
            self.latest_tl_cluster_result = None
            self.latest_tl_cluster_state = "UNKNOWN"
            self._last_clustered_yolo_batch_id = -1


        if self.autonomy_controller:
            self._route_debug_hidden_until_autonomy_reactivated = False
            dt = clock.get_time() / 1000.0
            self.autonomy_controller.tick(dt)

        if self.hero_actor is not None:
            self.hero_transform = self.hero_actor.get_transform()

        self._refresh_route_3d_debug()

    @staticmethod
    def on_world_tick(weak_self, timestamp):
        self = weak_self()
        if not self:
            return
        self.server_clock.tick()
        self.server_fps = self.server_clock.get_fps()
        self.simulation_time = timestamp.elapsed_seconds





    def compute_route(self, start_location, end_location):
        start_wp = self.town_map.get_waypoint(start_location, project_to_road=True)
        end_wp = self.town_map.get_waypoint(end_location, project_to_road=True)
        return self.route_planner.trace_route(start_wp.transform.location, end_wp.transform.location)

    def set_current_route(self, route):
        self.current_route = route
        self._last_route_debug_draw_time = 0.0
        if self.route_debug_3d_enabled:
            self._draw_route_3d_once(route, self._route_debug_life_time)

    def hide_route_debug_until_autonomy_reactivated(self):
        self._route_debug_hidden_until_autonomy_reactivated = True
        self._last_route_debug_draw_time = 0.0

    @staticmethod
    def _find_closest_route_index(route, location):
        if not route or location is None:
            return 0
        closest_idx = 0
        min_dist = float("inf")
        for i, (waypoint, _) in enumerate(route):
            try:
                dist = location.distance(waypoint.transform.location)
            except Exception:
                continue
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx

    def _get_route_debug_start_index(self, route):
        if not route:
            return 0
        if self.autonomy_controller is not None:
            idx = getattr(self.autonomy_controller, "current_wp_index", None)
            if isinstance(idx, int):
                return max(0, min(len(route) - 1, idx))
        if self.hero_actor is not None:
            try:
                return self._find_closest_route_index(route, self.hero_actor.get_location())
            except Exception:
                pass
        return 0

    def _draw_route_3d_once(self, route, life_time):
        if not route:
            return
        if self.world is None:
            return

        debug = self.world.debug
        start_idx = self._get_route_debug_start_index(route)
        if start_idx >= (len(route) - 1):
            return

        drawn_distance = 0.0
        for i in range(start_idx, len(route) - 1):
            wp1 = route[i][0]
            wp2 = route[i + 1][0]
            p1 = wp1.transform.location + carla.Location(z=0.5)
            p2 = wp2.transform.location + carla.Location(z=0.5)
            debug.draw_line(
                p1, p2,
                thickness=0.06,
                color=carla.Color(18, 36, 56),
                life_time=life_time,
            )
            try:
                drawn_distance += wp1.transform.location.distance(wp2.transform.location)
            except Exception:
                pass
            if drawn_distance >= self._route_debug_horizon_m:
                break

    def _refresh_route_3d_debug(self):
        if not self.route_debug_3d_enabled:
            return
        if (
            self._route_debug_hidden_until_autonomy_reactivated
            and self.autonomy_controller is None
        ):
            return
        if not self.current_route:
            return
        now = time.time()
        if (now - self._last_route_debug_draw_time) < self._route_debug_refresh_s:
            return
        self._last_route_debug_draw_time = now
        self._draw_route_3d_once(self.current_route, self._route_debug_life_time)

    def render_clicked_points(self, surface):
        for loc in self.clicked_points:
            x, y = self.map_image.world_to_pixel(loc)
            pygame.draw.circle(surface, COLOR_SCARLET_RED_0, (x, y), 12)

    def render_route(self, surface):
        if not self.current_route:
            return
        points = []
        for wp, _ in self.current_route:
            x, y = self.map_image.world_to_pixel(wp.transform.location)
            points.append((x, y))
        if len(points) > 1:
            pygame.draw.lines(surface, COLOR_SKY_BLUE_1, False, points, 4)

    def screen_to_world(self, sx, sy):
        scale_factor = self._input.wheel_offset

        display_w, display_h = self._hud.dim
        surface_w = self.surface_size

        translation_x = self._input.mouse_offset[0] * scale_factor + self.scale_offset[0]
        translation_y = self._input.mouse_offset[1] * scale_factor + self.scale_offset[1]

        center_offset_x = abs(display_w - surface_w) / 2 * scale_factor
        center_offset_y = 0

        px = sx - translation_x - center_offset_x
        py = sy - translation_y - center_offset_y

        ppm = self.map_image._pixels_per_meter
        scale = self.map_image.scale

        world_x = px / (scale * ppm) + self.map_image._world_offset[0]
        world_y = py / (scale * ppm) + self.map_image._world_offset[1]
        return carla.Location(world_x, world_y, 0.0)

    def get_screen_center_world(self):
        if self._hud is None:
            return None
        return self.screen_to_world(self._hud.dim[0] * 0.5, self._hud.dim[1] * 0.5)

    def get_map_center_world(self):
        if self.map_center_world is None and self.map_image is not None:
            self.map_center_world = self.map_image.get_world_center()
        if self.map_center_world is None:
            return None
        return carla.Location(
            x=float(self.map_center_world.x),
            y=float(self.map_center_world.y),
            z=float(self.map_center_world.z),
        )

    def get_nav_crosshair_screen_pos(self):
        if self.nav_crosshair_screen is None:
            self.reset_nav_crosshair()
        return int(self.nav_crosshair_screen[0]), int(self.nav_crosshair_screen[1])

    def reset_nav_crosshair(self):
        if self._hud is None:
            return
        self.nav_crosshair_screen = [self._hud.dim[0] * 0.5, self._hud.dim[1] * 0.5]

    def move_nav_crosshair(self, dx, dy):
        if self._hud is None:
            return
        if self.nav_crosshair_screen is None:
            self.reset_nav_crosshair()
        x = self.nav_crosshair_screen[0] + float(dx)
        y = self.nav_crosshair_screen[1] + float(dy)
        max_x = max(0.0, float(self._hud.dim[0] - 1))
        max_y = max(0.0, float(self._hud.dim[1] - 1))
        self.nav_crosshair_screen[0] = min(max(0.0, x), max_x)
        self.nav_crosshair_screen[1] = min(max(0.0, y), max_y)

    def update_nav_crosshair_from_joystick(self, vrx, vry, dt_s):
        if not self.show_navigation:
            return
        if vrx is None or vry is None:
            return
        dt_s = max(0.0, float(dt_s))
        if dt_s <= 0.0:
            return

        center = 512.0
        span = 511.0
        deadzone = 0.12
        speed_px_per_s = 700.0

        nx = (center - float(vrx)) / span
        ny = (float(vry) - center) / span
        nx = max(-1.0, min(1.0, nx))
        ny = max(-1.0, min(1.0, ny))

        if abs(nx) < deadzone:
            nx = 0.0
        if abs(ny) < deadzone:
            ny = 0.0
        if nx == 0.0 and ny == 0.0:
            return

        self.move_nav_crosshair(nx * speed_px_per_s * dt_s, ny * speed_px_per_s * dt_s)

    def render_nav_crosshair(self, display):
        if not self.show_navigation:
            return
        cx, cy = self.get_nav_crosshair_screen_pos()
        color = COLOR_ORANGE_0
        arm = 20
        gap = 5
        width = 3

        pygame.draw.line(display, color, (cx - arm, cy), (cx - gap, cy), width)
        pygame.draw.line(display, color, (cx + gap, cy), (cx + arm, cy), width)
        pygame.draw.line(display, color, (cx, cy - arm), (cx, cy - gap), width)
        pygame.draw.line(display, color, (cx, cy + gap), (cx, cy + arm), width)
        pygame.draw.circle(display, color, (cx, cy), 5, width)



    def render_actors(self, surface):
        self.render_clicked_points(surface)
        self.render_route(surface)


        if self.hero_actor is not None:
            hero_tf = self.hero_transform if self.hero_transform is not None else self.hero_actor.get_transform()
            hero_loc = hero_tf.location
            fwd = hero_tf.get_forward_vector()
            center = self.map_image.world_to_pixel(hero_loc)

            fwd_len = math.hypot(fwd.x, fwd.y)
            if fwd_len < 1e-6:
                pygame.draw.circle(surface, COLOR_CHAMELEON_0, center, 9)
                return

            ux = fwd.x / fwd_len
            uy = fwd.y / fwd_len
            nx = -uy
            ny = ux

            tip_len = 24.0
            back_len = 13.0
            half_width = 12.0

            tip = (
                int(center[0] + ux * tip_len),
                int(center[1] + uy * tip_len),
            )
            rear_center = (
                center[0] - ux * back_len,
                center[1] - uy * back_len,
            )
            left = (
                int(rear_center[0] + nx * half_width),
                int(rear_center[1] + ny * half_width),
            )
            right = (
                int(rear_center[0] - nx * half_width),
                int(rear_center[1] - ny * half_width),
            )

            pygame.draw.polygon(surface, COLOR_CHAMELEON_0, [tip, left, right])

    def clip_surfaces(self, clipping_rect):
        self.actors_surface.set_clip(clipping_rect)
        self.result_surface.set_clip(clipping_rect)

    def _compute_scale(self, scale_factor):
        m = self._input.mouse_pos

        px = (m[0] - self.scale_offset[0]) / float(self.prev_scaled_size)
        py = (m[1] - self.scale_offset[1]) / float(self.prev_scaled_size)

        diff_between_scales = (
            (float(self.prev_scaled_size) * px) - (float(self.scaled_size) * px),
            (float(self.prev_scaled_size) * py) - (float(self.scaled_size) * py),
        )

        self.scale_offset = (
            self.scale_offset[0] + diff_between_scales[0],
            self.scale_offset[1] + diff_between_scales[1],
        )

        self.prev_scaled_size = self.scaled_size
        self.map_image.scale_map(scale_factor)



    def center_map_on_location(self, world_location):
        if world_location is None or self.map_image is None or self._hud is None or self._input is None:
            return


        self._input.mouse_offset = [0, 0]
        self.scale_offset = [0, 0]


        scale_factor = self._input.wheel_offset
        self.map_image.scale_map(scale_factor)

        target_px, target_py = self.map_image.world_to_pixel(world_location)


        center_offset_x = abs(self._hud.dim[0] - self.surface_size) / 2 * scale_factor


        self.scale_offset = [
            (self._hud.dim[0] / 2) - target_px - center_offset_x,
            (self._hud.dim[1] / 2) - target_py,
        ]

    def center_map_on_hero(self):
        """Center the navigation/map view on the hero vehicle."""
        if self.hero_actor is None:
            return
        self.center_map_on_location(self.hero_actor.get_location())

    def center_map_on_map_center(self):
        self.center_map_on_location(self.get_map_center_world())






    def render(self, display):

        if not self.show_navigation:
            display.fill(COLOR_BLACK)

            if self.show_rgb:
                if self.rgb_camera is None:
                    self._activate_rgb_camera()
                if self.rgb_camera is not None:
                    self.rgb_camera.render(display)
                    if self.show_yolo_boxes:
                        self._render_yolo_boxes(display)
            else:
                if self.camera is None:
                    self._activate_third_person_camera()
                if self.camera is not None:
                    self.camera.render(display)
                    if self.show_yolo_boxes:
                        self._render_yolo_boxes(display)

            return
        
        self.result_surface.fill(COLOR_BLACK)


        scale_factor = self._input.wheel_offset
        self.scaled_size = int(self.map_image.width * scale_factor)
        if self.scaled_size != self.prev_scaled_size:
            self._compute_scale(scale_factor)


        self.actors_surface.fill(COLOR_BLACK)
        self.render_actors(self.actors_surface)

        surfaces = (
            (self.map_image.surface, (0, 0)),
            (self.actors_surface, (0, 0)),
        )


        translation_offset = (
            self._input.mouse_offset[0] * scale_factor + self.scale_offset[0],
            self._input.mouse_offset[1] * scale_factor + self.scale_offset[1],
        )
        center_offset = (abs(display.get_width() - self.surface_size) / 2 * scale_factor, 0)

        clipping_rect = pygame.Rect(
            -translation_offset[0] - center_offset[0],
            -translation_offset[1],
            self._hud.dim[0],
            self._hud.dim[1],
        )
        self.clip_surfaces(clipping_rect)

        _blits(self.result_surface, surfaces)

        display.blit(
            self.result_surface,
            (translation_offset[0] + center_offset[0], translation_offset[1]),
        )
        self.render_nav_crosshair(display)



    def _render_yolo_boxes(self, display):
        if self._yolo_font is None:
            self._yolo_font = pygame.font.Font(pygame.font.get_default_font(), 16)

        draw_cluster_debug_overlay_pygame(
            surface=display,
            image_width=self._hud.dim[0],
            image_height=self._hud.dim[1],
            result=self.latest_tl_cluster_result
            if self.latest_tl_cluster_result is not None
            else select_traffic_light_cluster_single_frame(
                detections=[],
                image_width=self._hud.dim[0],
                image_height=self._hud.dim[1],
                config=self.tl_cluster_cfg,
            ),
            config=self.tl_cluster_cfg,
            font=self._yolo_font,
            roi_color=None,
            centerline_color=None,
            candidate_color=COLOR_ALUMINIUM_1,
            cluster_color=COLOR_SKY_BLUE_1,
            winner_color=COLOR_ORANGE_0,
        )

    def destroy(self):
        if self.yolo is not None:
            self.yolo.stop()
            self.yolo = None
        if self.rgb_camera:
            self.rgb_camera.destroy()
        if self.spawned_hero is not None:
            self.spawned_hero.destroy()
        if self.camera:
            self.camera.destroy()





