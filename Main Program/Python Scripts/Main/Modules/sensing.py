import weakref
import time

import numpy as np
import pygame
import carla

from Modules.constants import (
    COLOR_ALUMINIUM_5,
    COLOR_ORANGE_2,
    COLOR_ALUMINIUM_4,
    COLOR_SCARLET_RED_0,
    COLOR_BUTTER_0,
    COLOR_CHAMELEON_0,
)

tls = carla.TrafficLightState


class TrafficLightSurfaces:
    def __init__(self):
        def make_surface(tl_state):
            w = 40
            surface = pygame.Surface((w, 3 * w), pygame.SRCALPHA)
            surface.fill(COLOR_ALUMINIUM_5 if tl_state != "h" else COLOR_ORANGE_2)

            if tl_state != "h":
                hw = int(w / 2)
                off = COLOR_ALUMINIUM_4
                red = COLOR_SCARLET_RED_0
                yellow = COLOR_BUTTER_0
                green = COLOR_CHAMELEON_0

                pygame.draw.circle(surface, red if tl_state == tls.Red else off, (hw, hw), int(0.4 * w))
                pygame.draw.circle(surface, yellow if tl_state == tls.Yellow else off, (hw, w + hw), int(0.4 * w))
                pygame.draw.circle(surface, green if tl_state == tls.Green else off, (hw, 2 * w + hw), int(0.4 * w))

            return pygame.transform.smoothscale(surface, (15, 45) if tl_state != "h" else (19, 49))

        self._original_surfaces = {
            "h": make_surface("h"),
            tls.Red: make_surface(tls.Red),
            tls.Yellow: make_surface(tls.Yellow),
            tls.Green: make_surface(tls.Green),
            tls.Off: make_surface(tls.Off),
            tls.Unknown: make_surface(tls.Unknown),
        }
        self.surfaces = dict(self._original_surfaces)

    def rotozoom(self, angle, scale):
        for key, surface in self._original_surfaces.items():
            self.surfaces[key] = pygame.transform.rotozoom(surface, angle, scale)


class ThirdPersonCamera:
    def __init__(self, world, vehicle, width, height):
        self.surface = None
        self.latest_bgr_frame = None
        self.sensor = None
        self._display_enabled = True
        self._fps = 0.0
        self._fps_frames = 0
        self._fps_window_start = time.time()

        bp_lib = world.get_blueprint_library()
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(width))
        cam_bp.set_attribute("image_size_y", str(height))
        cam_bp.set_attribute("fov", "90")

        transform = carla.Transform(
            carla.Location(x=-6.0, z=3.0),
            carla.Rotation(pitch=20.0),
        )

        self.sensor = world.spawn_actor(
            cam_bp,
            transform,
            attach_to=vehicle,
            attachment_type=carla.AttachmentType.SpringArmGhost,
        )

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: ThirdPersonCamera._on_image(weak_self, image))

    @staticmethod
    def _on_image(weak_self, image):
        self = weak_self()
        if not self:
            return

        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        bgr_array = array[:, :, :3]


        self.latest_bgr_frame = bgr_array.copy()
        if self._display_enabled:
            rgb_array = bgr_array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(rgb_array.swapaxes(0, 1))
        self._fps_frames += 1
        now = time.time()
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            self._fps = self._fps_frames / elapsed
            self._fps_frames = 0
            self._fps_window_start = now

    def set_display_enabled(self, enabled):
        self._display_enabled = bool(enabled)
        if not self._display_enabled:
            self.surface = None

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    def get_latest_bgr_frame(self):
        if self.latest_bgr_frame is None:
            return None
        return self.latest_bgr_frame

    def get_sensor_fps(self):
        return self._fps

    def destroy(self):
        if self.sensor:
            self.sensor.stop()
            self.sensor.destroy()


class RGBCamera:
    def __init__(self, world, vehicle, width, height):
        self.sensor = None
        self.surface = None
        self.latest_bgr_frame = None
        self._display_enabled = True
        self._fps = 0.0
        self._fps_frames = 0
        self._fps_window_start = time.time()

        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(width))
        bp.set_attribute("image_size_y", str(height))
        bp.set_attribute("fov", "90")

        bound_x = 0.5 + vehicle.bounding_box.extent.x
        bound_z = 0.5 + vehicle.bounding_box.extent.z
        transform = carla.Transform(
            carla.Location(x=0.3 * bound_x, y=0.0, z=1.05 * bound_z),
            carla.Rotation(pitch=10.0),
        )

        self.sensor = world.spawn_actor(
            bp,
            transform,
            attach_to=vehicle,
            attachment_type=carla.AttachmentType.Rigid,
        )

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: RGBCamera._parse_image(weak_self, image))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        bgr_array = array[:, :, :3]


        self.latest_bgr_frame = bgr_array.copy()
        if self._display_enabled:
            rgb_array = bgr_array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(rgb_array.swapaxes(0, 1))
        self._fps_frames += 1
        now = time.time()
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            self._fps = self._fps_frames / elapsed
            self._fps_frames = 0
            self._fps_window_start = now

    def set_display_enabled(self, enabled):
        self._display_enabled = bool(enabled)
        if not self._display_enabled:
            self.surface = None

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    def get_latest_bgr_frame(self):
        if self.latest_bgr_frame is None:
            return None
        return self.latest_bgr_frame

    def get_sensor_fps(self):
        return self._fps

    def destroy(self):
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()
