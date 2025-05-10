import math
from typing import Literal, Optional, Tuple

import numpy as np
from pyrr import Matrix44, Vector3, Vector4
from typing_extensions import TypeAlias

CameraMovement: TypeAlias = Literal[
    "forward", "backward", "left", "right", "up", "down"
]


class Camera:
    position: Vector3
    front: Vector3
    up: Vector3
    right: Vector3
    world_up: Vector3

    yaw: float
    pitch: float
    roll: float

    movement_speed: float
    pov: float
    aspect_ratio: float
    __zoom: float

    def __init__(
        self,
        position: Optional[Vector3] = None,
        up: Optional[Vector3] = None,
        yaw: float = -90.0,
        pitch: float = 0.0,
        roll: float = 0.0,
        aspect_ratio: float = 1.0,
        pov: float = 90.0,
        movement_speed: float = 2.5,
        zoom: float = 10.0,
    ) -> None:
        self.position = (
            position
            if position is not None
            else Vector3([0.0, 0.0, 0.0], dtype=np.float32)
        )
        self.world_up = (
            up if up is not None else Vector3([0.0, 1.0, 0.0], dtype=np.float32)
        )
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

        self.movement_speed = movement_speed
        self.pov = pov
        self.aspect_ratio = aspect_ratio
        self.__zoom = zoom

        self.__update_camera_vectors()

    @property
    def view_matrix(self) -> Matrix44:
        return Matrix44.look_at(
            self.position, self.position + self.front, self.up, dtype=np.float32
        )

    @property
    def projection_matrix(self) -> Matrix44:
        z = self.__zoom / 2
        return Matrix44.orthogonal_projection(
            z, -z, z, -z, 0.01, 100.0, dtype=np.float32
        )

    @property
    def zoom(self):
        return self.__zoom

    @zoom.setter
    def zoom(self, val: float):
        self.__zoom = min(max(1.0, val), 50.0)

    def move(self, movement: CameraMovement, delta_time: float = 1.0):
        disp = self.movement_speed * delta_time

        if movement == "forward":
            self.position += self.front * disp
        elif movement == "backward":
            self.position -= self.front * disp
        elif movement == "left":
            self.position -= self.right * disp
        elif movement == "right":
            self.position += self.right * disp
        elif movement == "up":
            self.position += self.world_up * disp
        elif movement == "down":
            self.position -= self.world_up * disp

    def look_around(
        self,
        xoffset: float = 0,
        yoffset: float = 0,
        zoffset: float = 0,
        constrain_pitch: bool = True,
    ):
        self.yaw += xoffset
        self.pitch += yoffset
        self.roll += zoffset

        if constrain_pitch:
            self.pitch = max(min(self.pitch, 89.0), -89.0)

        self.__update_camera_vectors()

    def pan(self, xoffset: float = 0, yoffset: float = 0, delta_time: float = 1.0):
        self.position += xoffset * delta_time * self.right
        self.position += yoffset * delta_time * self.up

    def walk(self, xoffset: float = 0, yoffset: float = 0, delta_time: float = 1.0):
        self.position += xoffset * delta_time * self.right
        self.position += yoffset * delta_time * self.front

    def reorient(self):
        self.pitch = 0
        self.yaw = -90
        self.roll = 0
        self.__update_camera_vectors()

    def __update_camera_vectors(self):
        yaw_rad = math.radians(self.yaw)
        pitch_rad = math.radians(self.pitch)
        roll_rad = math.radians(self.roll)

        x = math.cos(yaw_rad) * math.cos(pitch_rad)
        y = math.sin(pitch_rad)
        z = math.sin(yaw_rad) * math.cos(pitch_rad)

        front = Vector3([x, y, z], dtype=np.float32)
        self.front = front.normalized
        self.right = self.front.cross(self.world_up).normalized

        # Apply roll to up vector
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)
        self.up = (
            self.right * -sin_r + self.front.cross(self.right) * cos_r
        ).normalized

    def get_world_position(self, xy: Tuple[float, float]) -> Tuple[float, float]:
        """
        Convert screen coordinates to world coordinates at the XY plane (z=0).

        Args:
            xy: Tuple[float, float] - Screen coordinates where [0,0] is top-left
                and [1,1] is bottom-right

        Returns:
            Tuple[float, float] - World coordinates (x, y) at the XY plane
        """
        # Normalize screen coordinates to NDC space (-1 to 1)
        x_ndc = 2.0 * xy[0] - 1.0
        # Flip y-axis since screen coordinates are top-down
        y_ndc = 1.0 - 2.0 * xy[1]

        # Create ray in NDC space
        ndc_near = Vector4([x_ndc, y_ndc, -1.0, 1.0], dtype=np.float32)
        ndc_far = Vector4([x_ndc, y_ndc, 1.0, 1.0], dtype=np.float32)

        # Get inverse of projection-view matrix
        inv_proj_view = (self.projection_matrix * self.view_matrix).inverse

        # Transform to world space
        world_near = inv_proj_view * ndc_near
        world_far = inv_proj_view * ndc_far

        # Perspective division
        if abs(world_near.w) > 1e-6:
            world_near = world_near / world_near.w
        if abs(world_far.w) > 1e-6:
            world_far = world_far / world_far.w

        # Create ray direction
        ray_origin = Vector3([world_near.x, world_near.y, world_near.z])
        ray_target = Vector3([world_far.x, world_far.y, world_far.z])
        ray_direction = (ray_target - ray_origin).normalized

        # Intersect with XY plane (z=0)
        # If ray is parallel to XY plane, return None
        if abs(ray_direction.z) < 1e-6:
            # Return camera position projected to XY plane as fallback
            return (self.position.x, self.position.y)

        # Calculate distance to intersection
        t = -ray_origin.z / ray_direction.z

        # Calculate intersection point
        intersection = ray_origin + ray_direction * t

        return (float(intersection.x), float(intersection.y))
