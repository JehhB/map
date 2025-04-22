import math
from typing import Literal, Optional

import numpy as np
from pyrr import Matrix44, Vector3
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

    movement_speed: float
    pov: float
    aspect_ratio: float

    def __init__(
        self,
        position: Optional[Vector3] = None,
        up: Optional[Vector3] = None,
        yaw: float = -90.0,
        pitch: float = 0.0,
        aspect_ratio: float = 1.0,
        pov: float = 90.0,
        movement_speed: float = 2.5,
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

        self.movement_speed = movement_speed
        self.pov = pov
        self.aspect_ratio = aspect_ratio

        self.__update_camera_vectors()

    @property
    def view_matrix(self) -> Matrix44:
        return Matrix44.look_at(
            self.position, self.position + self.front, self.up, dtype=np.float32
        )

    @property
    def projection_matrix(self) -> Matrix44:
        return Matrix44.perspective_projection(
            self.pov, self.aspect_ratio, 0.01, 100.0, dtype=np.float32
        )

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
        self, xoffset: float = 0, yoffset: float = 0, constrain_pitch: bool = True
    ):
        self.yaw += xoffset
        self.pitch += yoffset

        if constrain_pitch:
            if self.pitch > 89.0:
                self.pitch = 89.0
            elif self.pitch < -89.0:
                self.pitch = -89.0

        self.__update_camera_vectors()

    def pan(self, xoffset: float = 0, yoffset: float = 0, delta_time: float = 1.0):
        self.position += xoffset * delta_time * self.right
        self.position += yoffset * delta_time * self.up

    def walk(self, xoffset: float = 0, yoffset: float = 0, delta_time: float = 1.0):
        self.position += xoffset * delta_time * self.right
        self.position += yoffset * delta_time * self.front

    def __update_camera_vectors(self):
        yaw_rad = math.radians(self.yaw)
        pitch_rad = math.radians(self.pitch)

        x = math.cos(yaw_rad) * math.cos(pitch_rad)
        y = math.sin(pitch_rad)
        z = math.cos(pitch_rad) * math.sin(yaw_rad)

        front = Vector3([x, y, z], dtype=np.float32)
        self.front = front.normalized
        self.right = self.front.cross(self.world_up).normalized
        self.up = self.right.cross(self.front).normalized
