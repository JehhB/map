from dataclasses import dataclass, field
from threading import Event, Thread
from time import sleep
from typing import Optional, Tuple

import pygame
from pygame import joystick
from typing_extensions import override

from ratmap_common import AbstractEvent, EventTarget
from ratmap_common.BaseEvent import BaseEvent


@dataclass
class JoystickEventDetail:
    name: str = "Unknown"
    axes: Tuple[float, ...] = field(default_factory=tuple)
    buttons: Tuple[bool, ...] = field(default_factory=tuple)
    hats: Tuple[Tuple[float, float], ...] = field(default_factory=tuple)

    left_stick: Tuple[float, float] = field(default_factory=lambda: (0, 0))
    right_stick: Tuple[float, float] = field(default_factory=lambda: (0, 0))
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    left_bumper: bool = False
    right_bumper: bool = False
    dpad: Tuple[bool, bool, bool, bool] = field(
        default_factory=lambda: (False, False, False, False),
        metadata={"docs": "(Up, Right, Down, Left)"},
    )

    button_south: bool = False
    button_east: bool = False
    button_west: bool = False
    button_north: bool = False

    button_start: bool = False
    button_select: bool = False
    button_guide: bool = False

    left_stick_pressed: bool = False
    right_stick_pressed: bool = False

    @override
    def __eq__(self, other: object, /) -> bool:
        if not isinstance(other, JoystickEventDetail):
            return super().__eq__(other)

        # For axes, use a small threshold to ignore tiny fluctuations
        threshold = 0.01
        if len(self.axes) != len(other.axes):
            return False
        for i in range(len(self.axes)):
            if abs(self.axes[i] - other.axes[i]) > threshold:
                return False

        # Compare the remaining attributes
        return (
            abs(self.left_stick[0] - other.left_stick[0]) <= threshold
            and abs(self.left_stick[1] - other.left_stick[1]) <= threshold
            and abs(self.right_stick[0] - other.right_stick[0]) <= threshold
            and abs(self.right_stick[1] - other.right_stick[1]) <= threshold
            and abs(self.left_trigger - other.left_trigger) <= threshold
            and abs(self.right_trigger - other.right_trigger) <= threshold
            and self.left_bumper == other.left_bumper
            and self.right_bumper == other.right_bumper
            and self.dpad == other.dpad
            and self.button_south == other.button_south
            and self.button_east == other.button_east
            and self.button_west == other.button_west
            and self.button_north == other.button_north
            and self.button_start == other.button_start
            and self.button_select == other.button_select
            and self.button_guide == other.button_guide
            and self.left_stick_pressed == other.left_stick_pressed
            and self.right_stick_pressed == other.right_stick_pressed
        )


class JoystickEvent(BaseEvent):
    detail: JoystickEventDetail

    def __init__(
        self,
        type: str,
        target: object = None,
        detail: Optional[JoystickEventDetail] = None,
    ):
        if detail is None:
            detail = JoystickEventDetail()
        super().__init__(type, target, detail)


class Joystick(EventTarget[JoystickEvent]):
    __joystick: Optional[joystick.JoystickType]
    __stop_event: Event
    __task_thread: Optional[Thread]
    __prev_detail: Optional[JoystickEventDetail]

    def __init__(self) -> None:
        super().__init__()
        _ = pygame.init()
        pygame.joystick.init()
        self.__is_running = False
        self.__stop_event = Event()
        self.__task_thread = None
        self.__joystick = None
        self.__prev_detail = None
        self.reload_joystick()

    def reload_joystick(self):
        if self.__joystick is not None:
            self.__joystick.quit()
        self.__joystick = None
        count = pygame.joystick.get_count()
        if count > 0:
            self.__joystick = pygame.joystick.Joystick(0)
            self.__joystick.init()

    def __map_ps4_controller(self, detail: JoystickEventDetail) -> JoystickEventDetail:
        """Map PS4 controller inputs to the generic format"""
        if len(detail.axes) >= 6:

            detail.left_stick = (detail.axes[0], detail.axes[1])

            detail.right_stick = (detail.axes[2], detail.axes[5])

            detail.left_trigger = (detail.axes[3] + 1) / 2
            detail.right_trigger = (detail.axes[4] + 1) / 2

        if len(detail.buttons) >= 13:

            detail.button_south = detail.buttons[0]
            detail.button_east = detail.buttons[1]
            detail.button_west = detail.buttons[3]
            detail.button_north = detail.buttons[2]

            detail.left_bumper = detail.buttons[4]
            detail.right_bumper = detail.buttons[5]

            detail.left_stick_pressed = detail.buttons[10]
            detail.right_stick_pressed = detail.buttons[11]

            detail.button_select = detail.buttons[8]
            detail.button_start = detail.buttons[9]
            detail.button_guide = detail.buttons[12]

        if len(detail.hats) > 0:
            hat = detail.hats[0]
            detail.dpad = (
                hat[1] < 0,
                hat[0] > 0,
                hat[1] > 0,
                hat[0] < 0,
            )

        return detail

    def __map_xbox_controller(self, detail: JoystickEventDetail) -> JoystickEventDetail:
        """Map Xbox controller inputs to the generic format"""
        if len(detail.axes) >= 6:

            detail.left_stick = (detail.axes[0], detail.axes[1])

            detail.right_stick = (detail.axes[3], detail.axes[4])

            detail.left_trigger = detail.axes[2]
            detail.right_trigger = detail.axes[5]

        if len(detail.buttons) >= 10:

            detail.button_south = detail.buttons[0]
            detail.button_east = detail.buttons[1]
            detail.button_west = detail.buttons[2]
            detail.button_north = detail.buttons[3]

            detail.left_bumper = detail.buttons[4]
            detail.right_bumper = detail.buttons[5]

            detail.left_stick_pressed = detail.buttons[8]
            detail.right_stick_pressed = detail.buttons[9]

            detail.button_select = detail.buttons[6]
            detail.button_start = detail.buttons[7]
            if len(detail.buttons) >= 11:
                detail.button_guide = detail.buttons[10]

        if len(detail.hats) > 0:
            hat = detail.hats[0]
            detail.dpad = (
                hat[1] < 0,
                hat[0] > 0,
                hat[1] > 0,
                hat[0] < 0,
            )

        return detail

    def __map_generic_controller(
        self, detail: JoystickEventDetail
    ) -> JoystickEventDetail:
        """Default mapping for unknown controllers"""
        if len(detail.axes) >= 4:
            detail.left_stick = (detail.axes[0], detail.axes[1])
            detail.right_stick = (detail.axes[2], detail.axes[3])

        if len(detail.axes) >= 6:
            detail.left_trigger = detail.axes[4]
            detail.right_trigger = detail.axes[5]

        if len(detail.buttons) >= 10:

            detail.button_south = detail.buttons[0]
            detail.button_east = detail.buttons[1]
            detail.button_west = detail.buttons[2]
            detail.button_north = detail.buttons[3]
            detail.left_bumper = detail.buttons[4]
            detail.right_bumper = detail.buttons[5]
            detail.button_select = detail.buttons[6]
            detail.button_start = detail.buttons[7]
            detail.left_stick_pressed = detail.buttons[8]
            detail.right_stick_pressed = detail.buttons[9]

        if len(detail.hats) > 0:
            hat = detail.hats[0]
            detail.dpad = (
                hat[1] < 0,
                hat[0] > 0,
                hat[1] > 0,
                hat[0] < 0,
            )

        return detail

    def __joystick_task(self):
        while not self.__stop_event.is_set():
            if self.__joystick is not None:
                pygame.event.pump()
                detail = JoystickEventDetail()

                detail.name = self.__joystick.get_name()

                num_axes = self.__joystick.get_numaxes()
                detail.axes = tuple(
                    self.__joystick.get_axis(i) for i in range(num_axes)
                )

                num_buttons = self.__joystick.get_numbuttons()
                detail.buttons = tuple(
                    self.__joystick.get_button(i) == 1 for i in range(num_buttons)
                )

                num_hats = self.__joystick.get_numhats()
                detail.hats = tuple(self.__joystick.get_hat(i) for i in range(num_hats))

                if (
                    "PS4" in detail.name
                    or "PlayStation" in detail.name
                    or "Wireless Controller" in detail.name
                ):
                    detail = self.__map_ps4_controller(detail)
                elif "Xbox" in detail.name:
                    detail = self.__map_xbox_controller(detail)
                else:
                    detail = self.__map_generic_controller(detail)

                if self.__prev_detail != detail:
                    event = JoystickEvent("joystick.change", self, detail)
                    self.emit(event)

                event = JoystickEvent("joystick.poll", self, detail)
                self.emit(event)

                self.__prev_detail = detail

            sleep(0.05)

    def start_thread(self) -> None:
        if self.__task_thread is not None:
            self.__stop_event.set()
            self.__task_thread.join()

        self.__stop_event.clear()
        self.__task_thread = Thread(target=self.__joystick_task, daemon=True)
        self.__task_thread.start()

    def stop_thread(self) -> None:
        if self.__task_thread is not None:
            self.__stop_event.set()
            self.__task_thread.join()
            self.__task_thread = None

    def rumble(self, low_freq: float, high_freq: float, duration: int) -> bool:
        if self.__joystick is not None:
            return self.__joystick.rumble(low_freq, high_freq, duration)
        return False

    def stop_rumble(self):
        if self.__joystick is not None:
            self.__joystick.stop_rumble()

    @override
    def dispose(self) -> None:
        self.stop_thread()
        if self.__joystick is not None:
            self.stop_rumble()
            self.__joystick.quit()
        self.__joystick = None
        pygame.joystick.quit()
        super().dispose()
