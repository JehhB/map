from __future__ import annotations

import math
import tkinter as tk

from reactivex.subject import BehaviorSubject
from typing_extensions import override

from tkinter_rx.util import SetDisposer


class Joystick(tk.Canvas):
    size: int
    radius: int
    stick_radius: int

    center_x: int
    center_y: int

    max_distance: int
    x: BehaviorSubject[float]
    y: BehaviorSubject[float]

    stick: int

    def __init__(
        self,
        parent: tk.Misc,
        x_subject: BehaviorSubject[float],
        y_subject: BehaviorSubject[float],
        size: int = 100,
    ):
        """
        Create a joystick widget that reports x and y position values from -1 to 1.

        Args:
            parent: Parent widget
            size: Size of the joystick in pixels
            **kwargs: Additional arguments to pass to the Canvas constructor
        """
        super().__init__(parent, width=size, height=size)
        self.size = size
        self.radius = size // 2
        self.stick_radius = size // 6

        # Center coordinates
        self.center_x = self.radius
        self.center_y = self.radius

        # Maximum distance the joystick can move from center
        self.max_distance = self.radius - self.stick_radius

        # Create reactive BehaviorSubjects for x and y positions (-1 to 1)
        # These are the single source of truth for joystick position
        self.x = x_subject
        self.y = y_subject

        # Draw the joystick components
        _ = self.create_oval(
            0, 0, self.size, self.size, fill="#e0e0e0", outline="#808080", width=2
        )
        self.stick = self.create_oval(
            self.center_x - self.stick_radius,
            self.center_y - self.stick_radius,
            self.center_x + self.stick_radius,
            self.center_y + self.stick_radius,
            fill="#808080",
            outline="#606060",
            width=2,
        )

        # Subscribe to changes in x and y to update the visual joystick position
        self.__disposer = SetDisposer()
        self.__disposer.add(self.x.subscribe(lambda _: self.__update_visual_position()))
        self.__disposer.add(self.y.subscribe(lambda _: self.__update_visual_position()))

        # Bind events
        _ = self.bind("<Button-1>", self.__on_mouse_down)
        _ = self.bind("<B1-Motion>", self.__on_mouse_drag)
        _ = self.bind("<ButtonRelease-1>", self.__on_mouse_up)

    def __on_mouse_down(self, event: tk.Event[tk.Misc]):
        """Handle mouse down event."""
        self.__update_from_mouse_position(event.x, event.y)

    def __on_mouse_drag(self, event: tk.Event[tk.Misc]):
        """Handle mouse drag event."""
        self.__update_from_mouse_position(event.x, event.y)

    def __on_mouse_up(self, _event: tk.Event[tk.Misc]):
        """Handle mouse up event by returning to center position."""
        self.set_position(0, 0)

    def __update_from_mouse_position(self, mouse_x: int, mouse_y: int):
        """
        Update the joystick position based on mouse coordinates.
        Constrain the joystick to the circular area.
        """
        # Calculate distance from center
        dx = mouse_x - self.center_x
        dy = mouse_y - self.center_y
        distance = math.sqrt(dx * dx + dy * dy)

        # If distance is greater than max distance, normalize
        if distance > self.max_distance:
            dx = dx * self.max_distance / distance
            dy = dy * self.max_distance / distance

        # Calculate normalized values (-1 to 1) and update subjects
        normalized_x = dx / self.max_distance
        normalized_y = dy / self.max_distance

        # Update the subjects (will trigger visual update via subscription)
        self.set_position(normalized_x, normalized_y)

    def __update_visual_position(self):
        """
        Update the visual joystick position based on the current values of x and y subjects.
        """
        # Get current values from subjects
        print(self)
        norm_x = self.x.value
        norm_y = self.y.value

        # Calculate absolute coordinates
        dx = norm_x * self.max_distance
        dy = norm_y * self.max_distance

        # Calculate final position
        stick_x = self.center_x + dx
        stick_y = self.center_y + dy

        # Move the joystick handle on canvas
        self.coords(
            self.stick,
            stick_x - self.stick_radius,
            stick_y - self.stick_radius,
            stick_x + self.stick_radius,
            stick_y + self.stick_radius,
        )

    def get_normalized_position(self):
        """Return the current normalized position as (x, y) where both are in range [-1, 1]."""
        return (self.x.value, self.y.value)

    def set_position(self, x: float, y: float):
        """Set the joystick position with normalized values (-1 to 1)."""
        # Clamp values to [-1, 1] range
        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))

        # Only update if values have changed significantly
        if abs(self.x.value - x) > 0.001 or abs(self.y.value - y) > 0.001:
            self.x.on_next(x)
            self.y.on_next(y)

    @override
    def destroy(self) -> None:
        self.__disposer.dispose()
        return super().destroy()
