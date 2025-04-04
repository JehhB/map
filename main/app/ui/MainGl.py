import threading
import time
import tkinter as tk
from typing import final

import numpy as np
from cv2.typing import MatLike
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from pyopengltk import OpenGLFrame
from typing_extensions import override

from app.Container import AbstractModule, Container, ModuleDefinition
from app.ui.Main import Main


@final
class MainGl(AbstractModule, OpenGLFrame):
    _container: Container

    @override
    @staticmethod
    def KEY() -> str:
        return "ui.maingl"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition["MainGl"]:
        return MainGl.factory, [Main]

    @staticmethod
    def factory(container: Container):
        main = container[Main]
        main_gl = MainGl(container, main, width=500, height=500)
        main_gl.pack(fill=tk.BOTH, expand=tk.YES)
        return main_gl

    def __init__(self, container: Container, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._container = container
        self.last_time = time.time()
        self.zoom = 5
        self._points_lock = threading.RLock()
        self._points = np.array([], dtype=np.float32)

        # Mouse rotation variables
        self.rotation_x = 0
        self.rotation_y = 0
        self.mouse_x = 0
        self.mouse_y = 0
        self.dragging = False

        def zoom(delta):
            self.zoom = max(self.zoom + delta, 1)

        # Mouse wheel for zoom
        self.bind("<Button-4>", lambda *_: zoom(-0.2))
        self.bind("<Button-5>", lambda *_: zoom(0.2))

        # Mouse drag for rotation
        self.bind("<ButtonPress-1>", self.start_drag)
        self.bind("<ButtonRelease-1>", self.stop_drag)
        self.bind("<B1-Motion>", self.drag)

    def start_drag(self, event):
        """Start tracking mouse movement for rotation."""
        self.mouse_x = event.x
        self.mouse_y = event.y
        self.dragging = True

    def stop_drag(self, event):
        """Stop tracking mouse movement."""
        self.dragging = False

    def drag(self, event):
        """Handle mouse dragging to rotate the view."""
        if self.dragging:
            # Calculate the rotation delta based on mouse movement
            dx = event.x - self.mouse_x
            dy = event.y - self.mouse_y

            # Update rotation angles
            self.rotation_y += dx * 0.5
            self.rotation_x += dy * 0.5

            # Store current mouse position
            self.mouse_x = event.x
            self.mouse_y = event.y

    @override
    def initgl(self):
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0, 0.1, 50.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0, 0, -self.zoom)

    @override
    def redraw(self):
        glPointSize(2)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0, -self.zoom)

        # Apply rotation based on mouse input
        glRotatef(self.rotation_x, 1, 0, 0)
        glRotatef(self.rotation_y, 0, 1, 0)

        self._draw_points()
        self.tkSwapBuffers()
        self.after(100, self.redraw)

    def set_points(self, points: MatLike):
        """
        Set the points to be rendered.
        Thread-safe method to update the points array.
        Args:
            points: Numpy array with shape (n, 6) or continuous array where each point
                   is represented by 6 consecutive values (x, y, z, r, g, b)
        """
        with self._points_lock:
            # Make a copy to avoid any potential modification from outside
            if points.size > 0:
                # Ensure we have a properly shaped array (reshape if necessary)
                if points.ndim == 1:
                    # If it's a flat array, reshape it to have 6 columns
                    points_reshaped = points.reshape(-1, 6)
                else:
                    points_reshaped = points
                # Make a copy to ensure thread safety
                self._points = np.array(points_reshaped, dtype=np.float32)
            else:
                self._points = np.array([], dtype=np.float32)

    def _draw_points(self):
        """
        Draw the points using OpenGL.
        Each point has position (x, y, z) and color (r, g, b).
        """
        with self._points_lock:
            if self._points.size == 0:
                return
            glBegin(GL_POINTS)
            for i in range(len(self._points)):
                x, y, z, r, g, b = self._points[i]
                glColor3f(r, g, b)
                glVertex3f(x, y, z)
            glEnd()
