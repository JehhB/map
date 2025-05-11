# pyright: reportMissingTypeStubs=false, reportUnknownMemberType=false, reportUnknownArgumentType=false, reportUnknownVariableType=false, reportAny=false

from __future__ import annotations

import tkinter as tk
from threading import RLock
from typing import Callable, List, Literal, Optional, Tuple, final

from OpenGL import GL
from pyopengltk import OpenGLFrame
from pyrr import Vector3
from typing_extensions import override

from ratmap_common.EventTarget import EventTarget
from tkinter_rx.TkinterEvent import TkinterEvent

from .Camera import Camera
from .Mesh import Mesh


@final
class MainGl(OpenGLFrame):
    DEFAULT_GL_WIDTH = 640
    DEFAULT_GL_HEIGHT = 480
    __event_target: EventTarget
    __mesh: List[Optional[Mesh]]

    __lock: RLock

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(
            master, width=MainGl.DEFAULT_GL_WIDTH, height=MainGl.DEFAULT_GL_HEIGHT
        )

        self.__event_target = EventTarget()
        self.shader_program = None

        self.__lock = RLock()
        self.__mesh = []

        self.__camera: Camera
        self.__last_clicked: Optional[Tuple[int, int]] = None

    @override
    def initgl(self):
        # Set up viewport and clear color
        GL.glViewport(0, 0, self.width, self.height)
        GL.glClearColor(0.1, 0.1, 0.1, 1.0)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
        GL.glLineWidth(2)

        self.__camera = Camera(Vector3([0.0, 0.0, 5.0]))

        # Create shaders and program
        self.create_shader_program()

        _ = self.bind("<Configure>", self.__handle_resize)
        _ = self.bind("<B1-Motion>", self.__on_drag)
        _ = self.bind("<ButtonRelease-1>", self.__release_hold)

        _ = self.bind(
            "<Button-1>",
            lambda e: self.event_target.emit(TkinterEvent("main_gl.click", self, e)),
        )

        _ = self.bind(
            "<Double-Button-1>",
            lambda e: self.event_target.emit(
                TkinterEvent("main_gl.double_click", self, e)
            ),
        )

    def create_shader_program(self):
        # Vertex Shader source code - using GLSL 1.30
        vertex_shader_source = """
        #version 130
        
        in vec3 aPos;
        in vec3 aColor;
        
        out vec3 vertexColor;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        
        void main()
        {
            vec4 clipPos = projection * view * model * vec4(aPos, 1.0);
            gl_Position = clipPos;
            float ndcDepth = clipPos.z;
            gl_PointSize = 5 / abs(ndcDepth);
            vertexColor = aColor;
        }
        """

        # Fragment Shader source code - using GLSL 1.30
        fragment_shader_source = """
        #version 130
        
        in vec3 vertexColor;
        out vec4 FragColor;
        
        void main()
        {
            FragColor = vec4(vertexColor, 1.0);
        }
        """

        # Compile vertex shader
        vertex_shader = GL.glCreateShader(GL.GL_VERTEX_SHADER)
        GL.glShaderSource(vertex_shader, vertex_shader_source)
        GL.glCompileShader(vertex_shader)
        self.check_compile_errors(vertex_shader, "VERTEX")

        # Compile fragment shader
        fragment_shader = GL.glCreateShader(GL.GL_FRAGMENT_SHADER)
        GL.glShaderSource(fragment_shader, fragment_shader_source)
        GL.glCompileShader(fragment_shader)
        self.check_compile_errors(fragment_shader, "FRAGMENT")

        # Link shaders to create a shader program
        self.shader_program = GL.glCreateProgram()
        GL.glAttachShader(self.shader_program, vertex_shader)
        GL.glAttachShader(self.shader_program, fragment_shader)
        GL.glLinkProgram(self.shader_program)
        self.check_compile_errors(self.shader_program, "PROGRAM")

        # Delete the shaders as they're now linked into the program and no longer needed
        GL.glDeleteShader(vertex_shader)
        GL.glDeleteShader(fragment_shader)

    def check_compile_errors(self, shader, shader_type):
        if shader_type != "PROGRAM":
            success = GL.glGetShaderiv(shader, GL.GL_COMPILE_STATUS)
            if not success:
                info_log = GL.glGetShaderInfoLog(shader)
                print(
                    f"ERROR::SHADER_COMPILATION_ERROR of type: {shader_type}\n{info_log.decode('ascii')}"
                )
        else:
            success = GL.glGetProgramiv(shader, GL.GL_LINK_STATUS)
            if not success:
                info_log = GL.glGetProgramInfoLog(shader)
                print(
                    f"ERROR::PROGRAM_LINKING_ERROR of type: {shader_type}\n{info_log.decode('ascii')}"
                )

    @override
    def redraw(self):
        with self.__lock:
            # Clear the screen
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

            # Use shader program
            GL.glUseProgram(self.shader_program)

            model_loc = GL.glGetUniformLocation(self.shader_program, "model")
            view_loc = GL.glGetUniformLocation(self.shader_program, "view")
            proj_loc = GL.glGetUniformLocation(self.shader_program, "projection")

            GL.glUniformMatrix4fv(
                view_loc, 1, GL.GL_FALSE, self.__camera.view_matrix.flatten()
            )
            GL.glUniformMatrix4fv(
                proj_loc, 1, GL.GL_FALSE, self.__camera.projection_matrix.flatten()
            )

            for mesh in self.__mesh:
                if mesh is not None:
                    mesh.draw(model_loc)

            # Schedule the next redraw
            _ = self.after(16, self.tkExpose, None)  # ~60 FPS

            GL.glUseProgram(0)

    @property
    def event_target(self):
        return self.__event_target

    @override
    def destroy(self) -> None:
        with self.__lock:
            if self.shader_program:
                GL.glDeleteProgram(self.shader_program)

            for mesh in self.__mesh:
                if mesh is not None:
                    mesh.free()
            self.__mesh = []

            self.__event_target.dispose()

        return super().destroy()

    def add_mesh(self, mesh: Mesh) -> int:
        with self.__lock:
            ret = len(self.__mesh)
            self.__mesh.append(mesh)
            return ret

    def new_mesh(
        self,
        type: Literal["points", "triangles", "lines", "line_strip", "quads"] = "points",
    ) -> int:
        if type == "triangles":
            type_int = GL.GL_TRIANGLES
        elif type == "lines":
            type_int = GL.GL_LINES
        elif type == "line_strip":
            type_int = GL.GL_LINE_STRIP
        elif type == "quads":
            type_int = GL.GL_QUADS
        else:
            type_int = GL.GL_POINTS

        mesh = Mesh(type=type_int)
        return self.add_mesh(mesh)

    def remove_mesh(self, id: int) -> None:
        with self.__lock:
            self.__mesh[id] = None

    def get_mesh(self, id: int) -> Optional[Mesh]:
        with self.__lock:
            return self.__mesh[id]

    def update_mesh(self, id: int, callback: Callable[[Mesh], None]) -> None:
        if id < 0:
            return

        with self.__lock:
            try:
                mesh = self.__mesh[id]
                if mesh is not None:
                    callback(mesh)
            except IndexError:
                pass

    @property
    def camera(self):
        return self.__camera

    def __handle_resize(self, e: tk.Event[tk.Misc]):
        GL.glViewport(0, 0, e.width, e.height)
        self.__camera.aspect_ratio = e.width / e.height

    def __on_drag(self, event: tk.Event[tk.Misc]):
        mouse_sensitivity = 0.1

        is_zooming = isinstance(event.state, int) and event.state & 0x0001

        if self.__last_clicked is not None:
            x_offset = event.x - self.__last_clicked[0]
            y_offset = event.y - self.__last_clicked[1]
            if is_zooming:
                self.__camera.zoom = self.__camera.zoom - y_offset * mouse_sensitivity
            else:
                self.__camera.pan(
                    -x_offset * mouse_sensitivity, -y_offset * mouse_sensitivity, 0.5
                )

        self.__last_clicked = (event.x, event.y)

    def __release_hold(self, _e: tk.Event[tk.Misc]):
        self.__last_clicked = None
