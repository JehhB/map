# pyright: reportMissingTypeStubs=false, reportUnknownMemberType=false, reportUnknownArgumentType=false, reportUnknownVariableType=false, reportAny=false

import ctypes
import math
import tkinter as tk
from threading import RLock
from typing import List, Optional, final

import numpy as np
from numpy.typing import NDArray
from OpenGL import GL
from pyopengltk import OpenGLFrame
from pyrr import Matrix44
from typing_extensions import override

from ratmap_common.EventTarget import EventTarget


class Mesh:
    model_matrix: Matrix44

    def __init__(
        self,
        vertices: NDArray[np.float32],
        indices: NDArray[np.uint32],
        pos_loc: int = 0,
        color_loc: int = 1,
        model_matrix: Optional[Matrix44] = None,
    ):
        self.__vao = GL.glGenVertexArrays(1)
        self.__vbo = 0
        self.__ebo = 0

        self.__pos_loc = pos_loc
        self.__color_loc = color_loc

        self.__updated = True
        self.__vertices = vertices
        self.__indeces = indices

        self.model_matrix = (
            model_matrix
            if model_matrix is not None
            else Matrix44.identity(dtype=np.float32)
        )

    def __update_buffers(self):
        if not self.__updated:
            return

        GL.glBindVertexArray(self.__vao)

        if self.__vbo != 0:
            GL.glDeleteBuffers(1, [self.__vbo])
        if self.__ebo != 0:
            GL.glDeleteBuffers(1, [self.__ebo])

        self.__vbo = GL.glGenBuffers(1)
        self.__ebo = GL.glGenBuffers(1)

        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.__vbo)
        GL.glBufferData(
            GL.GL_ARRAY_BUFFER,
            self.__vertices.nbytes,
            self.__vertices,
            GL.GL_STATIC_DRAW,
        )

        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self.__ebo)
        GL.glBufferData(
            GL.GL_ELEMENT_ARRAY_BUFFER,
            self.__indeces.nbytes,
            self.__indeces,
            GL.GL_STATIC_DRAW,
        )

        stride = 6 * 4  # x,y,z,r,g,b (6 floats, 4 bytes each)

        GL.glEnableVertexAttribArray(self.__pos_loc)
        GL.glVertexAttribPointer(
            self.__pos_loc, 3, GL.GL_FLOAT, False, stride, ctypes.c_void_p(0)
        )

        # Color: location 1
        GL.glEnableVertexAttribArray(self.__color_loc)
        GL.glVertexAttribPointer(
            self.__color_loc, 3, GL.GL_FLOAT, False, stride, ctypes.c_void_p(12)
        )

        GL.glBindVertexArray(0)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)

        self.__updated = True

    def draw(self, model_loc: int = -1):
        self.__update_buffers()

        GL.glBindVertexArray(self.__vao)

        if model_loc >= 0:
            GL.glUniformMatrix4fv(
                model_loc, 1, GL.GL_FALSE, self.model_matrix.flatten()
            )

        GL.glDrawElements(GL.GL_POINTS, 3, GL.GL_UNSIGNED_INT, ctypes.c_void_p(0))
        GL.glBindVertexArray(0)

    def free(self):
        GL.glDeleteVertexArrays(1, [self.__vao])

        if self.__vbo != 0:
            GL.glDeleteBuffers(1, [self.__vbo])
        self.__vbo = 0

        if self.__ebo != 0:
            GL.glDeleteBuffers(1, [self.__ebo])
        self.__ebo = 0


@final
class MainGl(OpenGLFrame):
    DEFAULT_GL_WIDTH = 640
    DEFAULT_GL_HEIGHT = 480
    __event_target: EventTarget
    __mesh: List[Optional[Mesh]]

    __view_matrix: Matrix44
    __projection_matrix: Matrix44

    __lock: RLock

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(
            master, width=MainGl.DEFAULT_GL_WIDTH, height=MainGl.DEFAULT_GL_HEIGHT
        )
        self.__event_target = EventTarget()
        self.shader_program = None

        self.__view_matrix = Matrix44.identity(dtype=np.float32)
        self.__projection_matrix = Matrix44.identity(dtype=np.float32)

        self.__lock = RLock()
        self.__mesh = []

    @override
    def initgl(self):
        # Set up viewport and clear color
        GL.glViewport(0, 0, self.width, self.height)
        GL.glClearColor(0.1, 0.1, 0.1, 1.0)
        GL.glPointSize(10)

        # Create shaders and program
        self.create_shader_program()

        # Create vertex buffer
        self.setup_triangle()

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
            gl_Position = projection * view * model * vec4(aPos, 1.0);
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

    def setup_triangle(self):
        # Triangle vertices with position (x, y, z) and color (r, g, b)
        vertices = np.array(
            [
                # Positions (x, y, z)    # Colors (r, g, b)
                -0.5,
                -0.5,
                0.0,
                1.0,
                0.0,
                0.0,  # Bottom-left, red
                0.5,
                -0.5,
                0.0,
                0.0,
                1.0,
                0.0,  # Bottom-right, green
                0.0,
                0.5,
                0.0,
                0.0,
                0.0,
                1.0,  # Top, blue
            ],
            dtype=np.float32,
        )
        indeces = np.array([0, 1, 2], dtype=np.uint32)
        self.__mesh.append(Mesh(vertices, indeces))

    @override
    def redraw(self):
        with self.__lock:
            # Clear the screen
            GL.glClear(GL.GL_COLOR_BUFFER_BIT)

            # Use shader program
            GL.glUseProgram(self.shader_program)

            model_loc = GL.glGetUniformLocation(self.shader_program, "model")
            view_loc = GL.glGetUniformLocation(self.shader_program, "view")
            proj_loc = GL.glGetUniformLocation(self.shader_program, "projection")

            GL.glUniformMatrix4fv(
                view_loc, 1, GL.GL_FALSE, self.__view_matrix.flatten()
            )
            GL.glUniformMatrix4fv(
                proj_loc, 1, GL.GL_FALSE, self.__projection_matrix.flatten()
            )

            for mesh in self.__mesh:
                if mesh is not None:
                    rotation = Matrix44.from_z_rotation(
                        math.radians(1), dtype=np.float32
                    )

                    mesh.model_matrix = rotation @ mesh.model_matrix
                    mesh.draw(model_loc)

            # Schedule the next redraw
            _ = self.after(16, self.tkExpose, None)  # ~60 FPS

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

    def remove_mesh(self, id: int) -> None:
        self.__mesh[id] = None

    def get_mesh(self, id: int) -> Optional[Mesh]:
        return self.__mesh[id]
