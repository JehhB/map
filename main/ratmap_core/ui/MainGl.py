# pyright: reportMissingTypeStubs=false, reportUnknownMemberType=false, reportUnknownArgumentType=false, reportUnknownVariableType=false, reportAny=false

import ctypes
import math
import time
import tkinter as tk
from typing import final

import numpy as np
from OpenGL import GL
from pyopengltk import OpenGLFrame
from typing_extensions import override

from ratmap_common.EventTarget import EventTarget


@final
class MainGl(OpenGLFrame):
    DEFAULT_GL_WIDTH = 640
    DEFAULT_GL_HEIGHT = 480
    __event_target: EventTarget

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(
            master, width=MainGl.DEFAULT_GL_WIDTH, height=MainGl.DEFAULT_GL_HEIGHT
        )
        self.__event_target = EventTarget()
        self.start_time = time.time()
        self.shader_program = None
        self.vao = None
        self.vbo = None

    @override
    def initgl(self):
        # Set up viewport and clear color
        GL.glViewport(0, 0, self.width, self.height)
        GL.glClearColor(0.1, 0.1, 0.1, 1.0)

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
        
        uniform float rotation;
        
        void main()
        {
            float s = sin(rotation);
            float c = cos(rotation);
            mat2 rotMatrix = mat2(c, -s, s, c);
            vec2 rotatedPos = rotMatrix * aPos.xy;
            gl_Position = vec4(rotatedPos.x, rotatedPos.y, aPos.z, 1.0);
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

        # Create VBO
        self.vbo = GL.glGenBuffers(1)

        # Bind and set vertex buffer
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
        GL.glBufferData(
            GL.GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL.GL_STATIC_DRAW
        )

        # Get attribute locations
        self.pos_attrib = GL.glGetAttribLocation(self.shader_program, "aPos")
        self.color_attrib = GL.glGetAttribLocation(self.shader_program, "aColor")

        # Unbind VBO
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)

    @override
    def redraw(self):
        # Clear the screen
        GL.glClear(GL.GL_COLOR_BUFFER_BIT)

        # Use shader program
        GL.glUseProgram(self.shader_program)

        # Calculate rotation angle based on time
        current_time = time.time() - self.start_time
        rotation_angle = current_time * math.pi / 2  # Rotate 90 degrees per second

        # Set uniform value for rotation
        rotation_location = GL.glGetUniformLocation(self.shader_program, "rotation")
        GL.glUniform1f(rotation_location, rotation_angle)

        # Bind buffer and set attributes
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)

        # Position attribute
        stride = 6 * 4  # 6 floats, 4 bytes each
        offset = 0
        GL.glEnableVertexAttribArray(self.pos_attrib)
        GL.glVertexAttribPointer(
            self.pos_attrib,
            3,
            GL.GL_FLOAT,
            GL.GL_FALSE,
            stride,
            ctypes.c_void_p(offset),
        )

        # Color attribute
        offset = 3 * 4  # Skip 3 floats, 4 bytes each
        GL.glEnableVertexAttribArray(self.color_attrib)
        GL.glVertexAttribPointer(
            self.color_attrib,
            3,
            GL.GL_FLOAT,
            GL.GL_FALSE,
            stride,
            ctypes.c_void_p(offset),
        )

        # Draw the triangle
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)

        # Disable attributes
        GL.glDisableVertexAttribArray(self.pos_attrib)
        GL.glDisableVertexAttribArray(self.color_attrib)

        # Unbind buffer
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)

        # Schedule the next redraw
        _ = self.after(16, self.tkExpose, None)  # ~60 FPS

    @property
    def event_target(self):
        return self.__event_target

    @override
    def destroy(self) -> None:
        # Clean up OpenGL resources
        if self.shader_program:
            GL.glDeleteProgram(self.shader_program)
        if self.vbo:
            GL.glDeleteBuffers(1, [self.vbo])

        self.__event_target.dispose()
        return super().destroy()


# Example usage
if __name__ == "__main__":
    root = tk.Tk()
    root.title("OpenGL Rotating Triangle")

    main_gl = MainGl(root)
    main_gl.pack(fill=tk.BOTH, expand=tk.YES)

    root.mainloop()
