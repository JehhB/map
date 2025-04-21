# pyright: reportMissingTypeStubs=false, reportUnknownMemberType=false, reportUnknownArgumentType=false, reportUnknownVariableType=false, reportAny=false

import ctypes
from typing import Optional

import numpy as np
from numpy.typing import NDArray
from OpenGL import GL
from pyrr import Matrix44


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
