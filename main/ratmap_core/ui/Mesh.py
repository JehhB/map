# pyright: reportMissingTypeStubs=false, reportUnknownMemberType=false, reportUnknownArgumentType=false, reportUnknownVariableType=false, reportAny=false

import ctypes
from threading import RLock
from typing import Optional

import numpy as np
import open3d as o3d
from numpy.typing import NDArray
from OpenGL import GL
from pyrr import Matrix44
from scipy.spatial import cKDTree
from skimage import measure


class Mesh:
    model_matrix: Matrix44
    type: int

    def __init__(
        self,
        vertices: Optional[NDArray[np.float32]] = None,
        indices: Optional[NDArray[np.uint32]] = None,
        pos_loc: int = 0,
        color_loc: int = 1,
        model_matrix: Optional[Matrix44] = None,
        type: int = GL.GL_POINTS,
    ):
        self.__lock = RLock()
        self.type = type

        self.__vbo = 0
        self.__ebo = 0

        self.__pos_loc = pos_loc
        self.__color_loc = color_loc

        self.__updated = True
        self.__vertices = (
            vertices if vertices is not None else np.array([], dtype=np.float32)
        )
        self.__indices = (
            indices
            if indices is not None
            else np.arange(self.__vertices.size, dtype=np.uint32)
        )

        self.model_matrix = (
            model_matrix
            if model_matrix is not None
            else Matrix44.identity(dtype=np.float32)
        )

    def __update_buffers(self):
        if not self.__updated:
            return

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
            self.__indices.nbytes,
            self.__indices,
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

        self.__updated = False

    def draw(self, model_loc: int = -1):
        with self.__lock:
            self.__update_buffers()

            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.__vbo)
            GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self.__ebo)

            # Set up vertex attributes
            stride = 6 * 4  # x,y,z,r,g,b (6 floats, 4 bytes each)
            GL.glEnableVertexAttribArray(self.__pos_loc)
            GL.glVertexAttribPointer(
                self.__pos_loc, 3, GL.GL_FLOAT, False, stride, ctypes.c_void_p(0)
            )
            GL.glEnableVertexAttribArray(self.__color_loc)
            GL.glVertexAttribPointer(
                self.__color_loc, 3, GL.GL_FLOAT, False, stride, ctypes.c_void_p(12)
            )

            # Draw
            if model_loc >= 0:
                GL.glUniformMatrix4fv(
                    model_loc, 1, GL.GL_FALSE, self.model_matrix.flatten()
                )

            GL.glDrawElements(
                self.type,
                self.__indices.size,
                GL.GL_UNSIGNED_INT,
                ctypes.c_void_p(0),
            )

            # Clean up
            GL.glDisableVertexAttribArray(self.__pos_loc)
            GL.glDisableVertexAttribArray(self.__color_loc)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
            GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0)

    def free(self):
        if self.__vbo != 0:
            GL.glDeleteBuffers(1, [self.__vbo])
        self.__vbo = 0

        if self.__ebo != 0:
            GL.glDeleteBuffers(1, [self.__ebo])
        self.__ebo = 0

    @property
    def vertices(self):
        return self.__vertices

    @vertices.setter
    def vertices(self, vertices: NDArray[np.float32]):
        with self.__lock:
            self.__vertices = vertices
            self.__updated = True

    @property
    def indices(self):
        return self.__indices

    @indices.setter
    def indices(self, indices: NDArray[np.uint32]):
        with self.__lock:
            self.__indices = indices
            self.__updated = True

    @staticmethod
    def marching_cubes(vertices: NDArray[np.float32], voxel_size: float = 0.05):
        min_bounds = np.min(vertices[:, :3], axis=0)
        max_bounds = np.max(vertices[:, :3], axis=0)
        margin = voxel_size * 2
        min_bounds -= margin
        max_bounds += margin

        dims = np.ceil((max_bounds - min_bounds) / voxel_size).astype(int)

        grid = np.zeros(dims, dtype=np.float32)
        color_grid = np.zeros((*dims, 3), dtype=np.float32)

        indices = np.floor((vertices[:, :3] - min_bounds) / voxel_size).astype(int)

        valid_mask = np.all((indices >= 0) & (indices < dims), axis=1)
        valid_indices = indices[valid_mask]
        valid_colors = vertices[valid_mask, 3:6]

        for i in range(len(valid_indices)):
            idx = tuple(valid_indices[i])
            grid[idx] = 1.0
            color_grid[idx] = valid_colors[i]

        verts, faces, _, _ = measure.marching_cubes(grid, level=0.5)

        verts_world = np.zeros_like(verts)
        verts_world = verts * voxel_size + min_bounds

        idx = np.clip(verts.astype(int), 0, dims - 1)
        vertex_colors = color_grid[idx[:, 0], idx[:, 1], idx[:, 2]]

        vertices = np.hstack([verts_world, vertex_colors], dtype=np.float32)

        return vertices, faces.astype(np.uint32)

    @staticmethod
    def ball_pivoting(vertices: NDArray[np.float32]):
        points = vertices[:, :3]
        colors = vertices[:, 3:6]

        print("creating point cloud")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        pcd_tree = cKDTree(points)

        print("computing normals")
        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        pcd.estimate_normals(search_param=search_param)

        print("meshing")
        radii = np.array([0.01, 0.05, 0.075])
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(radii)
        )

        print("preparing")
        verts = np.asarray(mesh.vertices, dtype=np.float32)
        _, idx = pcd_tree.query(verts, k=1)
        colors = np.asarray(pcd.colors, dtype=np.float32)[idx]
        faces = np.asarray(mesh.triangles, dtype=np.uint32)
        vertices = np.hstack([verts, colors], dtype=np.float32)
        return vertices, faces
