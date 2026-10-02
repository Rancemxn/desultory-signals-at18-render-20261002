"""Blender check: bulk world vertices, screen clearance and shadows match scalar math."""
from pathlib import Path
import sys

import bpy
import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handcam_blender import mesh_world_coordinates, project_shadow_mesh, screen_mesh_clearance


def check_mesh(mesh, matrix):
    scalar = np.asarray([matrix @ v.co for v in mesh.vertices], dtype=np.float32).reshape(-1, 3)
    actual = mesh_world_coordinates(mesh, matrix)
    np.testing.assert_array_equal(actual, scalar)
    screen = (.28, .28 * 720 / 1280)
    expected = min((float(p[2]) for p in scalar if abs(float(p[0])) <= screen[0] / 2
                   and abs(float(p[1])) <= screen[1] / 2), default=float('inf'))
    assert screen_mesh_clearance(mesh, matrix, screen) == expected
    shadow = mesh.copy()
    try:
        project_shadow_mesh(shadow, matrix)
        values = np.empty((len(shadow.vertices), 3), dtype=np.float32)
        shadow.vertices.foreach_get('co', values.ravel())
        expected = np.asarray([(float(p[0]) + .35 * float(p[2]), float(p[1]) - .45 * float(p[2]), 0.)
                               for p in scalar], dtype=np.float32).reshape(-1, 3)
        np.testing.assert_array_equal(values, expected)
    finally:
        bpy.data.meshes.remove(shadow)


def check():
    # Real saved hand animation, including nontrivial parent transforms.
    scene = bpy.context.scene
    frames = sorted({f for f in (1, 17, 30, 59, 60, 150, 151, scene.frame_end)
                     if scene.frame_start <= f <= scene.frame_end})
    for frame in frames:
        bpy.context.scene.frame_set(frame)
        graph = bpy.context.evaluated_depsgraph_get()
        for obj in bpy.context.scene.objects:
            if obj.type == 'MESH':
                evaluated = obj.evaluated_get(graph)
                mesh = evaluated.to_mesh()
                try:
                    check_mesh(mesh, evaluated.matrix_world)
                finally:
                    evaluated.to_mesh_clear()
    mesh = bpy.data.meshes.new('bulk geometry boundary test')
    try:
        check_mesh(mesh, Matrix.Identity(4))
        vertices = [(0, 0, 0), (.14, 0, -.001), (-.14, 0, .002), (.140001, 0, -1)]
        vertices += [tuple(p) for p in np.random.default_rng(47).uniform(-.2, .2, (1000, 3))]
        mesh.from_pydata(vertices, [], [])
        for matrix in (Matrix.Identity(4), Matrix.Translation(Vector((.04, -.07, .01)))
                       @ Matrix.Rotation(.73, 4, 'Z') @ Matrix.Diagonal((.27, -.31, .4, 1.)),
                       Matrix.Translation(Vector((5., 5., 0.)))):
            check_mesh(mesh, matrix)
    finally:
        bpy.data.meshes.remove(mesh)
    print('Bulk geometry matches scalar world coordinates, clearance and shadow vertices exactly')


if __name__ == '__main__':
    check()
