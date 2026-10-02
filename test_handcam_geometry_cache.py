"""Blender integration: cached subdivision must preserve the full deforming mesh."""
from pathlib import Path
import sys

import bpy
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
from handcam_blender import FINGERS, cache_static_geometry, restore_static_geometry


def check():
    arms={side:bpy.data.objects[name] for side,name in ((-1,'Hand_Left'),(1,'Hand_Right'))}
    meshes={side:next(o for o in bpy.context.scene.objects if o.type=='MESH' and o.parent==arm) for side,arm in arms.items()}
    originals={side:obj.data for side,obj in meshes.items()}
    for obj in meshes.values():
        for modifier in obj.modifiers:
            if modifier.type=='MULTIRES':
                modifier.levels=modifier.render_levels=1
            if modifier.type=='ARMATURE':
                modifier.use_deform_preserve_volume=True
    def pose(amount):
        for side,arm in arms.items():
            arm.rotation_euler.z=side*amount*.25
            for finger,chain in FINGERS.items():
                for joint,name in enumerate(chain):
                    bone=arm.pose.bones[name]
                    bone.rotation_mode='XYZ'
                    bone.rotation_euler=(amount*(-.3-.25*joint),0.,side*amount*.15 if joint==0 else 0.)
        bpy.context.view_layer.update()
    def points():
        result=[]
        graph=bpy.context.evaluated_depsgraph_get()
        for obj in meshes.values():
            evaluated=obj.evaluated_get(graph)
            mesh=evaluated.to_mesh()
            xyz=np.empty(len(mesh.vertices)*3,dtype=np.float32)
            mesh.vertices.foreach_get('co',xyz)
            transform=np.array(evaluated.matrix_world)
            result.append(xyz.reshape(-1,3)@transform[:3,:3].T+transform[:3,3])
            evaluated.to_mesh_clear()
        return np.concatenate(result)
    amounts=(0.,.25,.65,1.,.1)
    reference=[]
    for amount in amounts:
        pose(amount)
        reference.append(points())
    cached=cache_static_geometry(meshes)
    assert len(cached)==2
    maximum=0.
    for amount,expected in zip(amounts,reference):
        pose(amount)
        maximum=max(maximum,float(np.linalg.norm(points()-expected,axis=1).max()))
    assert maximum<1e-7, maximum
    restore_static_geometry(cached)
    assert all(obj.data is originals[side] for side,obj in meshes.items())
    assert all(m.show_viewport and m.show_render for obj in meshes.values() for m in obj.modifiers)
    assert np.max(np.abs(points()-reference[-1]))<1e-7
    # Animated mesh settings are deliberately left on the ordinary modifier path.
    obj=meshes[-1]
    obj.animation_data_create()
    assert cache_static_geometry({-1:obj})==[]
    obj.animation_data_clear()
    print(f'Static geometry cache matches full meshes at five poses; maximum difference {maximum*1000:.9f} mm')


if __name__=='__main__':
    check()
