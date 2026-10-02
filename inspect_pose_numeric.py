"""Inspect saved Blender animation using coordinates only; never render images."""
import argparse
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree

sys.path.insert(0,str(Path(__file__).resolve().parent))
from handcam_blender import FINGERS, camera_pixel, configure_camera, screen_rect, skin_pads
from handcam_motion import point_at


def inspect(job,rig,*,start=None,end=None,mesh_times=()):
    scene = bpy.context.scene
    configure_camera(scene,job)
    arms = {side:bpy.data.objects[name] for side,name in ((-1,'Hand_Left'),(1,'Hand_Right'))}
    begin = job['start'] if start is None else start
    finish = job['start']+job['duration'] if end is None else end
    previous = {}
    wrist_steps,joint_steps,crossings = [],[],[]
    boundaries = []
    first = max(1,round((begin-job['start'])*job['fps'])+1)
    last = min(job['frames'],round((finish-job['start'])*job['fps']))
    for frame in range(first,last+1):
        scene.frame_set(frame)
        t = job['start']+(frame-1)/job['fps']
        if frame in (first,last):
            boundaries.append({'time':t,'rigs':{arm.name:{'matrix':[list(row) for row in arm.matrix_world],
                'bones':{bone.name:{'head':list(arm.matrix_world@bone.head),'tail':list(arm.matrix_world@bone.tail)}
                         for bone in arm.pose.bones}} for arm in arms.values()}})
        for side,arm in arms.items():
            wrist = arm.matrix_world@arm.pose.bones['Bone.016'].head
            old = previous.get((side,'wrist'))
            if old:
                delta = wrist-old
                wrist_steps.append({'time':t,'hand':side,'step_mm':delta.length*1000,
                    'lateral_speed_mps':math.hypot(delta.x,delta.y)*job['fps']})
            previous[side,'wrist'] = wrist.copy()
            for bone in arm.pose.bones:
                rotation = bone.matrix.to_quaternion()
                rotation.normalize()
                old = previous.get((side,bone.name))
                if old:
                    angle = math.degrees(2*math.acos(min(1.,abs(rotation.dot(old)))))
                    joint_steps.append({'time':t,'hand':side,'bone':bone.name,'step_degrees':angle})
                previous[side,bone.name] = rotation.copy()
            for a,b in (('index','middle'),('middle','ring'),('ring','little')):
                first_tip = arm.pose.bones[FINGERS[a][-1]].tail
                second_tip = arm.pose.bones[FINGERS[b][-1]].tail
                expected = arm.data.bones[FINGERS[b][0]].head_local.x-arm.data.bones[FINGERS[a][0]].head_local.x
                reverse = -(second_tip.x-first_tip.x)*(1 if expected>0 else -1)*max(abs(v) for v in arm.scale)
                if reverse>.002:
                    crossings.append({'time':t,'hand':side,'fingers':[a,b],'reverse_mm':reverse*1000})
    projected,mesh_intersections = [],[]
    if mesh_times:
        indices = {(int(k.split(':')[0]),k.split(':')[1]):v for k,v in rig['pad_vertices'].items()}
        meshes = {side:next(o for o in scene.objects if o.type=='MESH' and o.parent==arm) for side,arm in arms.items()}
        x,y,w,h = screen_rect(job)
        for t in mesh_times:
            frame = (t-job['start'])*job['fps']+1
            scene.frame_set(math.floor(frame),subframe=frame%1)
            pads = skin_pads(meshes,indices)
            for c in job['contacts']:
                if not c['start']<=t<c['end']:
                    continue
                key = (-1 if c['hand']=='left' else 1,c['finger'])
                uv = point_at(c['points'],t)
                pixel = camera_pixel(scene,pads[key])
                projected.append({'time':t,'hand':key[0],'finger':key[1],'notes':c['note_ids'],
                    'pad_world':list(pads[key]),'projected_pixel':list(pixel),
                    'target_pixel':[x+w*uv[0],y+h*uv[1]],'error_px':math.dist(pixel,(x+w*uv[0],y+h*uv[1]))})
            graph = bpy.context.evaluated_depsgraph_get()
            trees = {}
            for side,obj in meshes.items():
                evaluated = obj.evaluated_get(graph)
                mesh = evaluated.to_mesh()
                try:
                    vertices = [evaluated.matrix_world@v.co for v in mesh.vertices]
                    polygons = [tuple(p.vertices) for p in mesh.polygons]
                    trees[side] = BVHTree.FromPolygons(vertices,polygons)
                    owner = {obj.vertex_groups[name].index:finger for finger,chain in FINGERS.items()
                             for name in chain if name in obj.vertex_groups}
                    vertex_owner = []
                    for v in mesh.vertices:
                        group = max(v.groups,key=lambda g:g.weight).group if v.groups else None
                        vertex_owner.append(owner.get(group))
                    fingers = {}
                    for finger in FINGERS:
                        faces = [poly for poly in polygons if all(vertex_owner[i]==finger for i in poly)]
                        if faces:
                            fingers[finger] = BVHTree.FromPolygons(vertices,faces)
                    names = list(fingers)
                    for i,a in enumerate(names):
                        for b in names[i+1:]:
                            count = len(fingers[a].overlap(fingers[b]))
                            if count:
                                mesh_intersections.append({'time':t,'parts':[f'{side}:{a}',f'{side}:{b}'],
                                                           'intersecting_polygon_pairs':count})
                finally:
                    evaluated.to_mesh_clear()
            count = len(trees[-1].overlap(trees[1]))
            if count:
                mesh_intersections.append({'time':t,'parts':['left_hand','right_hand'],
                                           'intersecting_polygon_pairs':count})
    return {'frames':last-first+1,'worst_wrist_steps':sorted(wrist_steps,key=lambda x:-x['lateral_speed_mps'])[:50],
        'worst_joint_steps':sorted(joint_steps,key=lambda x:-x['step_degrees'])[:50],
        'finger_order_violations':crossings,'projected_contacts':projected,'boundary_poses':boundaries,
        'mesh_intersections':mesh_intersections,'mesh_check_times':list(mesh_times),
        'self_mesh_scope':'pairs of fingers; faces wholly dominated by a finger chain, excluding shared palm webbing'}


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('job',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--start',type=float)
    parser.add_argument('--end',type=float)
    parser.add_argument('--mesh-times',nargs='*',type=float,default=[])
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    job = json.loads(args.job.read_text())
    rig = json.loads(args.job.with_name('rig.json').read_text())
    report = inspect(job,rig,start=args.start,end=args.end,mesh_times=args.mesh_times)
    args.output.write_text(json.dumps(report,indent=2))
    print(json.dumps({'frames':report['frames'],'worst_wrists':report['worst_wrist_steps'][:3],
        'worst_joints':report['worst_joint_steps'][:3],'order_violations':len(report['finger_order_violations']),
        'contact_projection':sorted(report['projected_contacts'],key=lambda x:-x['error_px'])[:6],
        'mesh_intersection_samples':len(report['mesh_intersections'])}),flush=True)
