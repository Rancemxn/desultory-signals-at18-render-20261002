import json
from pathlib import Path
import sys
import bpy

out = Path(sys.argv[sys.argv.index('--') + 1])
job = json.loads((out / 'job.json').read_text())
times = json.loads((out / 'review-times.json').read_text())
scene = bpy.context.scene
scene.render.resolution_x = 1280
scene.render.resolution_y = 720
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.color_mode = 'RGBA'
scene.render.film_transparent = True
(out / 'review-hands').mkdir(exist_ok=True)
boundaries = []
for frame in (1, job['frames']):
    scene.frame_set(frame)
    boundaries.append({'time': job['start'] + (frame - 1) / job['fps'], 'rigs': {
        arm.name: {'matrix': [list(row) for row in arm.matrix_world],
                   'bones': {bone.name: {'head': list(arm.matrix_world @ bone.head),
                                          'tail': list(arm.matrix_world @ bone.tail)}
                             for bone in arm.pose.bones}}
        for arm in scene.objects if arm.type == 'ARMATURE'}})
(out / 'boundary-poses.json').write_text(json.dumps(boundaries))
for index, t in enumerate(times):
    frame = (t - job['start']) * job['fps'] + 1
    scene.frame_set(int(frame), subframe=frame % 1)
    scene.render.filepath = str(out / 'review-hands' / f'{index}.png')
    bpy.ops.render.render(write_still=True)
