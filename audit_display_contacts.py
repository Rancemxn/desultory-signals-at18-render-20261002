"""Count planned contacts inside the actual effect mask; no image display/files.

This samples the renderer numerically at video frames, not native game scoring.
"""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np
import skia

from block_area import compose
from block_render import BlockRenderer, _dilate
from chart import load_chart
from handcam_motion import point_at
from handcam_blender import screen_rect


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path)
    parser.add_argument('chart', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--job', type=Path, help='Use the actual embedded chart viewport from a handcam job')
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    with zipfile.ZipFile(args.chart) as archive:
        chart = load_chart(archive.read('chart.json').decode('utf-8-sig'))
    width, height = screen_rect(json.loads(args.job.read_text()))[2:] if args.job else (1920, 1080)
    renderer = BlockRenderer(chart.block_areas, width, height, use_gpu=False)
    mw,mh = renderer.size
    ew,eh = renderer.effect_size
    violations, samples, frames = [], 0, 0
    for frame in range(9045):
        time = frame / 60
        contacts = [c for c in plan['contacts'] if c['start'] <= time < c['end']]
        rectangles = chart.block_areas.active(time)
        if not contacts or not rectangles:
            continue
        mask = renderer.active_mask(compose(rectangles), time)
        values = mask.toarray(colorType=skia.ColorType.kRGBA_8888_ColorType)[:, :, 0]
        expanded = _dilate(np.repeat(np.repeat(values, 2, axis=0), 2, axis=1))
        frames += 1
        for contact in contacts:
            samples += 1
            x, y = point_at(contact['points'], time)
            ix, iy = min(mw-1, max(0, int(x*mw))), min(mh-1, max(0, int(y*mh)))
            ex, ey = min(ew-1, max(0, int(x*ew))), min(eh-1, max(0, int(y*eh)))
            if values[iy, ix] or expanded[ey, ex]:
                violations.append(dict(time=time, notes=contact['note_ids'],
                    hand=contact['hand'], finger=contact['finger'],
                    fill=bool(values[iy, ix]), edge=bool(expanded[ey, ex]), xy=[x, y]))
        if frame % 600 == 0:
            print(f'DISPLAY {time:.1f}s samples={samples} overlaps={len(violations)}', flush=True)
    report = dict(boundary_mode=renderer.boundary_mode, fps=60, frames_with_contacts_and_blocks=frames,
        render_size=[width,height], mask_size=[mw,mh],
        contact_samples=samples, fill_samples=sum(v['fill'] for v in violations),
        fill_or_edge_samples=len(violations), passed=not violations, violations=violations,
        plan_sha256=hashlib.sha256(args.plan.read_bytes()).hexdigest(),
        scope='Planned contact centers; video frame samples; no visual inspection or native scoring')
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='violations'}))


if __name__ == '__main__':
    main()
