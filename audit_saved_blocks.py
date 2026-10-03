"""Compare measured saved-skin projections with block geometry, without images."""
import argparse,json,sys,zipfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from shapely.geometry import Point
from chart import load_chart
from block_area import compose
from contact_refinement import ContactGeometry
from handcam_blender import screen_rect

p=argparse.ArgumentParser()
p.add_argument('job',type=Path);p.add_argument('numeric',type=Path);p.add_argument('output',type=Path)
p.add_argument('--chart',type=Path,required=True)
a=p.parse_args()
job=json.loads(a.job.read_text());numeric=json.loads(a.numeric.read_text())
with zipfile.ZipFile(a.chart) as z:
 chart=load_chart(z.read('chart.json').decode())
g=ContactGeometry(chart);sx,sy,sw,sh=screen_rect(job)
violations={k:[] for k in ('native_forbidden','authored_fill','display_guard')}
for sample in numeric['projected_contacts']:
 t=sample['time']-chart.offset
 u,v=(sample['projected_pixel'][0]-sx)/sw,(sample['projected_pixel'][1]-sy)/sh
 point=Point(u,v)
 checks={'native_forbidden':chart.block_areas.contains(t,u*chart.width,v*chart.height),
         'authored_fill':g.normalized(compose(chart.block_areas.active(t))).covers(point),
         'display_guard':g.blocked(t).covers(point)}
 for name,hit in checks.items():
  if hit:violations[name].append(dict(time=sample['time'],hand=sample['hand'],finger=sample['finger'],notes=sample['notes'],uv=[u,v],pad_height_mm=sample['pad_world'][2]*1000,error_px=sample['error_px']))
result=dict(source_job=str(a.job.resolve()),source_numeric=str(a.numeric.resolve()),
            inspection='saved mesh and camera coordinates; no image reading',
            sample_times=numeric['mesh_check_times'],active_contact_samples=len(numeric['projected_contacts']),
            maximum_projection_error_px=max((s['error_px'] for s in numeric['projected_contacts']),default=0.),
            violation_counts={k:len(v) for k,v in violations.items()},violations=violations,
            native_ap_validated=False,scope='sampled active fingertip centroid, not the full finger silhouette or continuous-time proof')
a.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('sample_times','violations')},ensure_ascii=False))
