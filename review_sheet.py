import json
from pathlib import Path
import sys
import skia
from PIL import Image, ImageDraw
from chart import load_chart
from handcam_blender import screen_rect
from phigros_renderer import PhigrosRenderer, background_image

out = Path(sys.argv[1])
job = json.loads((out / 'job.json').read_text())
times = json.loads((out / 'review-times.json').read_text())
chartpath = next((out / 'input').rglob('chart.json'))
chart = load_chart(chartpath.read_text(encoding='utf-8-sig'), source=chartpath)
job.update(width=1280, height=720)
x, y, w, h = screen_rect(job)
renderer = PhigrosRenderer(chart, w, h, job['resources'], job['title'], job['level'], job['music_duration'])
bg = background_image(Path('inputs/illustration.jpg'), w, h, job['background_dim'], h * .015)
sheet = Image.new('RGB', (1280, ((len(times) + 1) // 2) * 384), '#111827')
draw = ImageDraw.Draw(sheet)
for index, t in enumerate(times):
    surface = skia.Surface(w, h)
    canvas = surface.getCanvas()
    canvas.drawImage(bg, 0, 0)
    renderer.draw(canvas, t - chart.offset)
    import io
    screen = Image.open(io.BytesIO(bytes(surface.makeImageSnapshot().encodeToData())))
    frame = Image.new('RGBA', (1280, 720), '#182332')
    frame.paste(screen, (x, y))
    frame.alpha_composite(Image.open(out / 'review-hands' / f'{index}.png').convert('RGBA'))
    col, row = index % 2, index // 2
    sheet.paste(frame.convert('RGB').resize((640, 360)), (col * 640, row * 384 + 24))
    draw.text((col * 640 + 12, row * 384 + 5), f'{t:.3f} s  |  REVIEWED BAKED POSE', fill='white')
sheet.save(out / 'review.jpg', quality=87)
