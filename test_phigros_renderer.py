"""Run with the project Python; generated fixtures need no reference assets or Blender."""
import json
import math
from pathlib import Path
import tempfile
import zipfile
import subprocess
import wave

import numpy as np
import skia

from bamboo import BambooShoot
from basis import Note, NoteType, VisualNote
from chart import load_chart
from handcam_blender import composite_command, screen_rect
from handcam import prepare_resources, render_screen
from handcam_render import chunk_command, completed_chunk
from phigros_renderer import PhigrosRenderer, background_image, mix_audio, read_info


def check():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        pack = root / 'respack'
        pack.mkdir()
        (pack / 'info.yml').write_text('holdAtlas: [2, 2]\nholdAtlasMH: [2, 2]\nhitFx: [2, 2]\nname: "Test: skin"\n')
        assert read_info(pack / 'info.yml')['name'] == 'Test: skin'
        for name, color in [('click', skia.ColorCYAN), ('drag', skia.ColorYELLOW), ('flick', skia.ColorRED),
                            ('hold', skia.ColorCYAN), ('hit_fx', skia.ColorWHITE)]:
            for suffix in (('', '_mh') if name != 'hit_fx' else ('',)):
                surface = skia.Surface(16, 16)
                surface.getCanvas().clear(color)
                surface.makeImageSnapshot().save(str(pack / f'{name}{suffix}.png'))
        chart = load_chart(json.dumps(dict(formatVersion=3, offset=.125, judgeLineList=[dict(bpm=120)])))
        line = chart.lines[0]
        line.floor = BambooShoot(0.)
        tap = Note(NoteType.TAP, 1., 0., 0j)
        drag = Note(NoteType.DRAG, 1., 0., 2+0j)
        hold = Note(NoteType.HOLD, 1.5, 1., -2+0j)
        fake = Note(NoteType.FLICK, 1., 0., 3+0j)
        line.visual_notes = [VisualNote(tap), VisualNote(drag, position_x=2),
                             VisualNote(hold, position_x=-2, speed=1), VisualNote(fake, is_fake=True, position_x=3)]
        line.notes = [tap, drag, hold]
        renderer = PhigrosRenderer(chart, 640, 360, root, 'Test', 'IN 14', 4)
        assert len(renderer.multi) == 2 and id(line.visual_notes[-1]) not in renderer.multi
        assert renderer.stats(.999) == (0, 0)
        assert renderer.stats(1.) == (2, 666667)
        assert renderer.stats(2.499) == (2, 666667) and renderer.stats(2.5) == (3, 1000000)
        assert renderer.hits == [(1.125, int(NoteType.TAP)), (1.125, int(NoteType.DRAG)), (1.625, int(NoteType.HOLD))]
        surface = skia.Surface(640, 360)
        def pixels(t):
            surface.getCanvas().clear(skia.ColorBLACK)
            renderer.draw(surface.getCanvas(), t)
            return surface.makeImageSnapshot().toarray().copy()
        before, hit, held = pixels(.98), pixels(1.08), pixels(2.)
        assert not np.array_equal(before, hit) and not np.array_equal(hit, held)
        assert np.array_equal(hit, pixels(1.08))  # Seeking does not accumulate stale effects/stats.
        from copy import copy
        ui = copy(line)
        ui.opacity = BambooShoot(0.)
        renderer.ui_lines['score'] = ui
        assert not np.array_equal(hit[:55, 480:], pixels(1.08)[:55, 480:])
        del renderer.ui_lines['score']
        # Above and below notes must be placed on opposite sides of a rotated line.
        line.angle = BambooShoot(math.pi / 2)
        normal = VisualNote(tap, floor=2.)
        below = VisualNote(tap, floor=2., above=False)
        assert line.note_state(.5, normal).head.real > line.note_state(.5, below).head.real
        background = skia.Surface(200, 100)
        background.getCanvas().clear(skia.ColorRED)
        background.makeImageSnapshot().save(str(root / 'cover.png'))
        image = background_image(root / 'cover.png', 300, 300, .2, 15).toarray()
        assert np.array_equal(image[0, 0], image[-1, -1]) and image[0, 0, :3].max() > 150
        # Tiny PCM fixtures verify exact hit offsets, Hold head sound, and pre-clip tails.
        def audio(path, values):
            with wave.open(str(path), 'wb') as stream:
                stream.setparams((2, 2, 48000, 0, 'NONE', 'not compressed'))
                stream.writeframes(values.astype('<i2').tobytes())
        audio(root / 'music.wav', np.zeros((9600, 2)))
        pulse = np.zeros((4800, 2)); pulse[0] = 10000; pulse[2400] = 5000
        for name in ('click', 'drag', 'flick'):
            audio(root / f'{name}.ogg', pulse)
        stats = mix_audio(root / 'music.wav', root, [(0., NoteType.TAP), (.075, NoteType.HOLD)],
                          .025, .1, .5, root / 'mixed.wav')
        with wave.open(str(root / 'mixed.wav'), 'rb') as stream:
            pcm = np.frombuffer(stream.readframes(4800), dtype='<i2').reshape(-1, 2)
        assert stats['hit_sounds'] == 2 and stats['samples'] == 4800
        assert abs(int(pcm[1200, 0]) - 2500) <= 1 and abs(int(pcm[2400, 0]) - 5000) <= 1
        assert np.count_nonzero(pcm[:, 0]) == 2
        job = dict(width=720, height=480, fps=30, screen=[.28, .1575], view_width=.28/.82, camera_y=-.015,
                   output=str(root), background=str(root/'cover.png'), audio=str(root/'music.wav'),
                   mixed_audio=str(root/'mixed.wav'), start=84., duration=.1)
        assert screen_rect(job) == (65, 42, 590, 332)
        command = composite_command(job)
        assert command[command.index('-ss') + 1] == '0'
        assert '[4:v][0:v]overlay=65:42' in command[command.index('-filter_complex') + 1]
        command = composite_command(job, 1)
        assert '[3:v][0:v]overlay=65:42' in command[command.index('-filter_complex') + 1]
        streamed = dict(job, screen_video=str(root / 'screen.mp4'))
        command = composite_command(streamed)
        assert command[command.index('-i') + 1] == str(root / 'screen.mp4')
        try:
            screen_rect(dict(job, height=200))
        except ValueError:
            pass
        else:
            raise AssertionError('Cropped tablet accepted')
        # Resource-pack atlas options must render through the same Hold path.
        (pack / 'info.yml').write_text('holdAtlas: [2, 2]\nholdAtlasMH: [2, 2]\nhitFx: [2, 2]\nholdRepeat: true\nholdCompact: true\nholdKeepHead: true\n')
        tiled = PhigrosRenderer(chart, 640, 360, root)
        tiled.draw(surface.getCanvas(), 1.8)
        assert tiled.hold_bodies and surface.makeImageSnapshot() is not None
        del tiled
        del renderer  # Skia images keep the source files mapped on Windows.
        archive = root / 'skin.zip'
        with zipfile.ZipFile(archive, 'w') as z:
            for p in pack.iterdir():
                z.write(p, p.name)
        imported = prepare_resources(archive, root / 'extracted')
        assert imported == (root / 'extracted').resolve()
        assert (imported / 'hold.png').read_bytes() == (pack / 'hold.png').read_bytes()
        streaming_job = dict(streamed, resources=str(imported), title='Test', level='IN 14', music_duration=4.,
                             frames=3, start=1., background_dim=.45)
        result = render_screen(chart, root / 'cover.png', root / 'screen', streaming_job)
        probe = json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-show_streams','-of','json',
                                                   str(root / 'screen.mp4')], text=True))['streams'][0]
        assert (probe['width'], probe['height'], probe['nb_read_frames']) == (590, 332, '3')
        del result
        # MKV's millisecond timestamps must retain the final frame when overlaid at 60 fps.
        for name in ('hands', 'shadows'):
            subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'lavfi', '-i',
                            'color=black@0:s=64x64:r=60,format=bgra', '-frames:v', '12',
                            '-c:v', 'ffv1', str(root / f'{name}.mkv')], check=True)
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'lavfi', '-i', 'color=blue:s=262x148:r=60',
                        '-frames:v', '12', '-c:v', 'libx264', '-preset', 'ultrafast', str(root / 'chart60.mp4')], check=True)
        background_image(root / 'cover.png', 320, 240, .2, 5).save(str(root / 'background-small.png'))
        small = dict(job, width=320, height=240, fps=60, screen_video=str(root / 'chart60.mp4'), background=str(root / 'background-small.png'))
        video = root / 'composite.mp4'
        subprocess.run(chunk_command(small, 1, 12, 0, 0, root / 'hands.mkv', root / 'shadows.mkv', video),
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        assert completed_chunk(video, small, 12)
        with zipfile.ZipFile(root / 'bad.zip', 'w') as z:
            z.writestr('../outside', 'bad')
        try:
            prepare_resources(root / 'bad.zip', root / 'bad-pack')
        except ValueError:
            pass
        else:
            raise AssertionError('Resource ZIP traversal accepted')
    print('Phigros renderer checks passed')


if __name__ == '__main__':
    check()
