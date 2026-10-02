"""Export a full chart as a 720p30 handcam development movie using local assets."""
from pathlib import Path
import sys

from handcam import main


if __name__ == '__main__':
    root = Path(__file__).resolve().parent
    raise SystemExit(main([
        '--model', str(root / 'refer/Hands + armature.blend'),
        '--resources', str(root / 'refer/0bd4a3b2ed2af56f98a0ed05b29dabf8.zip'),
        '--output', str(root / 'build/handcam-v5-dev'),
        '--full', '--stream', '--width', '1280', '--height', '720', '--fps', '30',
        *sys.argv[1:],
    ]))
