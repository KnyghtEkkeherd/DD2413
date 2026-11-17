#!/usr/bin/env python3
"""Compare the sounds in config/reactions.json with the audio files on Misty.

    python3 scripts/check_misty_sounds.py 192.168.1.237
"""
import json
import sys
from pathlib import Path

import requests

ip = sys.argv[1] if len(sys.argv) > 1 else sys.exit(__doc__)
data = json.load(open(Path(__file__).resolve().parents[1] / 'src/misty_ros/config/reactions.json'))
r = requests.get(f'http://{ip}/api/audio/list', timeout=5)
r.raise_for_status()
items = r.json().get('result') or []
on_robot = {(i.get('name') or i.get('Name')) if isinstance(i, dict) else str(i) for i in items}
configured = sorted({f for r in data['reactions'] for f in r.get('sounds', [])} |
                    {st['sound'] for r in data['reactions'] for st in r.get('steps', []) if 'sound' in st})
missing = [f for f in configured if f not in on_robot]
print(f'{len(on_robot)} files on Misty; {len(configured)} used in reactions.json')
print('MISSING on robot:', ', '.join(missing) if missing else 'none')
print('\nAll files on Misty:')
for name in sorted(on_robot):
    print(('  * ' if name in configured else '    ') + name)
