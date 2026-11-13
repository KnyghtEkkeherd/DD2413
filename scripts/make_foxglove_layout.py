#!/usr/bin/env python3
"""Generate foxglove/misty_wizard_layout.json.

Import it in Foxglove via Layouts -> Import from file (not drag-and-drop).

Re-run after changing buttons here, speech categories in speech_bank.yaml,
or sounds in sounds.yaml.
"""
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEECH = list(yaml.safe_load(open(ROOT / 'src/misty_ros/config/speech_bank.yaml')))
SOUNDS = yaml.safe_load(open(ROOT / 'src/misty_ros/config/sounds.yaml'))
GREEN, RED, BLUE, GREY, AMBER, DARK, PURPLE, TEAL = (
    '#2e9e5b', '#d33b3b', '#3b7dd8', '#6b6b7b', '#d8902b', '#8a1c1c', '#7a4fc9', '#1f8a8a')


class Layout:
    def __init__(self):
        self.configs = {}

    def panel(self, kind, cfg):
        pid = f'{kind}!{kind.lower()}{len(self.configs)}'
        self.configs[pid] = cfg
        return pid

    def split(self, items, direction, sizes=None):
        if len(items) == 1:
            return items[0]
        sizes = sizes or [1] * len(items)
        return {'first': items[0], 'second': self.split(items[1:], direction, sizes[1:]),
                'direction': direction, 'splitPercentage': sizes[0] / sum(sizes) * 100}

    def grid(self, buttons, per_row):
        rows = [buttons[i:i + per_row] for i in range(0, len(buttons), per_row)]
        return self.split([self.split(r, 'row') for r in rows], 'column')

    def button(self, data, color, label=None, editable=False):
        return self.panel('Publish', {
            'topicName': '/wizard/event', 'datatype': 'std_msgs/msg/String',
            'buttonText': label or data.replace('_', ' '), 'buttonTooltip': f'Send {data}',
            'buttonColor': color, 'advancedView': editable, 'value': json.dumps({'data': data})})

    def status(self, topic, size=16, title=None):
        return self.panel('RawMessages', {
            'foxglovePanelTitle': title or topic,
            'topicPath': f'{topic}.data', 'diffEnabled': False, 'diffMethod': 'custom',
            'diffTopicPath': '', 'showFullMessageForDiff': False, 'fontSize': size})

    def teleop(self, topic, title, rate=5):
        return self.panel('Teleop', {
            'foxglovePanelTitle': title, 'topic': topic, 'publishRate': rate,
            'upButton': {'field': 'linear-x', 'value': 1}, 'downButton': {'field': 'linear-x', 'value': -1},
            'leftButton': {'field': 'angular-z', 'value': 1}, 'rightButton': {'field': 'angular-z', 'value': -1}})

    def camera(self):
        return self.panel('Image', {'foxglovePanelTitle': 'Misty camera',
                                    'imageMode': {'imageTopic': '/misty/camera/image/compressed'}})

    def params(self):
        return self.panel('Parameters', {'title': 'Session: set participant_id + condition, then START'})

    def write(self, tree, name):
        out = ROOT / 'foxglove' / name
        out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps({'configById': self.configs, 'globalVariables': {}, 'userNodes': {},
                                   'playbackConfig': {'speed': 1}, 'layout': tree}, indent=2) + '\n')
        print(f'wrote {out} ({len(self.configs)} panels)')


def main():
    L = Layout()
    b = L.button
    speech_colors = {'CONFUSED': AMBER, 'FRUSTRATED': RED, 'LOST': PURPLE, 'ASK_HELP': PURPLE,
                     'THANKS': GREEN, 'RECOVER': GREEN, 'ARRIVE': GREEN, 'DISAGREE': RED}
    speech = [b(f'SAY:{c}', speech_colors.get(c, BLUE), c.replace('_', ' ').title()) for c in SPEECH]
    speech += [b('STOP_TALKING', DARK, 'Stop talking')]
    custom = b('SAY_TEXT:Type what Misty should say here', TEAL, 'Say custom text', editable=True)
    speech_tab = L.split([L.grid(speech, 5), custom], 'column', [72, 28])

    group_colors = {'Sad': PURPLE, 'Confused': AMBER, 'Frustrated': RED, 'Happy': GREEN}
    sound_rows = []
    for group, files in SOUNDS['groups'].items():
        color = group_colors.get(group, GREY)
        sound_rows.append(L.split([b(f'PLAY:{f}', color, f'{group}: ' + f.removeprefix('s_').removesuffix('.wav'))
                                   for f in files], 'row'))
    sound_rows.append(b('STOP_TALKING', DARK, 'Stop sound'))
    sounds_tab = L.split(sound_rows, 'column')
    tabs = L.panel('Tab', {'activeTabIdx': 0, 'tabs': [
        {'title': 'Speech', 'layout': speech_tab}, {'title': 'Sounds', 'layout': sounds_tab}]})

    failure = L.split([b('FAILURE', RED, '⚠ FAILURE: lost + sad sound + ask for help'),
                       b('STOP_TALKING', DARK, 'Cancel / stop talking')], 'row', [75, 25])
    motion = [b('NEXT_SEGMENT', BLUE, 'Next segment ▶'), b('LOOK_BACK', BLUE, 'Look back'),
              b('HEAD_CENTER', GREY, 'Head centre'), b('ROUTE_RESET', GREY, 'Route reset'),
              b('NUDGE_L', GREY, '◀ Nudge L'), b('NUDGE_FWD', GREY, '▲ Nudge fwd'),
              b('NUDGE_R', GREY, 'Nudge R ▶'), b('STOP', DARK, '■ STOP all')]
    session = L.split([b('START', BLUE, 'START session'), b('END', DARK, 'END session'),
                       b('TOUCH_SIM', GREY, 'Log touch (sim)')], 'row')
    status = L.split([L.status('/wizard/session', 18, 'Session'),
                      L.status('/wizard/said', 16, 'Misty said / played'),
                      L.status('/wizard/last', 16, 'Last action / head touch'),
                      L.status('/wizard/route', 16, 'Next route segment')], 'column')
    left = L.split([L.camera(),
                    L.split([L.teleop('/cmd_vel', 'DRIVE (arrows = forward/back/turn)'),
                             L.teleop('/misty/head_cmd', 'HEAD / CAMERA (up/down = tilt, left/right = pan)', rate=10)], 'row'),
                    L.grid(motion, 4)], 'column', [50, 28, 22])
    right = L.split([status, failure, tabs, L.split([session, L.params()], 'row', [55, 45])],
                    'column', [20, 12, 52, 16])
    L.write(L.split([left, right], 'row', [42, 58]), 'misty_wizard_layout.json')


main()
