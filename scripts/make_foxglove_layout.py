#!/usr/bin/env python3
"""Generate the Foxglove layouts (import in Foxglove: Layouts -> Import from file).

  foxglove/misty_wizard_manual_layout.json   mode:=manual (default)
  foxglove/misty_wizard_auto_layout.json     mode:=auto (state machine)

Re-run after changing buttons here or speech categories in speech_bank.yaml.
"""
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEECH = list(yaml.safe_load(open(ROOT / 'src/misty_ros/config/speech_bank.yaml')))
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

    def teleop(self, topic, title):
        return self.panel('Teleop', {
            'foxglovePanelTitle': title, 'topic': topic, 'publishRate': 5,
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


def manual():
    L = Layout()
    b = L.button
    phases = [b('PHASE:S1', GREY, 'Greeting'), b('PHASE:S2', GREY, 'Consent'),
              b('PHASE:S2b', GREY, 'Nudge'), b('PHASE:S3', GREY, 'Escort A'),
              b('PHASE:S4', GREY, 'Half pause'), b('PHASE:S5', GREY, 'Escort B'),
              b('PHASE:S6', AMBER, 'Confusion'), b('PHASE:S7', RED, 'Frustration'),
              b('PHASE:S8', PURPLE, 'Sad appeal'), b('PHASE:S8w', PURPLE, 'Help wait'),
              b('PHASE:S8r', GREEN, 'Recovery'), b('PHASE:S5b', GREEN, 'Arrive')]
    speech_colors = {'CONFUSED': AMBER, 'FRUSTRATED': RED, 'LOST_APPEAL': PURPLE,
                     'THANKS': GREEN, 'RECOVER': GREEN, 'ARRIVE': GREEN, 'DISAGREE': RED}
    speech = [b(f'SAY:{c}', speech_colors.get(c, BLUE), c.replace('_', ' ').title()) for c in SPEECH]
    speech += [b('STOP_TALKING', DARK, 'Stop talking')]
    custom = b('SAY_TEXT:Type what Misty should say here', TEAL, 'Say custom text', editable=True)
    motion = [b('NEXT_SEGMENT', BLUE, 'Next segment ▶'), b('LOOK_BACK', BLUE, 'Look back'),
              b('HEAD_CENTER', GREY, 'Head centre'), b('ROUTE_RESET', GREY, 'Route reset'),
              b('NUDGE_L', GREY, '◀ Nudge L'), b('NUDGE_FWD', GREY, '▲ Nudge fwd'),
              b('NUDGE_R', GREY, 'Nudge R ▶'), b('STOP', DARK, '■ STOP')]
    session = L.split([b('START', BLUE, 'START session'), b('END', DARK, 'END session'),
                       b('TOUCH_SIM', GREY, 'Log touch (sim)')], 'row')
    status = L.split([L.status('/wizard/state', 18, 'Phase / session'),
                      L.status('/wizard/cue', 16, 'Emotion cue (phone app) + line to say'),
                      L.status('/wizard/said', 16, 'Misty said'),
                      L.status('/wizard/last', 16, 'Last action / head touch'),
                      L.status('/wizard/allowed', 16, 'Next route segment')], 'column')
    left = L.split([L.camera(), L.split([L.teleop('/cmd_vel', 'DRIVE (arrows = forward/back/turn)'), L.teleop('/misty/head_cmd', 'HEAD / CAMERA (up/down = tilt, left/right = pan)')], 'row'),
                    L.grid(motion, 4)], 'column', [50, 28, 22])
    right = L.split([status, L.grid(phases, 6), L.grid(speech, 5), custom,
                     L.split([session, L.params()], 'row', [55, 45])], 'column', [24, 14, 32, 14, 16])
    L.write(L.split([left, right], 'row', [42, 58]), 'misty_wizard_manual_layout.json')


def auto():
    L = Layout()
    b = L.button
    rows = [
        [b('START', BLUE), b('YES', GREEN), b('NO', RED), b('NUDGE', GREY)],
        [b('DISTRACTED', GREY), b('TOUCH_SIM', GREY, 'TOUCH (sim)')],
        [b('END_SUCCESS', GREEN), b('END_FAIL', AMBER)],
        [b('DIR_CORRECT', GREEN), b('DIR_WRONG', RED), b('COMFORT', BLUE)],
        [b('NUDGE_L', GREY, '◀ NUDGE L'), b('NUDGE_FWD', GREY, '▲ NUDGE FWD'),
         b('NUDGE_R', GREY, 'NUDGE R ▶'), b('ABORT', DARK)],
    ]
    buttons = L.split([L.split(r, 'row') for r in rows], 'column')
    status = L.split([L.status('/wizard/state', 22, 'State'),
                      L.status('/wizard/allowed', 16, 'Valid buttons now'),
                      L.status('/wizard/cue', 16, 'Emotion cue (phone app)'),
                      L.status('/wizard/said', 16, 'Misty said')], 'column')
    left = L.split([L.camera(), L.split([L.teleop('/cmd_vel', 'DRIVE (arrows = forward/back/turn)'), L.teleop('/misty/head_cmd', 'HEAD / CAMERA (up/down = tilt, left/right = pan)')], 'row')],
                   'column', [65, 35])
    right = L.split([status, buttons, L.params()], 'column', [30, 45, 25])
    L.write(L.split([left, right], 'row', [45, 55]), 'misty_wizard_auto_layout.json')


manual()
auto()
