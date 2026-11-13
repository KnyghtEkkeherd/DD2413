#!/usr/bin/env python3
"""Generate foxglove/misty_wizard_layout.json (import it in Foxglove:
Layouts menu -> Import from file). Re-run after changing buttons here."""
import json
from pathlib import Path

configs = {}


def panel(kind, cfg):
    pid = f'{kind}!{kind.lower()}{len(configs)}'
    configs[pid] = cfg
    return pid


def split(items, direction, sizes=None):
    """Mosaic tree splitting items evenly (or by sizes, in %) along direction."""
    if len(items) == 1:
        return items[0]
    sizes = sizes or [100 / len(items)] * len(items)
    first = sizes[0] / sum(sizes) * 100
    return {'first': items[0], 'second': split(items[1:], direction, sizes[1:]),
            'direction': direction, 'splitPercentage': first}


def button(event, color, label=None):
    return panel('Publish', {
        'topicName': '/wizard/event', 'datatype': 'std_msgs/msg/String',
        'buttonText': label or event.replace('_', ' '), 'buttonTooltip': f'Send {event}',
        'buttonColor': color, 'advancedView': False, 'value': json.dumps({'data': event})})


def status(topic, size=18):
    return panel('RawMessages', {
        'topicPath': f'{topic}.data', 'diffEnabled': False, 'diffMethod': 'custom',
        'diffTopicPath': '', 'showFullMessageForDiff': False, 'fontSize': size})


def teleop(topic):
    return panel('Teleop', {
        'topic': topic, 'publishRate': 5,
        'upButton': {'field': 'linear-x', 'value': 1}, 'downButton': {'field': 'linear-x', 'value': -1},
        'leftButton': {'field': 'angular-z', 'value': 1}, 'rightButton': {'field': 'angular-z', 'value': -1}})


GREEN, RED, BLUE, GREY, AMBER, DARK = '#2e9e5b', '#d33b3b', '#3b7dd8', '#6b6b7b', '#d8902b', '#8a1c1c'
rows = [
    [button('START', BLUE), button('YES', GREEN), button('NO', RED), button('NUDGE', GREY)],
    [button('DISTRACTED', GREY), button('TOUCH_SIM', GREY, 'TOUCH (sim)')],
    [button('END_SUCCESS', GREEN), button('END_FAIL', AMBER)],
    [button('DIR_CORRECT', GREEN), button('DIR_WRONG', RED), button('COMFORT', BLUE)],
    [button('NUDGE_L', GREY, '◀ NUDGE L'), button('NUDGE_FWD', GREY, '▲ NUDGE FWD'),
     button('NUDGE_R', GREY, 'NUDGE R ▶'), button('ABORT', DARK)],
]
buttons = split([split(r, 'row') for r in rows], 'column')
statuses = split([status('/wizard/state', 22), status('/wizard/allowed'),
                  status('/wizard/cue'), status('/wizard/said')], 'column')
params = panel('Parameters', {'title': 'Session (set participant_id + condition, then START)'})
camera = panel('Image', {'imageMode': {'imageTopic': '/misty/camera/image/compressed'}})
drive = split([teleop('/cmd_vel'), teleop('/misty/head_cmd')], 'row')

layout = {
    'configById': configs, 'globalVariables': {}, 'userNodes': {}, 'playbackConfig': {'speed': 1},
    'layout': split([
        split([camera, drive], 'column', [65, 35]),
        split([statuses, buttons, params], 'column', [30, 45, 25]),
    ], 'row', [45, 55]),
}
out = Path(__file__).resolve().parents[1] / 'foxglove' / 'misty_wizard_layout.json'
out.write_text(json.dumps(layout, indent=2) + '\n')
print(f'wrote {out} ({len(configs)} panels)')
