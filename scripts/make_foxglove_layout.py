#!/usr/bin/env python3
"""Generate foxglove/misty_wizard_layout.json from config/reactions.json and config/study.yaml.

Tabs "Study" (answers + a button per study state) and "Emotions & hands" come
from study.yaml (states, emotions, gestures).

Every reaction becomes a button (one per sentence if "each": true), on a tab
named after its "group"; reactions in group "Main" go in the always-visible
row above the tabs. Reactions with "button": false get no button (still usable
in sequences and via REACT:<id>). Re-run after adding/removing reactions, then re-import in
Foxglove via Layouts -> Import from file (not drag-and-drop).
"""
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DATA = json.load(open(ROOT / 'src/misty_ros/config/reactions.json'))
STUDY = yaml.safe_load(open(ROOT / 'src/misty_ros/config/study.yaml'))
GREEN, RED, BLUE, GREY, DARK, TEAL = '#2e9e5b', '#d33b3b', '#3b7dd8', '#6b6b7b', '#8a1c1c', '#1f8a8a'
AMBER, PURPLE = '#c98a14', '#7b4fc9'
BREAKDOWN_STATES = {'CONFUSION', 'FRUSTRATION', 'LOST_APPEAL', 'HELP_WAIT', 'TOUCH_ACK', 'GRATITUDE', 'DISAGREE',
                    'RECOVERY'}
PER_ROW = 5


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

    def button(self, data, color, label, editable=False):
        return self.panel('Publish', {
            'topicName': '/wizard/event', 'datatype': 'std_msgs/msg/String',
            'buttonText': label, 'buttonTooltip': f'Send {data}', 'buttonColor': color,
            'advancedView': editable, 'value': json.dumps({'data': data})})

    def status(self, topic, size, title):
        return self.panel('RawMessages', {
            'foxglovePanelTitle': title, 'topicPath': f'{topic}.data', 'diffEnabled': False,
            'diffMethod': 'custom', 'diffTopicPath': '', 'showFullMessageForDiff': False, 'fontSize': size})

    def teleop(self, topic, title, rate):
        return self.panel('Teleop', {
            'foxglovePanelTitle': title, 'topic': topic, 'publishRate': rate,
            'upButton': {'field': 'linear-x', 'value': 1}, 'downButton': {'field': 'linear-x', 'value': -1},
            'leftButton': {'field': 'angular-z', 'value': 1}, 'rightButton': {'field': 'angular-z', 'value': -1}})


def reaction_buttons(L, r):
    color = r.get('color', BLUE)
    label = r.get('label', r['id'].replace('_', ' ').title())
    if r.get('each') and r.get('sentences'):
        out = []
        for i, s in enumerate(r['sentences']):
            text = s['text'] if isinstance(s, dict) else str(s)
            out.append(L.button(f'REACT:{r["id"]}:{i}', color, f'{label}: {text[:40]}'))
        return out
    return [L.button(f'REACT:{r["id"]}', color, label)]


def study_layout(L):
    """'Study' tab: answers + one button per state; 'Emotions & hands' tab."""
    states = list((STUDY.get('study') or {}).get('states') or {})
    answers = [L.button('YES', GREEN, 'YES'), L.button('NO', RED, 'NO'),
               L.button('DIR_OK', GREEN, 'Direction OK (thanks)'), L.button('DIR_WRONG', AMBER, 'Direction wrong'),
               L.button('RECOVER', GREEN, 'RECOVER (end breakdown)'), L.button('DISTRACTED', AMBER, 'Distracted'),
               L.button('TOUCH_SIM', GREY, 'Head touch (sim)'), L.button('CONTINUE', BLUE, 'Skip ▶ next state')]
    state_buttons = [L.button(f'STATE:{s}', RED if s in BREAKDOWN_STATES else (DARK if s == 'END' else BLUE),
                              s.replace('_', ' ').title()) for s in states]
    study = L.split([L.grid(answers, 4), L.grid(state_buttons, 5)], 'column', [40, 60])
    emotions = [L.button(f'EMOTION:{e}', PURPLE, e.title()) for e in (STUDY.get('emotions') or {})]
    gestures = [L.button(f'GESTURE:{g}', TEAL, g.replace('_', ' ').title()) for g in (STUDY.get('gestures') or {})]
    feel = L.split([L.grid(emotions, 4), L.grid(gestures, 4)], 'column', [55, 45])
    return [{'title': 'Study', 'layout': study}, {'title': 'Emotions & hands', 'layout': feel}]


def main():
    L = Layout()
    groups = {}
    for r in DATA['reactions']:
        if r.get('button', True) is False:
            continue
        groups.setdefault(r.get('group', 'Reactions'), []).append(r)
    study_tabs = study_layout(L)
    main_buttons = [b for r in groups.pop('Main', []) for b in reaction_buttons(L, r)]
    main_buttons.append(L.button('STOP_TALKING', DARK, 'Stop talking / cancel'))
    tabs = study_tabs + [{'title': g, 'layout': L.grid([b for r in rs for b in reaction_buttons(L, r)], PER_ROW)}
                         for g, rs in groups.items()]
    custom = L.button('SAY_TEXT:[neutral] Type what Misty should say here', TEAL,
                      'Say custom text ([voice] text)', editable=True)
    tabs.append({'title': 'Custom text', 'layout': custom})
    nav_buttons = L.split([L.button('NAV_START', GREEN, '▶ NAV START'), L.button('NAV_GO', BLUE, 'NAV GO (continue)'),
                           L.button('NAV_PAUSE', GREY, '❚❚ NAV PAUSE'), L.button('NAV_RESET', DARK, 'NAV RESET'),
                           L.button('MAP_RESET', DARK, 'MAP RESET (forget learned markers)')], 'row')
    # topics of both stacks: Nav2 (nav2.launch.py) and the custom one (nav.launch.py)
    view3d = L.panel('3D', {'foxglovePanelTitle': 'Map: markers, robot, plan, costmap', 'followTf': 'map',
                            'topics': {t: {'visible': True} for t in
                                       ('/localization/marker_map', '/global_costmap/costmap', '/plan',
                                        '/local_plan', '/nav/goal_pose', '/odometry/global', '/scan',
                                        '/slam/landmark_markers', '/slam/path', '/slam/pose',
                                        '/misty/depth_scan', '/sim/world', '/sim/true_pose')}})
    tabs.append({'title': 'Navigation', 'layout': L.split([nav_buttons, view3d], 'column', [18, 82])})
    tab_panel = L.panel('Tab', {'activeTabIdx': 0, 'tabs': tabs})

    motion = [L.button('LOOK_BACK', BLUE, 'Look back'), L.button('HEAD_CENTER', GREY, 'Head centre'),
              L.button('FACE_TRACK_ON', GREEN, 'Face tracking ON'),
              L.button('FACE_TRACK_OFF', DARK, 'Face tracking OFF'),
              L.button('STOP', DARK, '■ STOP all (pause study + nav)'), L.button('RESUME', GREEN, '▶ RESUME')]
    session = L.split([L.button('START', BLUE, 'START session'), L.button('END', DARK, 'END session'),
                       L.button('TOUCH_SIM', GREY, 'Log touch (sim)')], 'row')
    params = L.panel('Parameters', {'title': 'Session: set participant_id + condition, then START'})
    status = L.split([L.status('/study/state', 20, 'Study state'),
                      L.status('/wizard/session', 16, 'Session'),
                      L.status('/wizard/said', 16, 'Misty said / played'),
                      L.status('/wizard/last', 16, 'Last action / head touch'),
                      L.status('/nav/status', 16, 'Navigation status (fail_trigger: Parameters > /misty_nav)'),
                      L.status('/misty/face_tracking/status', 16, 'Face tracking status')],
                     'column')
    camera = L.panel('Image', {'foxglovePanelTitle': 'Misty camera (markers + phase)',
                               'imageMode': {'imageTopic': '/misty/camera/overlay/compressed'}})
    left = L.split([camera,
                    L.split([L.teleop('/cmd_vel', 'DRIVE: overrides Nav2 (arrows = forward/back/turn)', 10),
                             L.teleop('/misty/head_cmd', 'HEAD / CAMERA (up/down = tilt, left/right = pan)', 10)],
                            'row'),
                    L.grid(motion, 3)], 'column', [50, 28, 22])
    right = L.split([status, L.split(main_buttons, 'row'), tab_panel, L.split([session, params], 'row', [55, 45])],
                    'column', [24, 8, 54, 14])
    out = ROOT / 'foxglove' / 'misty_wizard_layout.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({'configById': L.configs, 'globalVariables': {}, 'userNodes': {},
                               'playbackConfig': {'speed': 1}, 'layout': L.split([left, right], 'row', [42, 58])},
                              indent=2, ensure_ascii=False) + '\n')
    print(f'wrote {out}: {len(L.configs)} panels, tabs: {[t["title"] for t in tabs]}')


main()
