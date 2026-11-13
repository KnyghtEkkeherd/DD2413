"""Wizard-of-Oz state machine for the Misty breakdown study.

Implements the study plan's state machine (S0..S9). The wizard presses buttons
in Foxglove, which publish std_msgs/String on /wizard/event:

    START YES NO NUDGE DISTRACTED TOUCH_SIM END_SUCCESS END_FAIL
    DIR_CORRECT DIR_WRONG COMFORT NUDGE_L NUDGE_R NUDGE_FWD ABORT

Autonomous on Misty: route segments, look-backs, speech, head-touch sensing.
Emotions (mood/face/LEDs/expressive gestures) are NOT driven from here: the
wizard sets them in the Misty app, prompted by /wizard/cue for each state.

Status topics (latched, for Foxglove Raw Messages panels):
    /wizard/state    current state, condition, participant
    /wizard/allowed  buttons that do something right now
    /wizard/cue      what to set in the Misty app now (from the plan)
    /wizard/said     last utterance

Condition and participant are ROS parameters (set them in Foxglove's
Parameters panel or as launch args) and are read when START is pressed.
One CSV per session is written to session_dir.
"""
import csv
import json
import queue
import random
import threading
import time
import uuid
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

from misty_ros.sdk import make_robot

try:
    import websocket
except ImportError:  # touch sensing + exact speech timing need python3-websocket
    websocket = None

WIZARD_EVENTS = {
    'START', 'YES', 'NO', 'NUDGE', 'DISTRACTED', 'TOUCH_SIM', 'END_SUCCESS', 'END_FAIL',
    'DIR_CORRECT', 'DIR_WRONG', 'COMFORT', 'NUDGE_L', 'NUDGE_R', 'NUDGE_FWD', 'ABORT',
}
DRIFT = {'NUDGE_L', 'NUDGE_R', 'NUDGE_FWD'}
CONDITIONS = ('CONTROL', 'BREAKDOWN')

STATES = {
    'S0': 'BOOT', 'S1': 'GREETING', 'S2': 'CONSENT_CHECK', 'S2b': 'NUDGE',
    'S3': 'ESCORT_A', 'S3b': 'LOOK_BACK', 'S4': 'HALF_PAUSE',
    'S5': 'ESCORT_B', 'S5b': 'ARRIVE',
    'S6': 'CONFUSION', 'S7': 'FRUSTRATION', 'S8': 'SADNESS_APPEAL', 'S8w': 'HELP_WAIT',
    'S8t': 'TOUCH_ACK', 'S8y': 'GRATITUDE', 'S8n': 'GENTLE_DISAGREE', 'S8r': 'RECOVERY',
    'S9': 'END',
}

# What the wizard should do in the Misty app on entering each state (from the plan).
CUES = {
    'S0': 'Face view, NEUTRAL mood. Set condition + participant_id, then START.',
    'S1': 'Neutral.',
    'S2': 'Neutral. Press YES / NO when the participant answers.',
    'S2b': 'GRUMPY briefly, then back to neutral.',
    'S3': 'Neutral. DISTRACTED if they talk to Misty; nudges for drift.',
    'S3b': 'HAPPY while looking back.',
    'S4': 'NEUTRAL, LEDs steady. Press END SUCCESS (control) or END FAIL (breakdown).',
    'S5': 'Neutral.',
    'S5b': 'HAPPY, LEDs green pulse.',
    'S6': 'LEDs AMBER flicker, mood sceptical/neutral, look left-right.',
    'S7': 'GRUMPY, LEDs RED, sharper head motion.',
    'S8': 'SAD, LEDs DIM BLUE, head lowers.',
    'S8w': 'Stay SAD. Wait. DIR CORRECT / DIR WRONG / COMFORT to end.',
    'S8t': 'Head tilts toward touch, LEDs a little brighter, soften one step (NOT happy).',
    'S8y': 'HAPPY briefly (Misty does not move yet).',
    'S8n': 'Head shake ("no").',
    'S8r': 'HAPPY, LEDs GREEN.',
    'S9': 'NEUTRAL face. Session saved.',
}

CSV_FIELDS = ['timestamp', 'participant_id', 'condition', 'state', 'event_source',
              'event', 'utterance_variant_index', 'detail']


class Abort(Exception):
    pass


class Ev:
    __slots__ = ('name', 'source', 'detail', 'deferred')

    def __init__(self, name, source, detail=''):
        self.name, self.source, self.detail = name, source, detail
        self.deferred = False

    def note(self, extra):
        parts = [self.detail, 'same press as deferred row above' if self.deferred else '', extra]
        return '; '.join(p for p in parts if p)


class MistyWizard(Node):
    def __init__(self):
        super().__init__('misty_wizard')
        share = Path(get_package_share_directory('misty_ros')) / 'config'
        self.declare_parameter('misty_ip', '')
        self.declare_parameter('participant_id', '')
        self.declare_parameter('condition', '')          # CONTROL or BREAKDOWN
        self.declare_parameter('study_config', str(share / 'study.yaml'))
        self.declare_parameter('speech_bank', str(share / 'speech_bank.yaml'))
        self.declare_parameter('session_dir', str(Path.home() / 'misty_ws' / 'sessions'))
        self.declare_parameter('request_timeout', 2.0)

        self.ip = self.get_parameter('misty_ip').value
        if not self.ip:
            raise RuntimeError('Parameter misty_ip is required (or set MISTY_IP for the launch file).')
        self.misty = make_robot(self.ip, float(self.get_parameter('request_timeout').value))

        self.events = queue.Queue()
        self.pending = deque()        # events deferred while Misty was busy
        self.drift_queue = deque()    # nudges pressed while a segment was driving
        self.tts_done = set()
        self.tts_lock = threading.Lock()
        self.ws_ok = False
        self.stop_flag = threading.Event()

        self.state = 'S0'
        self.allowed = set()
        self.participant = ''
        self.condition = ''
        self.csv_file = None
        self.csv = None
        self.last_variant = {}
        self.last_look_back = 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = {n: self.create_publisher(String, f'wizard/{n}', latched)
                    for n in ('state', 'allowed', 'cue', 'said', 'last')}
        self.create_subscription(String, 'wizard/event', self.on_wizard_msg, 10)
        self.head_pub = self.create_publisher(Vector3, 'misty/head_position', 10)

        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        if websocket is not None:
            threading.Thread(target=self.pubsub_loop, daemon=True).start()
        else:
            self.get_logger().warning('python3-websocket missing: no touch sensing, speech timing estimated')
        self.get_logger().info(f'Wizard ready for Misty {self.ip}; waiting for START on /wizard/event')

    # ---------------------------------------------------------------- inputs
    def on_wizard_msg(self, msg: String):
        name = msg.data.strip().upper().replace(' ', '_').replace('-', '_')
        if name.startswith('W_'):
            name = name[2:]
        if name not in WIZARD_EVENTS:
            self.get_logger().warning(f'Unknown wizard event "{msg.data}"')
            return
        if name == 'TOUCH_SIM':
            self.events.put(Ev('TOUCH', 'wizard', 'TOUCH_SIM'))
        else:
            self.events.put(Ev(name, 'wizard'))

    def pubsub_loop(self):
        """Misty's event websocket: head touch sensors and speech completion."""
        subs = {'misty_wizard_touch': 'TouchSensor', 'misty_wizard_tts': 'TextToSpeechComplete'}
        warned = False
        while not self.stop_flag.is_set():
            ws = None
            try:
                ws = websocket.create_connection(f'ws://{self.ip}/pubsub', timeout=5)
                for name, typ in subs.items():
                    ws.send(json.dumps({'Operation': 'subscribe', 'Type': typ, 'DebounceMs': 100,
                                        'EventName': name, 'Message': ''}))
                ws.settimeout(1.0)
                self.ws_ok, warned = True, False
                self.get_logger().info('Connected to Misty event websocket (touch + speech events)')
                while not self.stop_flag.is_set():
                    try:
                        self.on_pubsub(ws.recv())
                    except websocket.WebSocketTimeoutException:
                        continue
            except Exception as e:
                if not warned:
                    self.get_logger().warning(f'Misty event websocket unavailable ({e}); retrying. '
                                           'Touch sensing off, speech timing estimated.')
                    warned = True
            finally:
                self.ws_ok = False
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
            self.stop_flag.wait(3.0)

    def on_pubsub(self, raw):
        try:
            data = json.loads(raw)
        except (TypeError, ValueError):
            return                        # registration status strings
        if not isinstance(data, dict):
            return
        data = {k.lower(): v for k, v in data.items()}
        msg = data.get('message')
        if isinstance(msg, str):
            try:
                msg = json.loads(msg)
            except ValueError:
                return
        if not isinstance(msg, dict):
            return
        msg = {k.lower(): v for k, v in msg.items()}
        name = data.get('eventname')
        if name == 'misty_wizard_touch' and msg.get('iscontacted'):
            self.events.put(Ev('TOUCH', 'sensor', str(msg.get('sensorposition', ''))))
        elif name == 'misty_wizard_tts' and msg.get('utteranceid'):
            with self.tts_lock:
                self.tts_done.add(msg['utteranceid'])

    # ------------------------------------------------------------- logging
    def publish(self, topic, text):
        self.pub[topic].publish(String(data=text))

    def log(self, source, event, variant='', detail=''):
        name = STATES.get(self.state, '')
        self.get_logger().info(f'[{self.state} {name}] {source}:{event} {variant} {detail}'.rstrip())
        if self.csv is not None:
            self.csv.writerow({
                'timestamp': datetime.now().isoformat(timespec='milliseconds'),
                'participant_id': self.participant, 'condition': self.condition,
                'state': f'{self.state}_{name}', 'event_source': source, 'event': event,
                'utterance_variant_index': variant, 'detail': detail})
            self.csv_file.flush()

    def enter(self, state, allowed=(), detail=''):
        self.state = state
        self.allowed = set(allowed)
        self.log('auto', 'STATE_ENTER', detail=detail)
        session = f'{self.condition or "-"} / {self.participant or "-"}'
        self.publish('state', f'{state} {STATES[state]}   [{session}]')
        buttons = sorted('TOUCH_SIM' if a == 'TOUCH' else a for a in self.allowed)
        if state not in ('S0', 'S9'):
            buttons.append('ABORT')
        self.publish('allowed', ', '.join(buttons) or '(none, wait)')
        self.publish('cue', CUES[state])

    # --------------------------------------------------------- event waits
    def _take(self, timeout, include_pending):
        if include_pending and self.pending:
            return self.pending.popleft()
        try:
            return self.events.get(timeout=max(0.0, timeout))
        except queue.Empty:
            return None

    def wait_for(self, allowed, timeout=None):
        """Waiting state: return the first allowed event, or None on timeout.

        ABORT aborts; drift nudges run immediately if allowed; touches are
        always logged; anything else not allowed here is logged and ignored.
        """
        allowed = set(allowed)
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            remaining = 0.25 if deadline is None else deadline - time.monotonic()
            if deadline is not None and remaining <= 0:
                return None
            ev = self._take(min(remaining, 0.25), include_pending=True)
            if ev is None or self.stop_flag.is_set():
                if self.stop_flag.is_set():
                    raise Abort()
                continue
            if ev.name == 'ABORT':
                raise Abort()
            if ev.name in allowed:
                self.log(ev.source, ev.name, detail=ev.note(''))
                return ev
            if ev.name in DRIFT and DRIFT & allowed:
                self.log(ev.source, ev.name, detail=ev.note(''))
                self.drift(ev.name)
                continue
            self.log(ev.source, ev.name, detail=ev.note('ignored'))

    def busy(self, seconds=None, until=None, driving=False):
        """Misty is doing something: sleep, but stay responsive to ABORT.

        Touches are logged. While driving, DISTRACTED is answered on the fly
        and drift nudges are queued to run after the segment. Other wizard
        events are deferred to the next waiting state.
        """
        deadline = None if seconds is None else time.monotonic() + seconds
        while True:
            if until is not None and until():
                return
            if deadline is not None and time.monotonic() >= deadline:
                return
            if self.stop_flag.is_set():
                raise Abort()
            ev = self._take(0.1, include_pending=False)
            if ev is None:
                continue
            if ev.name == 'ABORT':
                raise Abort()
            if ev.name == 'TOUCH':
                self.log(ev.source, 'TOUCH', detail=ev.detail)
            elif driving and ev.name in DRIFT:
                self.log(ev.source, ev.name, detail='queued until segment ends')
                self.drift_queue.append(ev.name)
            elif driving and ev.name == 'DISTRACTED':
                self.log(ev.source, ev.name)
                self.say('CONCENTRATE', wait=False)
            else:
                self.log(ev.source, ev.name, detail='deferred (Misty busy)')
                ev.deferred = True
                self.pending.append(ev)

    def busy_until(self, t_end):
        self.busy(max(0.0, t_end - time.monotonic()))

    # -------------------------------------------------------- robot actions
    def call(self, what, fn, **kwargs):
        try:
            r = fn(**kwargs)
            if not r.ok:
                self.get_logger().warning(f'{what}: HTTP {r.status_code} {r.text[:200]}')
        except Exception as e:
            self.get_logger().warning(f'{what} failed: {e}')

    def say(self, category, wait=True, source='auto'):
        variants = self.speech.get(category) or [category]
        choices = [i for i in range(len(variants)) if i != self.last_variant.get(category)] or [0]
        idx = random.choice(choices)
        self.last_variant[category] = idx
        text = variants[idx]
        uid = f'wiz_{uuid.uuid4().hex[:12]}'
        self.log(source, f'SAY_{category}', variant=idx, detail=text)
        self.publish('said', f'{category}[{idx}]: {text}')
        self.call('speak', self.misty.speak, text=text, flush=True, utteranceId=uid)
        if not wait:
            return
        sp = self.cfg['speech']
        estimate = len(text.split()) * sp['fallback_s_per_word'] + sp['fallback_extra_s']
        if self.ws_ok:
            def done():
                with self.tts_lock:
                    return uid in self.tts_done
            self.busy(estimate + sp['completion_margin_s'], until=done)
        else:
            self.busy(estimate)

    def drive_segment(self, seg):
        if 'forward' in seg:
            f = seg['forward']
            self.call('drive_time', self.misty.drive_time, linearVelocity=float(f['velocity']),
                      angularVelocity=0.0, timeMs=int(f['time_ms']))
            ms = f['time_ms']
        elif 'turn' in seg:
            t = seg['turn']
            deg = float(t['degrees'])
            vel = abs(float(t.get('velocity', 30))) * (1 if deg >= 0 else -1)
            self.call('drive_time', self.misty.drive_time, linearVelocity=0.0,
                      angularVelocity=vel, timeMs=int(t['time_ms']), degree=deg)
            ms = t['time_ms']
        else:
            self.get_logger().error(f'Bad route segment {seg}: needs forward or turn')
            return
        self.busy(ms / 1000.0 + self.cfg['timing']['segment_settle'], driving=True)

    def drift(self, name):
        n = self.cfg['nudge']
        if name == 'NUDGE_FWD':
            self.call('drive_time', self.misty.drive_time, linearVelocity=float(n['forward_velocity']),
                      angularVelocity=0.0, timeMs=int(n['forward_ms']))
            ms = n['forward_ms']
        else:
            sign = 1.0 if name == 'NUDGE_L' else -1.0  # + = left (counter-clockwise)
            self.call('drive_time', self.misty.drive_time, linearVelocity=0.0,
                      angularVelocity=sign * float(n['turn_velocity']), timeMs=int(n['turn_ms']))
            ms = n['turn_ms']
        self.busy(ms / 1000.0 + 0.2, driving=True)

    def head(self, yaw):
        lb = self.cfg['look_back']
        self.call('move_head', self.misty.move_head, pitch=0.0, roll=0.0, yaw=float(yaw),
                  velocity=float(lb['head_velocity']), units='degrees')
        self.head_pub.publish(Vector3(x=0.0, y=0.0, z=float(yaw)))  # keep head teleop in sync

    def halt(self):
        self.call('stop', self.misty.stop)
        self.call('stop_speaking', self.misty.stop_speaking)

    # ----------------------------------------------------------- the study
    def run(self):
        while not self.stop_flag.is_set():
            try:
                self.session()
            except Abort:
                if self.stop_flag.is_set():
                    return
                self.log('wizard', 'ABORT')
                self.halt()
            except Exception as e:
                self.get_logger().error(f'State machine error: {e!r}')
                self.halt()
            try:
                self.end()
            except Exception as e:  # never let the worker thread die
                self.get_logger().error(f'Error ending session: {e!r}')
                self.csv_file = self.csv = None

    def load_config(self):
        with open(self.get_parameter('study_config').value) as f:
            self.cfg = yaml.safe_load(f)
        with open(self.get_parameter('speech_bank').value) as f:
            self.speech = yaml.safe_load(f)

    def start_session(self):
        """S0: wait for START with a valid condition + participant, open the CSV."""
        self.participant = self.condition = ''
        self.pending.clear()
        self.drift_queue.clear()
        self.enter('S0', {'START'})
        while True:
            self.wait_for({'START'})
            pid = str(self.get_parameter('participant_id').value).strip()
            cond = str(self.get_parameter('condition').value).strip().upper()
            if not pid or cond not in CONDITIONS:
                msg = (f'Cannot start: set participant_id (now "{pid}") and condition '
                       f'CONTROL/BREAKDOWN (now "{cond}") in the Parameters panel, then START.')
                self.get_logger().warning(msg)
                self.publish('cue', msg)
                continue
            try:
                self.load_config()
            except Exception as e:
                self.publish('cue', f'Cannot start: config error: {e}')
                self.get_logger().error(f'Config error: {e}')
                continue
            break
        self.open_session_log(pid, cond)

    def open_session_log(self, pid, cond):
        self.participant, self.condition = pid, cond
        out = Path(self.get_parameter('session_dir').value).expanduser()
        out.mkdir(parents=True, exist_ok=True)
        path = out / f'{pid}_{cond}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        self.csv_file = open(path, 'w', newline='')
        self.csv = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv.writeheader()
        self.log('wizard', 'START', detail=str(path))
        self.get_logger().info(f'Session log: {path}')

    def session(self):
        self.start_session()
        t, route = self.cfg['timing'], self.cfg['route']
        segments, k = route['segments'], int(route['halfway_index'])

        self.enter('S1')
        self.say('GREETING')

        self.enter('S2', {'YES', 'NO', 'NUDGE'})
        self.say('READY_Q')
        nudges = 0
        while True:
            ev = self.wait_for({'YES', 'NO', 'NUDGE'}, t['consent_timeout'])
            if ev is None:
                self.log('auto', 'TIMEOUT')
            elif ev.name == 'YES':
                break
            if nudges >= t['max_nudges']:
                self.log('auto', 'MAX_NUDGES', detail='continuing to S3')
                break
            nudges += 1
            self.enter('S2b', {'YES', 'NO', 'NUDGE'}, detail=f'nudge {nudges}')
            self.say('NUDGE')

        self.last_look_back = time.monotonic()
        self.escort(segments, range(0, k + 1), 'S3')

        self.enter('S4', {'END_SUCCESS', 'END_FAIL'} | DRIFT)
        ev = self.wait_for({'END_SUCCESS', 'END_FAIL'}, t['half_pause_timeout'])
        if ev is None:
            branch = self.condition
            self.log('auto', 'TIMEOUT', detail=f'using configured condition {branch}')
        else:
            branch = 'CONTROL' if ev.name == 'END_SUCCESS' else 'BREAKDOWN'
            if branch != self.condition:
                self.log('auto', 'CONDITION_MISMATCH',
                         detail=f'wizard chose {branch}, configured {self.condition}')

        if branch == 'CONTROL':
            pad = self.cfg.get('control_padding') or {}
            if pad.get('enabled'):
                self.say('CHECK_ROUTE')
                self.busy(float(pad.get('extra_wait', 0)))
        else:
            self.breakdown()

        self.last_look_back = time.monotonic()
        self.escort(segments, range(k + 1, len(segments)), 'S5')
        self.enter('S5b')
        self.say('ARRIVE')

    def escort(self, segments, indices, state):
        t = self.cfg['timing']
        allowed = {'DISTRACTED'} | DRIFT
        for i in indices:
            self.enter(state, allowed, detail=f'segment {i}: {segments[i]}')
            self.drive_segment(segments[i])
            while self.drift_queue:
                self.drift(self.drift_queue.popleft())
            # handle anything deferred that's valid while escorting
            for ev in [e for e in self.pending if e.name in allowed]:
                self.pending.remove(ev)
                self.log(ev.source, ev.name, detail=ev.note(''))
                if ev.name == 'DISTRACTED':
                    self.say('CONCENTRATE')
                else:
                    self.drift(ev.name)
            if segments[i].get('look_back') or \
                    time.monotonic() - self.last_look_back >= t['look_back_interval']:
                self.look_back(state)

    def look_back(self, resume_state):
        self.enter('S3b')
        self.head(self.cfg['look_back']['head_yaw_deg'])
        self.say('LOOK_BACK')
        self.busy(self.cfg['timing']['look_back_hold'])
        self.head(0)
        self.busy(1.0)
        self.last_look_back = time.monotonic()
        self.state = resume_state

    def breakdown(self):
        t = self.cfg['timing']
        self.call('stop', self.misty.stop)

        self.enter('S6')
        t0 = time.monotonic()
        self.say('CONFUSED')
        self.busy_until(t0 + t['confusion'])

        self.enter('S7')
        t0 = time.monotonic()
        self.say('FRUSTRATED')
        self.busy_until(t0 + t['frustration'])

        self.enter('S8')
        self.say('LOST_APPEAL')

        # Only the wizard (or the timeout) ends the help phase: touches and
        # directions are acknowledged but never shorten the breakdown.
        deadline = time.monotonic() + t['help_wait_timeout']
        waiting = {'TOUCH', 'DIR_CORRECT', 'DIR_WRONG', 'COMFORT'}
        while True:
            self.enter('S8w', waiting | DRIFT)
            ev = self.wait_for(waiting, deadline - time.monotonic())
            if ev is None:
                self.log('auto', 'TIMEOUT', detail='help wait expired')
                break
            if ev.name == 'COMFORT':
                break
            t0 = time.monotonic()
            if ev.name == 'TOUCH':
                self.enter('S8t', detail=ev.detail)
            elif ev.name == 'DIR_CORRECT':
                self.enter('S8y')
                self.say('THANKS')
            else:
                self.enter('S8n')
                self.say('DISAGREE')
            self.busy_until(t0 + t['ack'])

        self.enter('S8r')
        self.say('RECOVER')

    def end(self):
        self.drift_queue.clear()
        if self.state != 'S0' or self.csv is not None:
            self.enter('S9')
            self.call('stop', self.misty.stop)
            if hasattr(self, 'cfg'):
                self.head(0)
        if self.csv_file is not None:
            self.log('auto', 'SESSION_END')
            self.csv_file.close()
        self.csv_file = self.csv = None
        if not self.stop_flag.is_set():
            time.sleep(2.0)

    def shutdown(self):
        self.stop_flag.set()
        self.worker.join(timeout=3.0)
        self.call('stop', self.misty.stop)


def main():
    rclpy.init()
    node = None
    try:
        node = MistyWizard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
