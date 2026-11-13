"""Wizard-of-Oz reaction menu for Misty, driven from Foxglove.

There is no state machine: the wizard picks every reaction. Foxglove buttons
publish std_msgs/String on /wizard/event:

  Session   START / END              open / close the CSV log (participant_id
                                     and condition parameters are read at START)
  Speech    SAY:<CATEGORY>           random variant from the speech bank (index logged)
            SAY_TEXT:<text>          any text typed by the wizard
            STOP_TALKING             cut off speech, sounds and a running FAILURE
  Sounds    PLAY:<file.wav>          one of Misty's audio files (config/sounds.yaml)
  Failure   FAILURE                  predefined sequence from study.yaml
                                     (default: "I'm lost" -> sad sound -> "can you help me?")
  Motion    NEXT_SEGMENT / ROUTE_RESET   route segments from study.yaml
            LOOK_BACK / HEAD_CENTER      head turns back (+ a line) / straight
            NUDGE_L / NUDGE_R / NUDGE_FWD
            STOP                     stop driving, speech, sounds and FAILURE
            TOUCH_SIM                log a head touch by hand
  Free driving and head aiming: Foxglove Teleop panels (teleop_node).
  Emotions (mood, face, LEDs, gestures): the Misty phone app.

Status (latched, for Foxglove panels): /wizard/session, /wizard/said,
/wizard/last (last action, head touches), /wizard/route (next segment).
Head touches from Misty's sensors are logged automatically.
"""
import csv
import json
import queue
import random
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from misty_ros.sdk import make_robot

try:
    import websocket
except ImportError:  # touch sensing + exact speech timing need python3-websocket
    websocket = None

CONDITIONS = ('CONTROL', 'BREAKDOWN')
CSV_FIELDS = ['timestamp', 'participant_id', 'condition', 'event_source', 'event',
              'utterance_variant_index', 'detail']


class Cancelled(Exception):
    pass


class MistyWizard(Node):
    def __init__(self):
        super().__init__('misty_wizard')
        share = Path(get_package_share_directory('misty_ros')) / 'config'
        self.declare_parameter('misty_ip', '')
        self.declare_parameter('participant_id', '')
        self.declare_parameter('condition', '')          # CONTROL or BREAKDOWN
        self.declare_parameter('study_config', str(share / 'study.yaml'))
        self.declare_parameter('speech_bank', str(share / 'speech_bank.yaml'))
        self.declare_parameter('sounds', str(share / 'sounds.yaml'))
        self.declare_parameter('session_dir', str(Path.home() / 'misty_ws' / 'sessions'))
        self.declare_parameter('request_timeout', 2.0)

        self.ip = self.get_parameter('misty_ip').value
        if not self.ip:
            raise RuntimeError('Parameter misty_ip is required (or set MISTY_IP for the launch file).')
        self.misty = make_robot(self.ip, float(self.get_parameter('request_timeout').value))
        self.load_config()

        self.events = queue.Queue()
        self.stop_flag = threading.Event()
        self.tts_done = set()
        self.tts_lock = threading.Lock()
        self.ws_ok = False
        self.participant = self.condition = ''
        self.csv_file = self.csv = None
        self.log_lock = threading.Lock()
        self.last_variant = {}
        self.seg_i = 0
        self.sequence = None                  # running FAILURE thread
        self.cancel = threading.Event()       # cancels the running sequence

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = {n: self.create_publisher(String, f'wizard/{n}', latched)
                    for n in ('session', 'said', 'last', 'route')}
        self.head_pub = self.create_publisher(Vector3, 'misty/head_position', 10)
        self.create_subscription(String, 'wizard/event', self.on_wizard_msg, 10)

        self.publish_session()
        self.publish_route()
        self.note('Ready. Set participant_id + condition, press START.')
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        if websocket is not None:
            threading.Thread(target=self.pubsub_loop, daemon=True).start()
        else:
            self.get_logger().warning('python3-websocket missing: no touch sensing, speech timing estimated')
        self.get_logger().info(f'Wizard menu ready for Misty {self.ip}')

    # ------------------------------------------------------------------ config
    def load_config(self):
        def load(param):
            with open(self.get_parameter(param).value) as f:
                return yaml.safe_load(f) or {}
        self.cfg = load('study_config')
        self.speech = load('speech_bank')
        try:
            self.sounds = load('sounds')
        except OSError as e:
            self.sounds = {}
            self.get_logger().warning(f'No sounds config: {e}')

    # ------------------------------------------------------------------ inputs
    def on_wizard_msg(self, msg: String):
        head, _, arg = msg.data.strip().partition(':')
        name = head.strip().upper().replace(' ', '_').replace('-', '_')
        self.events.put((name, 'wizard', arg.strip()))

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
            self.events.put(('TOUCH', 'sensor', str(msg.get('sensorposition', ''))))
        elif name == 'misty_wizard_tts' and msg.get('utteranceid'):
            with self.tts_lock:
                self.tts_done.add(msg['utteranceid'])

    # ----------------------------------------------------------------- status
    def publish(self, topic, text):
        self.pub[topic].publish(String(data=text))

    def note(self, text):
        self.publish('last', f'{time.strftime("%H:%M:%S")}  {text}')

    def publish_session(self):
        if self.csv is not None:
            self.publish('session', f'REC  {self.participant} ({self.condition})')
        else:
            self.publish('session', 'Not recording: set participant_id + condition, press START')

    def publish_route(self):
        segs = self.cfg.get('route') or []
        if self.seg_i < len(segs):
            seg = segs[self.seg_i]
            if 'forward' in seg:
                what = f"forward {seg['forward']['velocity']}% {seg['forward']['time_ms']} ms"
            else:
                what = f"turn {seg['turn']['degrees']} deg"
            self.publish('route', f'Next segment {self.seg_i + 1}/{len(segs)}: {what}')
        else:
            self.publish('route', f'Route done ({len(segs)} segments). Route reset to start over.')

    def log(self, source, event, variant='', detail=''):
        self.get_logger().info(f'{source}:{event} {variant} {detail}'.rstrip())
        with self.log_lock:
            if self.csv is not None:
                self.csv.writerow({
                    'timestamp': datetime.now().isoformat(timespec='milliseconds'),
                    'participant_id': self.participant, 'condition': self.condition,
                    'event_source': source, 'event': event,
                    'utterance_variant_index': variant, 'detail': detail})
                self.csv_file.flush()

    # ---------------------------------------------------------- robot actions
    def call(self, what, fn, **kwargs):
        """Send a command; return True if Misty accepted it."""
        try:
            r = fn(**kwargs)
            try:
                body = r.json() if r.ok else {}
            except ValueError:
                body = {}
            if not isinstance(body, dict):
                body = {}
            if r.ok and body.get('status') in (None, 'Success'):
                return True
            err = body.get('error') or f'HTTP {r.status_code}'
            self.get_logger().warning(f'{what}: {err}')
            self.note(f'{what} failed: {err}')
        except Exception as e:
            self.get_logger().warning(f'{what} failed: {e}')
            self.note(f'{what} failed: {e}')
        return False

    def say(self, category, source='wizard', wait=False):
        variants = self.speech.get(category)
        if not variants:
            self.note(f'No speech category "{category}" in speech_bank.yaml')
            return
        choices = [i for i in range(len(variants)) if i != self.last_variant.get(category)] or [0]
        idx = random.choice(choices)
        self.last_variant[category] = idx
        text = variants[idx]
        uid = f'wiz_{uuid.uuid4().hex[:12]}'
        self.log(source, f'SAY_{category}', variant=idx, detail=text)
        self.publish('said', f'{category}[{idx}]: {text}')
        self.call('speak', self.misty.speak, text=text, flush=True, utteranceId=uid)
        if wait:
            sp = self.cfg['speech']
            estimate = len(text.split()) * sp['fallback_s_per_word'] + sp['fallback_extra_s']
            if self.ws_ok:
                def done():
                    with self.tts_lock:
                        return uid in self.tts_done
                self.sleep(estimate + sp['completion_margin_s'], until=done)
            else:
                self.sleep(estimate)

    def play(self, filename, source='wizard'):
        volume = int(self.sounds.get('volume', 80))
        self.log(source, 'PLAY_SOUND', detail=filename)
        self.publish('said', f'sound: {filename}')
        if self.call(f'Sound {filename}', self.misty.play_audio, fileName=filename, volume=volume):
            self.note(f'Playing {filename}')
        else:
            self.log('auto', 'PLAY_FAILED', detail=filename)

    def drive(self, label, **kw):
        if self.call('drive', self.misty.drive_time, **kw):
            self.note(label)

    def head(self, yaw):
        lb = self.cfg['look_back']
        self.call('move_head', self.misty.move_head, pitch=0.0, roll=0.0, yaw=float(yaw),
                  velocity=float(lb['head_velocity']), units='degrees')
        self.head_pub.publish(Vector3(x=0.0, y=0.0, z=float(yaw)))  # keep head teleop in sync

    def sleep(self, seconds, until=None):
        """Wait inside a sequence; raises Cancelled on STOP."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.cancel.is_set() or self.stop_flag.is_set():
                raise Cancelled()
            if until is not None and until():
                return
            time.sleep(0.05)

    def stop_everything(self, what):
        self.cancel.set()
        self.log('wizard', what)
        self.call('stop_speaking', self.misty.stop_speaking)
        self.call('stop_audio', self.misty.stop_audio)

    # ---------------------------------------------------------------- failure
    def start_failure(self):
        if self.sequence is not None and self.sequence.is_alive():
            self.note('FAILURE already running (STOP to cancel)')
            return
        self.cancel.clear()
        self.log('wizard', 'FAILURE')
        self.sequence = threading.Thread(target=self.run_failure, daemon=True)
        self.sequence.start()

    def run_failure(self):
        self.note('FAILURE sequence running')
        try:
            for step in self.cfg.get('failure_sequence') or []:
                if 'say' in step:
                    self.say(step['say'], source='failure', wait=True)
                elif 'play' in step or 'sound_group' in step:
                    if 'play' in step:
                        name = step['play']
                    else:
                        files = (self.sounds.get('groups') or {}).get(step['sound_group']) or []
                        if not files:
                            self.note(f'No sounds in group {step["sound_group"]}')
                            continue
                        name = random.choice(files)
                    self.play(name, source='failure')
                    self.sleep(float(step.get('wait', 2.0)))
                elif 'pause' in step:
                    self.sleep(float(step['pause']))
            self.log('failure', 'FAILURE_DONE')
            self.note('FAILURE sequence done')
        except Cancelled:
            self.log('failure', 'FAILURE_CANCELLED')
            self.note('FAILURE sequence cancelled')

    # ---------------------------------------------------------------- session
    def start_session(self):
        if self.csv is not None:
            self.note(f'Already recording {self.participant}; press END first')
            return
        pid = str(self.get_parameter('participant_id').value).strip()
        cond = str(self.get_parameter('condition').value).strip().upper()
        if not pid or cond not in CONDITIONS:
            self.note(f'Cannot START: set participant_id (now "{pid}") and condition '
                      f'CONTROL/BREAKDOWN (now "{cond}") in the Parameters panel.')
            return
        try:
            self.load_config()             # pick up edits to the YAML files
        except Exception as e:
            self.note(f'Cannot START: config error: {e}')
            return
        self.seg_i = 0
        self.publish_route()
        out = Path(self.get_parameter('session_dir').value).expanduser()
        out.mkdir(parents=True, exist_ok=True)
        path = out / f'{pid}_{cond}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        with self.log_lock:
            self.participant, self.condition = pid, cond
            self.csv_file = open(path, 'w', newline='')
            self.csv = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
            self.csv.writeheader()
        self.log('wizard', 'SESSION_START', detail=str(path))
        self.publish_session()
        self.note(f'Recording {pid} ({cond})')

    def end_session(self):
        if self.csv is None:
            self.note('No session running')
            return
        self.cancel.set()
        self.call('stop', self.misty.stop)
        self.log('wizard', 'SESSION_END')
        with self.log_lock:
            self.csv_file.close()
            self.csv_file = self.csv = None
            self.participant = self.condition = ''
        self.publish_session()
        self.note('Session saved')

    # ----------------------------------------------------------------- events
    def handle_event(self, name, source, arg):
        if name == 'TOUCH' or name == 'TOUCH_SIM':
            detail = arg or ('TOUCH_SIM' if name == 'TOUCH_SIM' else '')
            self.log(source, 'TOUCH', detail=detail)
            self.note(f'Head touched ({detail})')
        elif name == 'START':
            self.start_session()
        elif name == 'END':
            self.end_session()
        elif name == 'SAY':
            self.say(arg.upper())
        elif name == 'SAY_TEXT':
            if arg:
                self.log('wizard', 'SAY_CUSTOM', detail=arg)
                self.publish('said', f'custom: {arg}')
                self.call('speak', self.misty.speak, text=arg, flush=True)
        elif name == 'PLAY':
            if arg:
                self.play(arg)
        elif name == 'FAILURE':
            self.start_failure()
        elif name == 'STOP_TALKING':
            self.stop_everything('STOP_TALKING')
            self.note('Stopped talking + sounds')
        elif name == 'STOP':
            self.stop_everything('STOP')
            self.call('stop', self.misty.stop)
            self.note('STOPPED driving, speech and sounds')
        elif name == 'NEXT_SEGMENT':
            self.next_segment()
        elif name == 'ROUTE_RESET':
            self.seg_i = 0
            self.log('wizard', name)
            self.publish_route()
            self.note('Route reset to segment 1')
        elif name == 'LOOK_BACK':
            self.log('wizard', name)
            self.head(self.cfg['look_back']['head_yaw_deg'])
            self.say('LOOK_BACK')
            t = threading.Timer(float(self.cfg['look_back'].get('return_after_s', 4)), self.head, args=(0,))
            t.daemon = True
            t.start()
            self.note('Looking back')
        elif name == 'HEAD_CENTER':
            self.log('wizard', name)
            self.head(0)
            self.note('Head centred')
        elif name in ('NUDGE_L', 'NUDGE_R', 'NUDGE_FWD'):
            self.nudge(name)
        else:
            self.get_logger().warning(f'Unknown wizard event "{name}"')
            self.note(f'Unknown button "{name}"')

    def next_segment(self):
        segs = self.cfg.get('route') or []
        if self.seg_i >= len(segs):
            self.note('Route already finished: press Route reset')
            return
        seg = segs[self.seg_i]
        self.log('wizard', 'NEXT_SEGMENT', detail=f'segment {self.seg_i}: {seg}')
        if 'forward' in seg:
            f = seg['forward']
            self.drive(f'Segment {self.seg_i + 1}: forward', linearVelocity=float(f['velocity']),
                       angularVelocity=0.0, timeMs=int(f['time_ms']))
        elif 'turn' in seg:
            t = seg['turn']
            deg = float(t['degrees'])
            vel = abs(float(t.get('velocity', 30))) * (1 if deg >= 0 else -1)
            self.drive(f'Segment {self.seg_i + 1}: turn {deg:g} deg', linearVelocity=0.0,
                       angularVelocity=vel, timeMs=int(t['time_ms']), degree=deg)
        self.seg_i += 1
        self.publish_route()

    def nudge(self, name):
        n = self.cfg['nudge']
        self.log('wizard', name)
        if name == 'NUDGE_FWD':
            self.drive('Nudge forward', linearVelocity=float(n['forward_velocity']),
                       angularVelocity=0.0, timeMs=int(n['forward_ms']))
        else:
            sign = 1.0 if name == 'NUDGE_L' else -1.0  # + = left (counter-clockwise)
            self.drive(f'Nudge {"left" if sign > 0 else "right"}', linearVelocity=0.0,
                       angularVelocity=sign * float(n['turn_velocity']), timeMs=int(n['turn_ms']))

    def run(self):
        while not self.stop_flag.is_set():
            try:
                name, source, arg = self.events.get(timeout=0.25)
            except queue.Empty:
                continue
            try:
                self.handle_event(name, source, arg)
            except Exception as e:  # one bad press must not kill the menu
                self.get_logger().error(f'Error handling {name}: {e!r}')
                self.note(f'Error on {name}: {e}')

    def shutdown(self):
        self.stop_flag.set()
        self.cancel.set()
        self.worker.join(timeout=3.0)
        self.call('stop', self.misty.stop)
        with self.log_lock:
            if self.csv_file is not None:
                self.csv.writerow({'timestamp': datetime.now().isoformat(timespec='milliseconds'),
                                   'participant_id': self.participant, 'condition': self.condition,
                                   'event_source': 'auto', 'event': 'SESSION_END',
                                   'utterance_variant_index': '', 'detail': 'node shutdown'})
                self.csv_file.close()
                self.csv_file = self.csv = None


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
