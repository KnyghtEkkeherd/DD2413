"""Wizard-of-Oz actions for Misty, driven from Foxglove and the study state machine.

The wizard (Foxglove buttons) and study_node send std_msgs/String on /wizard/event:

  Session   START / END              open / close the CSV log (participant_id and
                                     condition parameters are read at START)
  Speech    REACT:<id>[:<n>]         a reaction from config/reactions.json: random (or
                                     n-th) sentence with its voice, a sound, or a sequence
            STUDY_SAY:<id>           same, then /wizard/done = <id> when it has finished
            SAY_TEXT:[voice] <text>  any text; PLAY:<file.wav> any audio file
            STOP_TALKING             cut off speech, sounds, sequences and gestures
  Emotion   EMOTION:<name>           face image, LEDs, optional arms / head / sound
                                     (emotions in config/study.yaml)
  Hands     GESTURE:<name>           arm sequence (gestures in config/study.yaml)
  Head      LOOK_BACK / HEAD_CENTER  head turns back (+ a line) / straight
  Study     YES, NO, DISTRACTED, DIR_OK, DIR_WRONG, RECOVER, CONTINUE, STATE:<name>,
            PAUSE, RESUME: handled by study_node, logged here
            STUDY_STATE:<from>><to>  state change (from study_node), logged
            STOP                     stop speech, sounds, sequences, gestures and the wheels
            TOUCH_SIM                log a head touch by hand
  Driving is NOT done here: the wizard drives with the controller / Foxglove DRIVE
  panel through twist_mux, which overrides Nav2.
  Eye expressions: reaction/button -> eye image preset (eyes in study.yaml); EMOTION:<name>
  sets the full emotion preset (face + LEDs + arms) from emotions in study.yaml.

Status (latched): /wizard/session, /wizard/said, /wizard/last, /wizard/condition
(condition while a session runs, '' otherwise); /wizard/done (speech finished),
/wizard/touch (head touches, from Misty's sensors or TOUCH_SIM).
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
LED_PATTERNS = {'breathe': 'Breathe', 'blink': 'Blink', 'transitonce': 'TransitOnce'}
STUDY_EVENTS = {'YES', 'NO', 'DISTRACTED', 'DIR_OK', 'DIR_WRONG', 'RECOVER', 'CONTINUE', 'STATE', 'PAUSE', 'RESUME'}
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
        self.declare_parameter('reactions', str(share / 'reactions.json'))
        self.declare_parameter('session_dir', str(Path.home() / 'misty_ws' / 'sessions'))
        self.declare_parameter('request_timeout', 2.0)
        self.declare_parameter('face_tracking_start_pitch_deg', -10.0)
        self.declare_parameter('face_tracking_start_velocity', 90.0)
        self.declare_parameter('face_tracking_aim_head', False)   # FACE_TRACK_ON: tilt the head up first
                                                                  # (only for face_tracker mode: head)

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
        self.sequence = None                  # running sequence (e.g. FAILURE) thread
        self.cancel = threading.Event()       # cancels the running sequence
        self.eye_lock = threading.Lock()      # serializes eye commands + reset timers
        self.eye_generation = 0               # prevents older timers resetting newer eyes
        self.eye_timer = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = {n: self.create_publisher(String, f'wizard/{n}', latched)
                    for n in ('session', 'said', 'last', 'condition')}
        self.done_pub = self.create_publisher(String, 'wizard/done', 10)
        self.touch_pub = self.create_publisher(String, 'wizard/touch', 10)
        self.head_pub = self.create_publisher(Vector3, 'misty/head_position', 10)
        self.head_goal_pub = self.create_publisher(Vector3, 'misty/head_goal', 10)
        self.arms_pub = self.create_publisher(Vector3, 'misty/arms_cmd', 10)
        self.create_subscription(String, 'wizard/event', self.on_wizard_msg, 10)

        self.publish_session()
        self.publish('condition', '')
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
        with open(self.get_parameter('study_config').value) as f:
            self.cfg = yaml.safe_load(f) or {}
        self.reactions_mtime = None
        self.load_reactions()

    def load_reactions(self):
        """(Re)load reactions.json if it changed. A broken file keeps the last good one."""
        path = Path(self.get_parameter('reactions').value)
        try:
            mtime = path.stat().st_mtime
            if mtime == getattr(self, 'reactions_mtime', None):
                return
            data = json.loads(path.read_text())
            reactions = {r['id']: r for r in data.get('reactions', [])}
            voices = {k: v for k, v in (data.get('voices') or {}).items() if isinstance(v, dict)}
        except (OSError, ValueError, KeyError, TypeError) as e:
            msg = f'reactions.json not loaded: {e}'
            self.get_logger().error(msg)
            if hasattr(self, 'pub'):
                self.note(msg)
            if not hasattr(self, 'reactions'):
                self.reactions, self.voices = {}, {}
            return
        self.reactions, self.voices, self.reactions_mtime = reactions, voices, mtime
        self.get_logger().info(f'Loaded {len(reactions)} reactions, {len(voices)} voices from {path}')

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

    # ------------------------------------------------------------ eye display
    def eye_file(self, emotion):
        """Resolve a named emotion (or filename) to an image stored on Misty."""
        config = self.cfg.get('eyes') or {}
        presets = config.get('emotions') or {}
        name = presets.get(emotion, emotion)
        if not isinstance(name, str) or not name.lower().endswith(('.jpg', '.jpeg', '.png', '.gif')):
            self.get_logger().warning(f'Unknown eye expression {emotion!r} in study.yaml')
            return None
        return name

    def show_eyes(self, emotion, hold_s=None):
        """Display eyes now; optionally revert after hold_s seconds.

        Returns a generation token so a long sequence can restore its own eyes
        without overwriting a newer button's expression.
        """
        filename = self.eye_file(emotion)
        if filename is None:
            return None
        with self.eye_lock:
            self.eye_generation += 1
            token = self.eye_generation
            if self.eye_timer is not None:
                self.eye_timer.cancel()
                self.eye_timer = None
            self.call(f'Eyes {emotion}', self.misty.display_image,
                      fileName=filename, alpha=1.0)
            if hold_s is not None and hold_s > 0:
                timer = threading.Timer(float(hold_s), self.reset_eyes, kwargs={'token': token})
                timer.daemon = True
                self.eye_timer = timer
                timer.start()
        return token

    def reset_eyes(self, token=None):
        """Restore neutral/default eyes unless a newer action superseded us."""
        config = self.cfg.get('eyes') or {}
        if not config:
            return
        filename = self.eye_file(config.get('default', 'neutral'))
        if filename is None:
            return
        with self.eye_lock:
            if token is not None and token != self.eye_generation:
                return
            self.eye_generation += 1
            if self.eye_timer is not None:
                self.eye_timer.cancel()
                self.eye_timer = None
            self.call('Restore eyes', self.misty.display_image,
                      fileName=filename, alpha=1.0)

    def eyes_for(self, category, key, timed=True):
        """Apply an optional eyes.reactions/eyes.buttons entry from study.yaml."""
        config = self.cfg.get('eyes') or {}
        emotion = (config.get(category) or {}).get(key)
        if emotion is None:
            return None
        hold_s = float(config.get('one_shot_hold_s', 4.0)) if timed else None
        return self.show_eyes(emotion, hold_s=hold_s)

    def voice_params(self, *specs):
        """Merge voice settings: later specs override earlier ones.

        A spec is a preset name from reactions.json "voices" or a dict with
        pitch / rate / voice / language (and optionally "voice": <preset>).
        """
        out = dict(self.voices.get('neutral', {}))
        for spec in specs:
            if not spec:
                continue
            if isinstance(spec, str):
                if spec not in self.voices:
                    self.note(f'Unknown voice preset "{spec}"')
                out.update(self.voices.get(spec, {}))
            elif isinstance(spec, dict):
                if isinstance(spec.get('voice'), str) and spec['voice'] in self.voices:
                    out.update(self.voices[spec['voice']])
                    spec = {k: v for k, v in spec.items() if k != 'voice'}
                out.update({k: v for k, v in spec.items() if k in ('pitch', 'rate', 'voice', 'language')})
        return out

    def speak(self, text, params, source, event, variant='', wait=False):
        uid = f'wiz_{uuid.uuid4().hex[:12]}'
        style = ', '.join(f'{k}={v}' for k, v in params.items() if v not in ('', None))
        self.log(source, event, variant=variant, detail=f'{text}  [{style}]' if style else text)
        self.publish('said', f'{event}{f"[{variant}]" if variant != "" else ""}: {text}')
        self.call('speak', self.misty.speak, text=text, flush=True, utteranceId=uid,
                  pitch=params.get('pitch'), speechRate=params.get('rate'),
                  voice=params.get('voice') or None, language=params.get('language') or None)
        if wait:
            sp = self.cfg['speech']
            rate = float(params.get('rate') or 1.0) or 1.0
            estimate = len(text.split()) * sp['fallback_s_per_word'] / rate + sp['fallback_extra_s']
            if self.ws_ok:
                def done():
                    with self.tts_lock:
                        return uid in self.tts_done
                self.sleep(estimate + sp['completion_margin_s'], until=done)
            else:
                self.sleep(estimate)

    def pick(self, rid, items, index=None):
        """Choose an item: the given index, else random without repeating the last."""
        if index is not None:
            if not 0 <= index < len(items):
                self.note(f'{rid} has no item {index}')
                return None, None
            return index, items[index]
        choices = [i for i in range(len(items)) if i != self.last_variant.get(rid)] or [0]
        i = random.choice(choices)
        self.last_variant[rid] = i
        return i, items[i]

    def say_reaction(self, rid, index=None, source='wizard', wait=False):
        r = self.reactions.get(rid)
        if not r or not r.get('sentences'):
            self.note(f'No sentences for reaction "{rid}" in reactions.json')
            return
        i, s = self.pick(rid, r['sentences'], index)
        if s is None:
            return
        text = s['text'] if isinstance(s, dict) else str(s)
        params = self.voice_params(r.get('voice'), {k: v for k, v in r.items() if k in ('pitch', 'rate', 'language')},
                                   s if isinstance(s, dict) else None)
        self.speak(text, params, source, f'SAY_{rid}', variant=i, wait=wait)

    def sound_reaction(self, rid, index=None, source='wizard'):
        r = self.reactions.get(rid)
        if not r or not r.get('sounds'):
            self.note(f'No sounds for reaction "{rid}" in reactions.json')
            return
        i, f = self.pick(rid, r['sounds'], index)
        if f is not None:
            self.play(f, source=source, volume=r.get('volume'), variant=i)

    def react(self, rid, index=None):
        self.load_reactions()
        r = self.reactions.get(rid)
        if r is None:
            self.note(f'Unknown reaction "{rid}" (not in reactions.json)')
            return
        if r.get('steps'):
            self.start_sequence(rid)
        elif r.get('sentences'):
            self.eyes_for('reactions', rid)
            self.say_reaction(rid, index)
        elif r.get('sounds'):
            self.eyes_for('reactions', rid)
            self.sound_reaction(rid, index)
        else:
            self.note(f'Reaction "{rid}" has no sentences, sounds or steps')

    def play(self, filename, source='wizard', volume=None, variant=''):
        volume = int(volume if volume is not None else 80)
        self.log(source, 'PLAY_SOUND', variant=variant, detail=filename)
        self.publish('said', f'sound: {filename}')
        if self.call(f'Sound {filename}', self.misty.play_audio, fileName=filename, volume=volume):
            self.note(f'Playing {filename}')
        else:
            self.log('auto', 'PLAY_FAILED', detail=filename)

    def head(self, yaw, pitch=0.0, velocity=None):
        lb = self.cfg['look_back']
        speed = float(lb['head_velocity']) if velocity is None else float(velocity)
        pitch = max(-40.0, min(26.0, float(pitch)))  # negative pitches look upward
        yaw = max(-81.0, min(81.0, float(yaw)))
        self.call('move_head', self.misty.move_head, pitch=pitch, roll=0.0, yaw=yaw,
                  velocity=speed, units='degrees')
        self.head_pub.publish(Vector3(x=pitch, y=0.0, z=yaw))  # keep teleop in sync

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
        self.reset_eyes()

    # -------------------------------------------------------------- sequences
    def start_sequence(self, rid):
        if self.sequence is not None and self.sequence.is_alive():
            self.note('A sequence is already running (Stop talking / STOP to cancel)')
            return
        self.cancel.clear()
        self.log('wizard', f'SEQUENCE_{rid}')
        eyes_token = self.eyes_for('reactions', rid, timed=False)
        self.sequence = threading.Thread(target=self.run_sequence, args=(rid, eyes_token), daemon=True)
        self.sequence.start()

    def run_sequence(self, rid, eyes_token=None):
        src = rid.lower()
        self.note(f'{rid} running')
        try:
            for step in self.reactions[rid]['steps']:
                if 'say' in step:
                    self.say_reaction(step['say'], step.get('index'), source=src, wait=True)
                elif 'text' in step:
                    self.speak(step['text'], self.voice_params(step.get('voice'), step), src, 'SAY_TEXT', wait=True)
                elif 'sound' in step or 'sound_from' in step:
                    if 'sound' in step:
                        self.play(step['sound'], source=src, volume=step.get('volume'))
                    else:
                        self.sound_reaction(step['sound_from'], step.get('index'), source=src)
                    self.sleep(float(step.get('wait', 2.0)))
                elif 'pause' in step:
                    self.sleep(float(step['pause']))
            self.log(src, f'SEQUENCE_{rid}_DONE')
            self.note(f'{rid} done')
        except Cancelled:
            self.log(src, f'SEQUENCE_{rid}_CANCELLED')
            self.note(f'{rid} cancelled')
        finally:
            if eyes_token is not None:
                self.reset_eyes(token=eyes_token)

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
            self.note(f'Cannot START: study.yaml error: {e}')
            return
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
        self.publish('condition', cond)            # study_node starts on this
        self.note(f'Recording {pid} ({cond})')

    def end_session(self):
        if self.csv is None:
            self.note('No session running')
            return
        self.cancel.set()
        self.call('stop', self.misty.stop)
        self.reset_eyes()
        self.log('wizard', 'SESSION_END')
        self.publish('condition', '')
        with self.log_lock:
            self.csv_file.close()
            self.csv_file = self.csv = None
            self.participant = self.condition = ''
        self.publish_session()
        self.note('Session saved')

    # ----------------------------------------------------------------- events
    def handle_event(self, name, source, arg):
        # Optional YAML mappings let non-reaction buttons (e.g. LOOK_BACK)
        # change eyes as well. REACT/FAILURE use reaction-specific handling.
        if name not in ('REACT', 'SAY', 'FAILURE', 'STOP', 'STOP_TALKING'):
            self.eyes_for('buttons', name)
        if name == 'TOUCH' or name == 'TOUCH_SIM':
            detail = arg or ('TOUCH_SIM' if name == 'TOUCH_SIM' else '')
            self.log(source, 'TOUCH', detail=detail)
            self.note(f'Head touched ({detail})')
            self.touch_pub.publish(String(data=detail))
        elif name == 'STUDY_SAY':
            threading.Thread(target=self.study_say, args=(arg.strip(),), daemon=True).start()
        elif name == 'EMOTION':
            self.emotion(arg.strip().upper())
        elif name == 'GESTURE':
            self.gesture(arg.strip().upper())
        elif name == 'STUDY_STATE':
            self.log('study', 'STATE', detail=arg)
            self.note(f'Study state: {arg}')
        elif name in STUDY_EVENTS or name.startswith('NAV'):
            self.log(source, name, detail=arg)        # handled by study_node / mission
        elif name == 'START':
            self.start_session()
        elif name == 'END':
            self.end_session()
        elif name in ('REACT', 'SAY'):
            rid, _, idx = arg.partition(':')
            self.react(rid.strip(), int(idx) if idx.strip().isdigit() else None)
        elif name in ('FACE_TRACK_ON', 'FACE_TRACK_OFF'):
            self.log(source, name)
            if name == 'FACE_TRACK_ON' and self.get_parameter('face_tracking_aim_head').value:
                # Aim slightly up and straight ahead before auto-tracking starts.
                self.head(0.0,
                          pitch=self.get_parameter('face_tracking_start_pitch_deg').value,
                          velocity=self.get_parameter('face_tracking_start_velocity').value)
            self.note(f'{name}: see face tracking status panel')
        elif name == 'FAILURE':
            self.react('FAILURE')
        elif name == 'SAY_TEXT':
            # "[sad] text", "[pitch=1.4 rate=0.8] text" or "[sad rate=0.6] text"
            text, preset, extra = arg, None, {}
            if text.startswith('[') and ']' in text:
                spec, text = text[1:text.index(']')], text[text.index(']') + 1:].strip()
                for tok in spec.replace(',', ' ').split():
                    key, eq, val = tok.partition('=')
                    if not eq:
                        preset = key
                    elif key in ('pitch', 'rate'):
                        try:
                            extra[key] = float(val)
                        except ValueError:
                            self.note(f'Bad number in "{tok}"')
                    elif key in ('voice', 'language'):
                        extra[key] = val
            if text:
                self.load_reactions()
                self.speak(text, self.voice_params(preset, extra), 'wizard', 'SAY_CUSTOM')
        elif name == 'PLAY':
            if arg:
                self.play(arg)
        elif name == 'STOP_TALKING':
            self.stop_everything('STOP_TALKING')
            self.note('Stopped talking + sounds')
        elif name == 'STOP':
            self.stop_everything('STOP')
            self.call('stop', self.misty.stop)
            self.note('STOPPED: wheels, speech, sounds, gestures; study + Nav2 paused (RESUME to go on)')
        elif name == 'LOOK_BACK':
            self.log('wizard', name)
            self.head(self.cfg['look_back']['head_yaw_deg'])
            self.load_reactions()
            if self.cfg['look_back'].get('say', 'LOOK_BACK'):
                self.say_reaction(self.cfg['look_back'].get('say', 'LOOK_BACK'))
            t = threading.Timer(float(self.cfg['look_back'].get('return_after_s', 4)), self.head, args=(0,))
            t.daemon = True
            t.start()
            self.note('Looking back')
        elif name == 'HEAD_CENTER':
            self.log('wizard', name)
            self.head(0)
            self.note('Head centred')
        else:
            self.get_logger().warning(f'Unknown wizard event "{name}"')
            self.note(f'Unknown button "{name}"')

    # ---------------------------------------------------- study helpers
    def study_say(self, rid):
        """Speak / play a reaction for the study and report when it's done (/wizard/done)."""
        self.load_reactions()
        r = self.reactions.get(rid)
        self.cancel.clear()               # a new study step: an earlier STOP mustn't cut it short
        try:
            if r is None:
                self.note(f'Unknown reaction "{rid}" (not in reactions.json)')
            elif r.get('steps'):
                self.run_sequence(rid)
            elif r.get('sentences'):
                self.say_reaction(rid, source='study', wait=True)
            elif r.get('sounds'):
                self.sound_reaction(rid, source='study')
                self.sleep(float(r.get('wait', 2.0)))
        except Cancelled:
            pass
        except Exception as e:
            self.get_logger().error(f'study_say {rid}: {e!r}')
        self.done_pub.publish(String(data=rid))

    def emotion(self, name):
        e = (self.cfg.get('emotions') or {}).get(name)
        if e is None:
            self.note(f'Unknown emotion "{name}" (emotions in study.yaml)')
            return
        self.log('wizard', f'EMOTION_{name}')
        if e.get('face'):
            self.call('display_image', self.misty.display_image, fileName=e['face'], alpha=1.0)
        if e.get('led'):
            r, g, b = (int(v) for v in e['led'])
            if e.get('led2') and e.get('pattern'):
                r2, g2, b2 = (int(v) for v in e['led2'])
                self.call('transition_led', self.misty.transition_led, red=r, green=g, blue=b, red2=r2,
                          green2=g2, blue2=b2, transitionType=LED_PATTERNS.get(str(e['pattern']).lower(), 'Breathe'),
                          timeMs=int(e.get('led_ms', 800)))
            else:
                self.call('change_led', self.misty.change_led, red=r, green=g, blue=b)
        if e.get('arms'):
            self.arms(*e['arms'])
        if e.get('head'):
            self.head_goal(*e['head'])
        if e.get('sound'):
            self.load_reactions()
            self.sound_reaction(e['sound'])
        self.note(f'Emotion {name}')

    def arms(self, left, right):
        nan = float('nan')
        self.arms_pub.publish(Vector3(x=nan if left is None else float(left),
                                      y=nan if right is None else float(right), z=0.0))

    def head_goal(self, pitch, yaw):
        self.head_goal_pub.publish(Vector3(x=float(pitch), y=0.0, z=float(yaw)))

    def gesture(self, name):
        steps = (self.cfg.get('gestures') or {}).get(name)
        if not steps:
            self.note(f'Unknown gesture "{name}" (gestures in study.yaml)')
            return
        self.log('wizard', f'GESTURE_{name}')

        def run():
            try:
                for st in steps:
                    if st.get('arms'):
                        self.arms(*st['arms'])
                    self.sleep(float(st.get('wait', 0.5)))
            except Cancelled:
                pass
        self.cancel.clear()
        threading.Thread(target=run, daemon=True).start()
        self.note(f'Gesture {name}')

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
        self.reset_eyes()
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
