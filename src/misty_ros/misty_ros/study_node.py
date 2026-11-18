"""Study state machine (config/study.yaml -> study.states), from the study plan.

Starts when the wizard starts a session (START: /wizard/condition becomes CONTROL or
BREAKDOWN) in study.first_state, and stops at END. Each state's entry actions are
sent as wizard events (EMOTION, GESTURE, STUDY_SAY, LOOK_BACK, END) and Nav2
targets (NAV_GOTO:<marker>); transitions come from

    arrive          Nav2 reached the state's marker (/nav/arrived)
    after_speech    everything in `say` was spoken (/wizard/done)
    after, timeout  time in the state
    events          wizard buttons (YES, NO, DIR_OK, ...) and head touches (/wizard/touch)
    max_visits      loop limit (e.g. at most 3 nudges)

The wizard can jump to any state (STATE:<name>), skip ahead (CONTINUE), and pause /
resume everything (STOP or PAUSE / RESUME): timers freeze, Nav2 is held, the
emergency lock blocks Nav2 and this node's motions in twist_mux; the wizard can still
drive by hand. States without `nav` hold Nav2 still (/study/hold_nav lock).
Every state change is logged in the session CSV (STUDY_STATE).
Status: /study/state (latched).
"""
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

BUTTONS = {'YES', 'NO', 'DISTRACTED', 'DIR_OK', 'DIR_WRONG', 'RECOVER', 'CONTINUE'}
HEAD_MOVES = {   # (seconds from entry, pitch, yaw)
    'sweep': [(0.0, 0, -35), (1.4, 0, 35), (2.8, 0, -20), (4.0, 0, 0)],
    'shake': [(0.0, 5, -20), (0.35, 5, 20), (0.7, 5, -20), (1.05, 5, 20), (1.4, 0, 0)],
}


class Study(Node):
    def __init__(self):
        super().__init__('misty_study')
        share = Path(get_package_share_directory('misty_ros')) / 'config'
        self.declare_parameter('study_config', str(share / 'study.yaml'))
        self.declare_parameter('say_timeout_s', 40.0)    # give up waiting for /wizard/done
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ev = self.create_publisher(String, 'wizard/event', 10)
        self.state_pub = self.create_publisher(String, 'study/state', latched)
        self.hold_pub = self.create_publisher(Bool, 'study/hold_nav', latched)
        self.estop_pub = self.create_publisher(Bool, 'wizard/estop', latched)
        self.head_pub = self.create_publisher(Vector3, 'misty/head_goal', 10)
        self.create_subscription(String, 'wizard/event', self.on_event, 20)
        self.create_subscription(String, 'wizard/done', self.on_done, 10)
        self.create_subscription(String, 'wizard/touch', lambda m: self.trigger('TOUCH'), 10)
        self.create_subscription(String, 'wizard/condition', self.on_condition, latched)
        self.create_subscription(String, 'nav/arrived', self.on_arrived, 10)
        self.create_subscription(String, 'nav/failed', self.on_failed, 10)

        self.load()
        self.state, self.condition = 'IDLE', ''
        self.return_to, self.return_entered = None, 0.0
        self.visits = {}
        self.paused_at = None
        self.pending_arrival = None
        self.reset_state_vars()
        self.hold(False)
        self.estop_pub.publish(Bool(data=False))
        self.publish_state()
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'Study state machine: {len(self.states)} states; waiting for START')

    # ------------------------------------------------------------- helpers
    def load(self):
        cfg = yaml.safe_load(open(self.get_parameter('study_config').value)) or {}
        self.cfg = cfg.get('study') or {}
        self.states = {str(k): self.clean(v or {}) for k, v in (self.cfg.get('states') or {}).items()}
        self.look_back_return = float((cfg.get('look_back') or {}).get('return_after_s', 4))

    @staticmethod
    def clean(c):
        """YAML 1.1 reads unquoted yes / no / on as booleans: map them back to names."""
        name = lambda k: {True: 'YES', False: 'NO'}.get(k, k) if isinstance(k, bool) else str(k).upper()
        c = {('events' if k is True else k): v for k, v in c.items()}       # unquoted `on:`
        for key in ('events', 'do'):
            if isinstance(c.get(key), dict):
                c[key] = {name(k): v for k, v in c[key].items()}
        return c

    def reset_state_vars(self):
        self.entered = time.monotonic()
        self.say_queue, self.say_waiting, self.say_sent_at, self.speech_done = [], None, 0.0, None
        self.head_plan = []
        self.next_look_back = None
        self.look_back_until = None
        self.note = ''

    def send(self, text):
        self.ev.publish(String(data=text))

    def hold(self, on):
        self.hold_pub.publish(Bool(data=bool(on)))

    def publish_state(self):
        if self.state == 'IDLE':
            text = 'IDLE: set participant + condition, press START'
        else:
            text = f'{self.state}  ({self.condition})'
            if self.paused_at is not None:
                text += '  PAUSED: press RESUME'
            if self.note:
                text += f'  · {self.note}'
        self.state_pub.publish(String(data=text))

    def cur(self):
        return self.states.get(self.state) or {}

    # ------------------------------------------------------------ inputs
    def on_condition(self, msg):
        cond = msg.data.strip().upper()
        if cond and self.state == 'IDLE':
            self.condition = cond
            self.load()                                   # pick up edits made between sessions
            self.visits, self.return_to = {}, None
            self.enter(self.cfg.get('first_state', 'APPROACH'), 'START')
        elif not cond and self.state != 'IDLE':
            self.send('NAV_RESET')
            self.state, self.condition, self.paused_at = 'IDLE', '', None
            self.reset_state_vars()
            self.hold(False)
            self.estop_pub.publish(Bool(data=False))
            self.publish_state()

    def on_event(self, msg):
        head, _, arg = msg.data.strip().partition(':')
        name = head.strip().upper()
        if name == 'STOP':                             # the emergency lock works with or without a session
            self.estop_pub.publish(Bool(data=True))
        elif name == 'RESUME':
            self.estop_pub.publish(Bool(data=False))
        if self.state == 'IDLE':
            return
        if name == 'STATE':
            target = arg.strip().upper()
            if target in self.states:
                if self.paused_at is not None:
                    self.resume()
                self.enter(target, 'wizard')
            else:
                self.get_logger().warning(f'unknown state {target}')
        elif name in ('STOP', 'PAUSE'):
            self.pause(estop=(name == 'STOP'))
        elif name == 'RESUME':
            self.resume()
        elif name in BUTTONS:
            self.trigger(name)

    def on_done(self, msg):
        if self.say_waiting is not None and msg.data.strip() == self.say_waiting:
            self.say_waiting = None
            self.next_say()

    def on_arrived(self, msg):
        if self.paused_at is not None:                 # apply after RESUME
            self.pending_arrival = msg
            return
        c = self.cur()
        if c.get('nav') and msg.data.strip() == str(self.marker(c['nav'])) and c.get('arrive'):
            self.go(c['arrive'], 'arrived')

    def on_failed(self, msg):
        if self.cur().get('nav'):
            self.note = (f'NAV FAILED to marker {msg.data}: drive closer by hand, then press '
                         f'the {self.state} state button to retry')
            self.publish_state()

    def marker(self, role):
        return (self.cfg.get('markers') or {}).get(role, role)

    # ------------------------------------------------------------ transitions
    def trigger(self, event):
        if self.state == 'IDLE' or self.paused_at is not None:
            return
        c = self.cur()
        if event in (c.get('events') or {}):
            self.go(c['events'][event], event)
        elif event in (c.get('do') or {}):
            self.send(f'REACT:{c["do"][event]}')
        elif event == 'CONTINUE':
            self.go(self.default_next(c), 'CONTINUE')
        elif event == 'TOUCH' and self.cfg.get('touch_reaction'):
            self.send(f'REACT:{self.cfg["touch_reaction"]}')

    def default_next(self, c):
        for key in ('arrive', 'after_speech'):
            if c.get(key):
                return c[key]
        for key in ('after', 'timeout'):
            if c.get(key):
                return c[key][1]
        return None

    def go(self, target, via):
        if target is None:                            # decided by the condition
            target = (self.cur().get('by_condition') or {}).get(self.condition)
            if target is None:
                self.get_logger().warning(f'{self.state}: no next state for condition {self.condition}')
                return
        if target == 'RETURN':
            target, via = self.return_to or self.state, f'RETURN after {via}'
        self.enter(target, via)

    def enter(self, name, via):
        c = self.states.get(name)
        if c is None:
            self.get_logger().error(f'unknown state {name}')
            return
        n = self.visits.get(name, 0) + 1
        if c.get('max_visits') and n > int(c['max_visits'][0]) and via != 'wizard':
            return self.enter(c['max_visits'][1], f'{via}, {name} limit')
        self.visits[name] = n
        was_nav = bool(self.cur().get('nav')) if self.state != 'IDLE' else False
        resume_clock = None
        if self.returns(c) and not self.returns(self.cur()):
            self.return_to = self.state                # sub-state: remember where to go back to
            self.return_entered = self.entered         # ... and how long we'd been there
        elif via.startswith('RETURN') and name == self.return_to:
            resume_clock = self.return_entered         # back from a sub-state: keep the parent's
                                                       # clock (e.g. HELP_WAIT's 120 s timeout)
        old, self.state = self.state, name
        self.reset_state_vars()
        if resume_clock is not None:
            self.entered = resume_clock
        self.send(f'STUDY_STATE:{old}>{name} ({via})')
        self.get_logger().info(f'{old} -> {name} ({via})')

        if c.get('emotion'):
            self.send(f'EMOTION:{c["emotion"]}')
        if c.get('gesture'):
            self.send(f'GESTURE:{c["gesture"]}')
        head = c.get('head')
        if isinstance(head, list):
            self.head_pub.publish(Vector3(x=float(head[0]), y=0.0, z=float(head[1])))
        elif head in HEAD_MOVES:
            self.head_plan = [(self.entered + t, p, y) for t, p, y in HEAD_MOVES[head]]
        if c.get('nav'):
            self.hold(False)
            self.send(f'NAV_GOTO:{self.marker(c["nav"])}')
            if c.get('look_back'):
                self.next_look_back = self.entered + float(self.cfg.get('look_back_every_s', 15))
        else:
            if was_nav:
                self.send('NAV_RESET')
            self.hold(True)
        says = c.get('say')
        self.say_queue = list(says) if isinstance(says, list) else ([says] if says else [])
        self.next_say()
        if c.get('end_session'):
            self.send('END')
        self.publish_state()

    def returns(self, c):
        targets = list((c.get('events') or {}).values()) + [c.get('after_speech'), c.get('arrive')]
        targets += [c[k][1] for k in ('after', 'timeout') if c.get(k)]
        return 'RETURN' in targets

    def next_say(self):
        if self.say_queue:
            self.say_waiting = self.say_queue.pop(0)
            self.say_sent_at = time.monotonic()
            self.send(f'STUDY_SAY:{self.say_waiting}')
        elif self.speech_done is None:
            self.speech_done = time.monotonic()

    # ------------------------------------------------------------ pause
    def pause(self, estop):
        if self.paused_at is None:
            self.paused_at = time.monotonic()
            self.hold(True)
            self.send('NAV_PAUSE')
        if estop:
            self.estop_pub.publish(Bool(data=True))
        self.publish_state()

    def resume(self):
        self.estop_pub.publish(Bool(data=False))
        if self.paused_at is None:
            return
        dt = time.monotonic() - self.paused_at
        self.paused_at = None
        self.entered += dt                            # freeze timers while paused
        self.say_sent_at += dt
        if self.speech_done is not None:
            self.speech_done += dt
        if self.next_look_back is not None:
            self.next_look_back += dt
        if self.look_back_until is not None:
            self.look_back_until += dt
        self.head_plan = [(t + dt, p, y) for t, p, y in self.head_plan]
        pending, self.pending_arrival = self.pending_arrival, None
        if self.cur().get('nav') and self.look_back_until is None:
            self.hold(False)
            self.send('NAV_GO')
        self.publish_state()
        if pending is not None:
            self.on_arrived(pending)

    # ------------------------------------------------------------ timing
    def tick(self):
        if self.state == 'IDLE' or self.paused_at is not None:
            return
        now = time.monotonic()
        c = self.cur()
        while self.head_plan and self.head_plan[0][0] <= now:
            _, p, y = self.head_plan.pop(0)
            self.head_pub.publish(Vector3(x=float(p), y=0.0, z=float(y)))
        if self.say_waiting is not None and now - self.say_sent_at > float(self.get_parameter('say_timeout_s').value):
            self.get_logger().warning(f'no /wizard/done for {self.say_waiting}: going on')
            self.say_waiting = None
            self.next_say()
        # automatic look-back while escorting: stop, look, resume
        if self.look_back_until is not None and now >= self.look_back_until:
            self.look_back_until = None
            self.next_look_back = now + float(self.cfg.get('look_back_every_s', 15))
            self.hold(False)
            self.send('NAV_GO')
        elif self.next_look_back is not None and self.look_back_until is None and now >= self.next_look_back:
            self.look_back_until = now + self.look_back_return + 1.0
            self.hold(True)
            self.send('NAV_PAUSE')
            self.send('LOOK_BACK')
        for key in ('after', 'timeout'):
            if c.get(key) and now - self.entered >= float(c[key][0]):
                return self.go(c[key][1], key)
        if 'after_speech' in c:
            if self.speech_done is not None and now - self.speech_done >= float(c.get('after_speech_pause_s', 0.0)):
                return self.go(c.get('after_speech'), 'speech done')


def main():
    rclpy.init()
    node = Study()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
