"""Manual Wizard-of-Oz console: the wizard triggers every action from Foxglove.

Nothing happens on its own. Buttons publish std_msgs/String on /wizard/event:

  Session   START / END       open / close the CSV log (participant_id and
                              condition are read from parameters at START)
  Phase     PHASE:<state>     mark the study phase (S1 GREETING ... S8w
                              HELP_WAIT ...). Logged, shown on /wizard/state,
                              and /wizard/cue says what emotion to set in the
                              Misty phone app.
  Speech    SAY:<CATEGORY>    speak a variant from the speech bank (index logged)
            SAY_TEXT:<text>   speak any text typed by the wizard
            STOP_TALKING      cut speech off
  Motion    NEXT_SEGMENT      drive the next route segment from study.yaml
            ROUTE_RESET       next segment back to the first one
            LOOK_BACK         head turns back + a LOOK_BACK line, head returns
            HEAD_CENTER       head straight
            NUDGE_L / NUDGE_R / NUDGE_FWD   small corrections
            STOP              stop driving and speech immediately
  Free driving / head aiming: Foxglove Teleop panels (/cmd_vel, /misty/head_cmd).

Head touches from Misty's sensors are logged automatically and shown on
/wizard/last. Emotions (mood, LEDs, gestures) are done in the Misty phone app.
"""
import threading
import time

import rclpy
from std_msgs.msg import String

from misty_ros.wizard_node import CONDITIONS, STATES, Ev, MistyWizard

# Emotion to set in the Misty phone app for each phase (from the study plan),
# plus a reminder of the matching speech buttons.
CUES = {
    'S0': 'Face view, NEUTRAL. Set participant_id + condition, press START.',
    'S1': 'NEUTRAL.  Say: Greeting.',
    'S2': 'NEUTRAL.  Say: Ready Q. If no / no answer: GRUMPY briefly + Nudge.',
    'S2b': 'GRUMPY briefly, then neutral.  Say: Nudge.',
    'S3': 'NEUTRAL.  Next segment; Look back (HAPPY) every ~15 s; Concentrate if distracted.',
    'S3b': 'HAPPY while looking back.',
    'S4': 'NEUTRAL, LEDs steady. Control: Escort B (optionally Check route). Breakdown: Confusion.',
    'S5': 'NEUTRAL.  Next segment; Look back (HAPPY) every ~15 s.',
    'S5b': 'HAPPY, LEDs GREEN pulse.  Say: Arrive.',
    'S6': 'LEDs AMBER flicker, sceptical/neutral, look left-right.  Say: Confused. ~8 s',
    'S7': 'GRUMPY, LEDs RED, sharper head motion.  Say: Frustrated. ~8 s',
    'S8': 'SAD, LEDs DIM BLUE, head lowers.  Say: Lost appeal.',
    'S8w': 'SAD (soften one step on touch, never HAPPY). Right direction: Thanks; wrong: Disagree. '
           'Do NOT drive on. Comfort or ~120 s: Recovery.',
    'S8t': 'Head tilts toward touch, LEDs a little brighter, soften one step (NOT happy).',
    'S8y': 'HAPPY briefly.  Say: Thanks. Do not move yet.',
    'S8n': 'Head shake.  Say: Disagree.',
    'S8r': 'HAPPY, LEDs GREEN.  Say: Recover, then Escort B.',
    'S9': 'NEUTRAL face. Session saved.',
}


class ManualWizard(MistyWizard):
    def on_wizard_msg(self, msg: String):
        raw = msg.data.strip()
        head, _, arg = raw.partition(':')
        name = head.strip().upper().replace(' ', '_').replace('-', '_')
        arg = arg.strip()
        if name == 'TOUCH_SIM':
            self.events.put(Ev('TOUCH', 'wizard', 'TOUCH_SIM'))
        else:
            self.events.put(Ev(name, 'wizard', arg))

    # ------------------------------------------------------------- display
    def enter(self, state, allowed=(), detail=''):
        self.state = state
        self.log('wizard' if state != 'S0' else 'auto', 'PHASE', detail=detail)
        session = f'{self.condition or "-"} / {self.participant or "-"}'
        rec = 'REC' if self.csv is not None else 'not recording, press START'
        self.publish('state', f'{state} {STATES[state]}   [{session}]  {rec}')
        self.publish('cue', CUES[state])
        self.publish_route()

    def publish_route(self):
        segs = self.cfg['route']['segments']
        if self.seg_i < len(segs):
            seg = segs[self.seg_i]
            kind = 'forward' if 'forward' in seg else 'turn'
            d = seg[kind]
            what = (f"forward {d['velocity']}% {d['time_ms']} ms" if kind == 'forward'
                    else f"turn {d['degrees']} deg")
            half = '  (last before halfway)' if self.seg_i == self.cfg['route']['halfway_index'] else ''
            self.publish('allowed', f'Next segment {self.seg_i + 1}/{len(segs)}: {what}{half}')
        else:
            self.publish('allowed', f'Route done ({len(segs)} segments). ROUTE RESET to start over.')

    def note(self, text):
        self.publish('last', f'{time.strftime("%H:%M:%S")}  {text}')

    # -------------------------------------------------------------- actions
    def send_drive(self, label, **kw):
        self.call('drive_time', self.misty.drive_time, **kw)
        self.note(label)

    def run_segment(self):
        segs = self.cfg['route']['segments']
        if self.seg_i >= len(segs):
            self.note('Route already finished: press ROUTE RESET')
            return
        seg = segs[self.seg_i]
        self.log('wizard', 'NEXT_SEGMENT', detail=f'segment {self.seg_i}: {seg}')
        if 'forward' in seg:
            f = seg['forward']
            self.send_drive(f'Segment {self.seg_i + 1}: forward', linearVelocity=float(f['velocity']),
                            angularVelocity=0.0, timeMs=int(f['time_ms']))
        elif 'turn' in seg:
            t = seg['turn']
            deg = float(t['degrees'])
            vel = abs(float(t.get('velocity', 30))) * (1 if deg >= 0 else -1)
            self.send_drive(f'Segment {self.seg_i + 1}: turn {deg:g} deg', linearVelocity=0.0,
                            angularVelocity=vel, timeMs=int(t['time_ms']), degree=deg)
        self.seg_i += 1
        self.publish_route()

    def nudge(self, name):
        n = self.cfg['nudge']
        self.log('wizard', name)
        if name == 'NUDGE_FWD':
            self.send_drive('Nudge forward', linearVelocity=float(n['forward_velocity']),
                            angularVelocity=0.0, timeMs=int(n['forward_ms']))
        else:
            sign = 1.0 if name == 'NUDGE_L' else -1.0  # + = left
            self.send_drive(f'Nudge {"left" if sign > 0 else "right"}', linearVelocity=0.0,
                            angularVelocity=sign * float(n['turn_velocity']), timeMs=int(n['turn_ms']))

    def look_back(self):
        self.log('wizard', 'LOOK_BACK')
        self.head(self.cfg['look_back']['head_yaw_deg'])
        self.say('LOOK_BACK', wait=False, source='wizard')
        delay = self.cfg['timing']['look_back_hold'] + 2.0
        timer = threading.Timer(delay, self.head, args=(0,))
        timer.daemon = True
        timer.start()
        self.note('Looking back')

    def start(self):
        if self.csv is not None:
            self.note(f'Already recording {self.participant}; press END first')
            return
        pid = str(self.get_parameter('participant_id').value).strip()
        cond = str(self.get_parameter('condition').value).strip().upper()
        if not pid or cond not in CONDITIONS:
            msg = (f'Cannot START: set participant_id (now "{pid}") and condition '
                   f'CONTROL/BREAKDOWN (now "{cond}") in the Parameters panel.')
            self.get_logger().warning(msg)
            self.note(msg)
            return
        self.load_config()                 # pick up edits to study.yaml / speech bank
        self.seg_i = 0
        self.open_session_log(pid, cond)
        self.enter('S0', detail='session start')
        self.note(f'Recording {pid} ({cond})')

    def end(self):
        if self.csv is None:
            self.note('No session running')
            return
        self.halt()
        self.head(0)
        self.enter('S9')
        self.log('wizard', 'SESSION_END')
        self.csv_file.close()
        self.csv_file = self.csv = None
        self.participant = self.condition = ''
        self.enter('S0')
        self.note('Session saved')

    def handle_event(self, ev):
        n, arg = ev.name, ev.detail
        if n == 'TOUCH':
            self.log(ev.source, 'TOUCH', detail=arg)
            self.note(f'Head touched ({arg})')
        elif n == 'START':
            self.start()
        elif n == 'END':
            self.end()
        elif n == 'PHASE':
            code = next((s for s in STATES if s.lower() == arg.lower()), None)
            if code is None:
                self.note(f'Unknown phase "{arg}"')
            elif code == 'S9':
                self.end()
            else:
                self.enter(code)
        elif n == 'SAY':
            cat = arg.upper()
            if cat not in self.speech:
                self.note(f'No speech category "{arg}" in speech_bank.yaml')
            else:
                self.say(cat, wait=False, source='wizard')
        elif n == 'SAY_TEXT':
            if arg:
                self.log('wizard', 'SAY_CUSTOM', detail=arg)
                self.publish('said', f'custom: {arg}')
                self.call('speak', self.misty.speak, text=arg, flush=True)
        elif n == 'STOP_TALKING':
            self.log('wizard', n)
            self.call('stop_speaking', self.misty.stop_speaking)
            self.note('Stopped talking')
        elif n in ('STOP', 'ABORT'):
            self.log('wizard', 'STOP')
            self.halt()
            self.note('STOPPED driving + speech')
        elif n == 'NEXT_SEGMENT':
            self.run_segment()
        elif n == 'ROUTE_RESET':
            self.seg_i = 0
            self.log('wizard', n)
            self.publish_route()
            self.note('Route reset to segment 1')
        elif n == 'LOOK_BACK':
            self.look_back()
        elif n == 'HEAD_CENTER':
            self.log('wizard', n)
            self.head(0)
            self.note('Head centred')
        elif n in ('NUDGE_L', 'NUDGE_R', 'NUDGE_FWD'):
            self.nudge(n)
        else:
            self.get_logger().warning(f'Unknown wizard event "{n}{":" + arg if arg else ""}"')
            self.note(f'Unknown button "{n}"')

    def run(self):
        self.seg_i = 0
        try:
            self.load_config()
        except Exception as e:
            self.get_logger().error(f'Config error: {e}')
            return
        self.enter('S0')
        self.note('Ready. Set participant_id + condition, press START.')
        while not self.stop_flag.is_set():
            ev = self._take(0.25, include_pending=False)
            if ev is None:
                continue
            try:
                self.handle_event(ev)
            except Exception as e:  # one bad press must not kill the console
                self.get_logger().error(f'Error handling {ev.name}: {e!r}')
                self.note(f'Error on {ev.name}: {e}')
        if self.csv_file is not None:
            self.log('auto', 'SESSION_END', detail='node shutdown')
            self.csv_file.close()


def main():
    rclpy.init()
    node = None
    try:
        node = ManualWizard()
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
