"""Marker follower: lock onto the target ArUco marker and drive straight at it.

Default navigation of study.launch.py (nav:=follow). No map: steering comes from
the detection (bearing and distance in the camera frame, plus the measured head yaw,
minus what Misty has turned since the detection: detections arrive late).

    in view, within turn_only_deg   drive towards it, steering (at least min_angular,
                                    smaller doesn't turn Misty); straight within
                                    center_deadband_deg; stop standoff_m in front of it
                                    -> /nav/arrived
    in view, further off-centre     turn part of the way in place (IMU), stand still
                                    (settle_s), decide again on a fresh detection
    lost right after an overshoot   one turn back to where it must be (IMU), then look
    new target not in view          turn continuously in place until it's seen (one
                                    full turn at most); then stand still and wait for the
                                    wizard to steer Misty until she sees it
    obstacle (front ToF < stop_m)   simple avoidance: turn avoid_turn_deg away from it
                                    (towards the freer side if the side ToFs say so),
                                    drive avoid_forward_m, then turn back / search
The head is kept neutral while driving (the camera is in the head). Face tracking
(face_tracker) is switched off when a drive starts and on when Misty has stopped at
the marker (FACE_TRACK_OFF / FACE_TRACK_ON on /wizard/event; face_tracking: false
leaves it to the wizard). The wizard's
controller / Foxglove DRIVE always override this node (twist_mux); B / STOP pauses it,
START / RESUME (or NAV_GO) continues.

Same interface as mission_node (node name misty_nav): /wizard/event NAV_GOTO:<id>
(study), NAV_START (the `mission` list with start/fail/goal roles), NAV_GO,
NAV_PAUSE / STOP, RESUME, NAV_RESET; /nav/status (latched), /nav/arrived, REACT:<name>.
Output: /nav/cmd_vel (TwistStamped, m/s) -> twist_mux (priority 10) -> Misty.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Follower(Node):
    def __init__(self):
        super().__init__('misty_nav')
        P = self.declare_parameter
        P('mission', [1, 2, 3])                 # NAV_START: drive to these markers in order
        P('start_markers', [1])
        P('fail_markers', [2])
        P('goal_markers', [3])
        P('fail_trigger', False)
        P('start_reaction', 'GREETING')
        P('fail_reaction', 'FAILURE')
        P('goal_reaction', 'ARRIVE')
        P('waypoint_reaction', '')
        P('wait_after_start', True)
        P('wait_after_fail', True)
        P('head_pitch_deg', 0.0)                # head pose while driving (camera forward)
        P('head_tolerance_deg', 5.0)            # re-centre the head when it's further off than this
        P('flip_x', False)                      # True if the detector's x axis points left
        P('standoff_m', 0.6)                    # stop this far from the marker (camera to marker)
        P('arrive_tol_m', 0.08)
        P('max_linear', 0.20)                   # m/s
        P('min_linear', 0.06)                   # m/s
        P('k_linear', 0.5)                      # m/s per metre of remaining distance
        P('k_angular', 0.5)                     # rad/s per rad of bearing
        P('min_angular', 0.15)                  # rad/s: steering at least this (smaller doesn't turn Misty)
        P('max_angular', 0.25)                  # rad/s: steering while driving, at most
        P('center_deadband_deg', 6.0)           # marker this close to the centre: drive straight
        P('turn_only_deg', 20.0)                # drive window; further off-centre: turn in place
        P('turn_rate', 0.3)                     # rad/s for turns in place
        P('turn_fraction', 0.6)                 # turn this share of the angle, then settle and look
        P('settle_s', 0.8)                      # stand still after an in-place turn
        P('lost_timeout_s', 1.0)                # no detection for this long: target lost
        P('reacquire_s', 5.0)                   # lost this recently: turn back to where it must be
        P('search_on_new_target', True)         # new target not in view: turn in place to find it
        P('search_rate', 0.3)                   # rad/s, continuous, one full turn at most
        P('search_direction', 1.0)              # +1 = left, -1 = right
        P('stop_m', 0.35)                       # front ToF closer than this: obstacle
        P('obstacle_sensors', ['tof_center'])   # Range frame_ids that count as "in the path"
        P('avoid', True)                        # False: just stop at obstacles (the wizard steers)
        P('avoid_turn_deg', 45.0)               # avoidance: turn this far away from the obstacle,
        P('avoid_forward_m', 0.4)               # ... drive this far, then turn back / search
        P('avoid_speed', 0.12)                  # m/s
        P('face_tracking', True)                # face tracking (face_tracker) off while driving,
                                                # on once Misty has stopped at a marker
        P('rate', 10.0)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd = self.create_publisher(TwistStamped, 'nav/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'nav/status', latched)
        self.arrived_pub = self.create_publisher(String, 'nav/arrived', 10)
        self.wizard = self.create_publisher(String, 'wizard/event', 10)
        self.head_goal = self.create_publisher(Vector3, 'misty/head_goal', 10)
        self.create_subscription(String, 'wizard/event', self.on_event, 10)
        self.create_subscription(String, 'misty/markers', self.on_markers, 20)
        self.create_subscription(Range, 'misty/tof', self.on_tof, 50)
        self.create_subscription(Vector3, 'misty/head_state', self.on_head, 10)
        self.create_subscription(Imu, 'misty/imu', self.on_imu, 50)

        self.state, self.goto, self.index, self.paused_from = 'IDLE', None, 0, None
        self.seen = {}                          # id -> (time, x right, z forward, IMU yaw at detection)
        self.tof = {}                           # frame -> (time, range, max_range)
        self.yaw = None
        self.head = (0.0, 0.0, 0.0)             # (time, pitch, yaw) degrees, measured
        self.last_center = 0.0
        self.status, self.moving = '', False
        self.reset_phase()
        self.create_timer(1.0 / float(self.p('rate')), self.tick)
        self.set_status('IDLE: press NAV START (or a study state that drives)')

    # ------------------------------------------------------------------ basics
    def p(self, name):
        return self.get_parameter(name).value

    def set_status(self, text):
        if text != self.status:
            self.status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(text)

    def reset_phase(self):
        self.phase, self.phase_until, self.look_since = 'track', 0.0, 0.0
        self.turn_dir, self.turn_angle, self.turn_speed, self.turn_yaw0 = 0.0, 0.0, 0.0, None
        self.searched, self.search_turned, self.search_last_yaw = False, 0.0, None

    def drive(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd.publish(msg)
        self.moving = v != 0.0 or w != 0.0

    def stop(self):
        self.drive(0.0, 0.0)

    # ------------------------------------------------------------------ inputs
    def on_markers(self, msg):
        try:
            data = json.loads(msg.data)
            now = time.monotonic()
            for d in data.get('detections', []):
                x = float(d['x'])
                self.seen[int(d['id'])] = (now, -x if self.p('flip_x') else x, float(d['z']), self.yaw)
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warning(f'bad /misty/markers message: {e}', throttle_duration_sec=10.0)

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_head(self, msg):
        self.head = (time.monotonic(), float(msg.x), float(msg.z))

    def on_tof(self, msg):
        self.tof[msg.header.frame_id] = (time.monotonic(), msg.range, msg.max_range)

    def on_event(self, msg):
        e = msg.data.strip().upper()
        if e.startswith('NAV_GOTO:'):
            try:
                self.goto = int(e.split(':', 1)[1])
            except ValueError:
                return
            self.begin()
        elif e == 'NAV_START':
            self.goto = None
            if self.state in ('IDLE', 'DONE'):
                self.index = 0
            self.begin()
        elif e in ('NAV_GO', 'RESUME'):
            if self.state == 'PAUSED':
                self.state = self.paused_from or 'IDLE'
                self.set_status(f'resumed: {self.state}')
            elif e == 'NAV_GO' and self.state == 'WAIT':
                self.index += 1
                self.begin()
        elif e in ('NAV_PAUSE', 'STOP'):
            if self.state == 'FOLLOW':
                self.paused_from, self.state = 'FOLLOW', 'PAUSED'
                self.reset_phase()
                self.stop()
                self.set_status('PAUSED: NAV GO / RESUME to continue')
        elif e == 'NAV_RESET':
            self.state, self.index, self.goto = 'IDLE', 0, None
            self.stop()
            self.set_status('IDLE (reset): press NAV START')

    # ------------------------------------------------------------------ helpers
    def target(self):
        if self.goto is not None:
            return self.goto
        mission = list(self.p('mission'))
        return mission[self.index] if self.index < len(mission) else None

    def role(self, mid):
        for role in ('start', 'fail', 'goal'):
            if mid in self.p(f'{role}_markers'):
                return role
        return 'waypoint'

    def begin(self):
        mid = self.target()
        self.reset_phase()
        if mid is None:
            self.state = 'DONE'
            self.stop()
            return self.set_status('DONE: mission complete')
        self.face_tracking(False)               # the camera must look for the marker, not faces
        self.center_head(force=True)
        self.state = 'FOLLOW'
        self.set_status(f'FOLLOW marker {mid}')

    def face_tracking(self, on):
        if self.p('face_tracking'):
            self.wizard.publish(String(data='FACE_TRACK_ON' if on else 'FACE_TRACK_OFF'))

    def center_head(self, force=False):
        """The camera is in the head: keep it neutral while driving (re-centre at most 1/s)."""
        _, pitch, yaw = self.head
        tol, pitch0 = float(self.p('head_tolerance_deg')), float(self.p('head_pitch_deg'))
        if (force or abs(yaw) > tol or abs(pitch - pitch0) > tol) and time.monotonic() - self.last_center > 1.0:
            self.head_goal.publish(Vector3(x=pitch0, y=0.0, z=0.0))
            self.last_center = time.monotonic()

    def turned_since(self, yaw0):
        return 0.0 if yaw0 is None or self.yaw is None else wrap(self.yaw - yaw0)

    def bearing_of(self, s):
        """Body bearing to the marker now (+ = left)."""
        _, x, z, yaw_then = s
        head = self.head[2] if time.monotonic() - self.head[0] < 3.0 else 0.0
        return math.atan2(-x, z) + math.radians(head) - self.turned_since(yaw_then)

    def obstacle(self):
        now = time.monotonic()
        near = [r for f, (t, r, mx) in self.tof.items()
                if f in self.p('obstacle_sensors') and now - t < 1.0 and r < mx and r < float(self.p('stop_m'))]
        return min(near) if near else None

    def free_side(self):
        """+1 = left looks freer, -1 = right, by the side ToFs (if their readings are valid)."""
        now = time.monotonic()
        rng = {}
        for f in ('tof_left', 'tof_right'):
            t, r, mx = self.tof.get(f, (0.0, math.inf, math.inf))
            rng[f] = r if now - t < 1.0 and r > 0.05 else mx      # < 5 cm: blocked / bogus -> ignore
        return 1.0 if rng['tof_left'] >= rng['tof_right'] else -1.0

    # ------------------------------------------------------------------ phases
    def start_turn(self, angle, rate, phase='turn'):
        self.phase, self.turn_dir, self.turn_angle = phase, math.copysign(1.0, angle), abs(angle)
        self.turn_speed, self.turn_yaw0 = rate, self.yaw
        self.phase_until = time.monotonic() + max(0.5, abs(angle) / max(rate, 0.05) * 2.0)
        self.drive(0.0, self.turn_dir * rate)

    def turn_finished(self, now):
        if self.turn_yaw0 is not None and self.yaw is not None \
                and self.turn_dir * self.turned_since(self.turn_yaw0) >= self.turn_angle:
            return True
        return now >= self.phase_until

    def settle(self, seconds):
        self.stop()
        self.phase, self.look_since, self.phase_until = 'settle', time.monotonic(), time.monotonic() + seconds

    def tick(self):
        if self.state != 'FOLLOW':
            return
        try:
            self.step()
        except Exception as e:                  # never die mid-run: stop and report
            self.stop()
            self.get_logger().error(f'follower error: {e!r}', throttle_duration_sec=5.0)

    def step(self):
        self.center_head()
        mid, now = self.target(), time.monotonic()
        s = self.seen.get(mid)
        fresh = s is not None and now - s[0] <= float(self.p('lost_timeout_s'))

        # ---- running manoeuvres
        if self.phase in ('turn', 'avoid_turn'):
            if self.turn_finished(now):
                if self.phase == 'avoid_turn':      # avoidance: now drive past the obstacle
                    self.phase = 'avoid_drive'
                    self.phase_until = now + float(self.p('avoid_forward_m')) / float(self.p('avoid_speed'))
                else:
                    self.settle(float(self.p('settle_s')))
            else:
                self.drive(0.0, self.turn_dir * self.turn_speed)
            return
        if self.phase == 'avoid_drive':
            if self.obstacle() is not None:         # still blocked: turn further away
                return self.start_avoid()
            if now < self.phase_until:
                self.drive(float(self.p('avoid_speed')), 0.0)
                return self.set_status(f'AVOID: driving past the obstacle (marker {mid})')
            self.phase, self.searched = 'track', False  # done: find the marker again
        if self.phase == 'search':
            if self.yaw is not None and self.search_last_yaw is not None:
                self.search_turned += abs(self.turned_since(self.search_last_yaw))
            self.search_last_yaw = self.yaw
            if s is not None and s[0] > self.look_since:
                self.phase = 'track'                # found it: track right away
            elif self.search_turned >= 2 * math.pi or now >= self.phase_until:
                self.stop()
                self.phase = 'wait'
            else:
                self.drive(0.0, self.turn_dir * self.turn_speed)
                return
        if self.phase == 'settle':
            seen_recently = s is not None and now - s[0] < float(self.p('reacquire_s'))
            new = s is not None and s[0] > self.look_since
            if now < self.phase_until or (not new and seen_recently
                                          and now < self.phase_until + float(self.p('lost_timeout_s'))):
                return
            self.phase = 'track'

        # ---- target not in view
        if not fresh:
            recent = s is not None and now - s[0] < float(self.p('reacquire_s'))
            if self.phase == 'track' and recent and s[3] is not None and self.yaw is not None:
                bearing = self.bearing_of(s)        # overshoot: turn back once, then look
                if abs(bearing) > math.radians(float(self.p('center_deadband_deg'))):
                    self.start_turn(bearing * float(self.p('turn_fraction')), 0.7 * float(self.p('turn_rate')))
                    return self.set_status(f'REACQUIRE marker {mid}: turning back {math.degrees(bearing):+.0f} deg')
            if self.phase == 'track' and not self.searched and self.p('search_on_new_target'):
                self.searched = True
                self.search_turned, self.search_last_yaw, self.look_since = 0.0, self.yaw, now
                self.start_turn(math.copysign(2 * math.pi, float(self.p('search_direction'))),
                                float(self.p('search_rate')), phase='search')
                return self.set_status(f'SEARCH marker {mid}: turning to find it')
            if self.moving:
                self.stop()
            self.phase = 'wait'
            last = '' if s is None else f' (last seen {now - s[0]:.0f} s ago)'
            return self.set_status(f'WAITING: marker {mid} not in view{last}. Steer Misty until she sees it')

        # ---- target in view
        self.phase = 'track'
        bearing = self.bearing_of(s)
        dist = math.hypot(s[1], s[2])
        err = dist - float(self.p('standoff_m'))
        if err <= float(self.p('arrive_tol_m')):
            self.stop()
            return self.arrive(mid, dist)
        obstacle = self.obstacle()
        if obstacle is not None:
            if self.p('avoid'):
                return self.start_avoid(obstacle)
            self.stop()
            return self.set_status(f'BLOCKED: obstacle {obstacle:.2f} m ahead (marker {mid}). Steer Misty around it')
        if abs(bearing) > math.radians(float(self.p('turn_only_deg'))):
            self.start_turn(bearing * float(self.p('turn_fraction')), float(self.p('turn_rate')))
            return self.set_status(f'FOLLOW marker {mid}: turning {math.degrees(bearing):+.0f} deg')
        if abs(bearing) < math.radians(float(self.p('center_deadband_deg'))):
            w = 0.0
        else:
            w = math.copysign(clamp(abs(float(self.p('k_angular')) * bearing), float(self.p('min_angular')),
                                    float(self.p('max_angular'))), bearing)
        v = clamp(float(self.p('k_linear')) * err, float(self.p('min_linear')), float(self.p('max_linear')))
        self.drive(v * math.cos(bearing), w)
        self.set_status(f'FOLLOW marker {mid}: {err:.2f} m to go, {math.degrees(bearing):+.0f} deg')

    def start_avoid(self, obstacle=None):
        side = self.free_side()
        self.start_turn(side * math.radians(float(self.p('avoid_turn_deg'))), float(self.p('turn_rate')),
                        phase='avoid_turn')
        what = f'{obstacle:.2f} m ' if obstacle is not None else ''
        self.set_status(f'AVOID: obstacle {what}ahead, turning {"left" if side > 0 else "right"}')

    def arrive(self, mid, dist):
        self.set_status(f'ARRIVED at marker {mid} ({dist:.2f} m)')
        if self.goto is not None:               # study target: report, no reactions
            self.state = 'ARRIVED'
            self.face_tracking(True)
            self.arrived_pub.publish(String(data=str(mid)))
            return
        role = self.role(mid)

        def react(name):
            if name:
                self.wizard.publish(String(data=f'REACT:{name}'))
        if role == 'start':
            react(self.p('start_reaction'))
            if self.p('wait_after_start'):
                self.state = 'WAIT'
                self.face_tracking(True)
                return self.set_status(f'WAIT at start marker {mid}: press NAV GO to continue')
        elif role == 'fail' and self.p('fail_trigger'):
            react(self.p('fail_reaction'))
            if self.p('wait_after_fail'):
                self.state = 'WAIT'
                self.face_tracking(True)
                return self.set_status(f'WAIT at fail marker {mid}: press NAV GO to continue')
        elif role == 'goal':
            react(self.p('goal_reaction'))
            self.state = 'DONE'
            self.face_tracking(True)
            return self.set_status(f'DONE: arrived at goal marker {mid}')
        else:
            react(self.p('waypoint_reaction'))
        self.index += 1
        self.begin()


def main():
    rclpy.init()
    node = Follower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
