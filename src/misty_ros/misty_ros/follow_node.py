"""Marker follower: lock onto the target ArUco marker and drive straight at it.

A light alternative to Nav2 (study.launch.py nav:=follow, the default): no map, no
odometry. Steering comes directly from the detection (bearing and
distance in the camera frame, plus the measured head yaw from /misty/head_state,
since the camera turns with the head), so it doesn't depend on speed calibration.

    target in view   drive towards it (turning towards it while driving); stop
                     standoff_m in front of it -> /nav/arrived
    off-centre       > turn_only_deg: turn part of the way in place, stand still
                     (settle_s) and decide again on a detection taken after stopping
    new target not   search: turn search_step_deg in place, stop and look, repeat for
      in view        one full turn (IMU); then stand still and wait: the wizard steers
                     Misty (controller / Foxglove DRIVE override this node in twist_mux)
                     until she sees it, then she takes over by herself
    lost (overshoot) one turn back to where it must be (IMU), then look
    obstacle         front time-of-flight closer than stop_m: stop and wait until the
                     wizard has steered her clear

Same interface as mission_node (node name misty_nav), so the study and the Foxglove
buttons work unchanged: /wizard/event NAV_GOTO:<id> (study), NAV_START (the
`mission` list with start/fail/goal roles), NAV_GO, NAV_PAUSE (also STOP),
NAV_RESET; /nav/status (latched), /nav/arrived, REACT:<name> for mission roles.
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


class Follower(Node):
    def __init__(self):
        super().__init__('misty_nav')            # same name as mission_node: same Foxglove parameters
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
        P('standoff_m', 0.6)                    # stop this far from the marker (camera to marker)
        P('arrive_tol_m', 0.08)
        P('max_linear', 0.20)                   # m/s
        P('min_linear', 0.06)                   # m/s: slower than this Misty doesn't move
        P('k_linear', 0.5)                      # m/s per metre of remaining distance
        P('max_angular', 0.25)                  # rad/s: steering while driving, at most
        P('min_angular', 0.15)                  # rad/s: steering at least this (smaller doesn't turn Misty)
        P('k_angular', 0.5)                     # rad/s per rad of bearing
        P('center_deadband_deg', 6.0)           # marker this close to the centre: drive straight
        P('turn_rate', 0.3)                     # rad/s for turns in place (must overcome Misty's dead zone)
        P('flip_x', False)                      # True if the detector's x axis points left (sign check)
        P('turn_only_deg', 20.0)                # drive window: marker within this -> drive (steering);
                                                # further off-centre -> turn in place, then settle
        P('turn_fraction', 0.6)                 # turn this share of the angle, then stop and look again
                                                # (Misty keeps turning a bit after the stop command)
        P('settle_s', 0.8)                      # after turning in place: stand still this long and use a
                                                # detection taken AFTER stopping before deciding again
        P('reacquire_s', 5.0)                   # lost after an overshoot: turn back towards where the
                                                # marker must be (IMU) once, then look
        P('search_on_new_target', True)         # new target not in view: turn in place in steps to find it
        P('search_step_deg', 30.0)              # search: turn this much, ...
        P('search_look_s', 1.2)                 # ... stand still this long to look, repeat (one full turn)
        P('search_direction', 1.0)              # +1 = turn left, -1 = right
        P('lost_timeout_s', 1.0)                # no detection for this long: stop and wait
        P('stop_m', 0.35)                       # front ToF closer than this: stop
        P('obstacle_sensors', ['tof_center'])   # Range frame_ids that count as "in the path"
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
        self.head_yaw = (0.0, 0.0)              # (time, degrees + = left): the camera turns with the head
        self.head_pitch = 0.0
        self.last_center = 0.0

        self.state, self.goto, self.index = 'IDLE', None, 0
        self.paused_from = None
        self.seen = {}                          # id -> (time, x right, z forward, robot yaw at detection)
        self.yaw = None                         # IMU yaw (rad): detections are late, so the turn made
                                                # since a detection is subtracted (latency compensation)
        self.create_subscription(Imu, 'misty/imu', self.on_imu, 50)
        self.tof = {}                           # frame -> (time, range, max_range)
        self.phase, self.phase_until, self.look_since = 'track', 0.0, 0.0
        self.turn_start_yaw, self.turn_angle, self.turn_dir = None, 0.0, 0.0
        self.searched, self.search_turned = False, 0.0
        self.status = ''
        self.moving = False
        self.create_timer(1.0 / float(self.p('rate')), self.tick)
        self.set_status('IDLE: press NAV START (or a study state that drives)')

    def p(self, name):
        return self.get_parameter(name).value

    def set_status(self, text):
        if text != self.status:
            self.status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(text)

    # ---------------------------------------------------------------- inputs
    def on_markers(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        now = time.monotonic()
        for d in data.get('detections', []):
            x = -float(d['x']) if self.p('flip_x') else float(d['x'])
            self.seen[int(d['id'])] = (now, x, float(d['z']), self.yaw)

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_head(self, msg):
        self.head_yaw = (time.monotonic(), float(msg.z))
        self.head_pitch = float(msg.x)

    def center_head(self, force=False):
        """Keep the head (camera) neutral while driving: re-centre it if it's more than
        head_tolerance_deg off (e.g. the wizard turned it), at most once a second."""
        now = time.monotonic()
        off = abs(self.head_yaw[1]) > float(self.p('head_tolerance_deg')) or \
            abs(self.head_pitch - float(self.p('head_pitch_deg'))) > float(self.p('head_tolerance_deg'))
        if (force or off) and now - self.last_center > 1.0:
            self.head_goal.publish(Vector3(x=float(self.p('head_pitch_deg')), y=0.0, z=0.0))
            self.last_center = now

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
        elif e == 'NAV_GO':
            if self.state == 'WAIT':
                self.index += 1
                self.begin()
            elif self.state == 'PAUSED':
                self.state = self.paused_from or 'IDLE'
                self.set_status(f'resumed: {self.state}')
        elif e in ('NAV_PAUSE', 'STOP'):
            if self.state == 'FOLLOW':
                self.paused_from, self.state = self.state, 'PAUSED'
                self.stop()
                self.set_status('PAUSED: NAV GO / RESUME to continue')
        elif e == 'NAV_RESET':
            self.state, self.index, self.goto = 'IDLE', 0, None
            self.stop()
            self.set_status('IDLE (reset): press NAV START')

    # ---------------------------------------------------------------- logic
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
        if mid is None:
            self.state = 'DONE'
            self.stop()
            self.set_status('DONE: mission complete')
            return
        self.last_center = 0.0
        self.center_head(force=True)
        self.phase, self.searched, self.search_turned = 'track', False, 0.0
        self.state = 'FOLLOW'
        self.set_status(f'FOLLOW marker {mid}')

    def drive(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd.publish(msg)
        self.moving = v != 0.0 or w != 0.0

    def stop(self):
        self.drive(0.0, 0.0)

    def blocked(self):
        now = time.monotonic()
        near = [r for f, (t, r, mx) in self.tof.items()
                if f in self.p('obstacle_sensors') and now - t < 1.0 and r < mx and r < float(self.p('stop_m'))]
        return min(near) if near else None

    def turned_since(self, yaw0):
        if yaw0 is None or self.yaw is None:
            return 0.0
        return math.atan2(math.sin(self.yaw - yaw0), math.cos(self.yaw - yaw0))

    def start_turn(self, angle, phase, rate):
        """Turn in place by `angle` (rad, + = left), measured with the IMU (time limit as a backup)."""
        self.phase, self.turn_angle, self.turn_dir = phase, abs(angle), math.copysign(1.0, angle)
        self.turn_start_yaw = self.yaw
        self.phase_until = time.monotonic() + max(0.4, abs(angle) / max(rate, 0.05) * 2.0)
        self.drive(0.0, self.turn_dir * rate)

    def turn_done(self, now):
        if self.yaw is not None and self.turn_start_yaw is not None:
            if self.turn_dir * self.turned_since(self.turn_start_yaw) >= self.turn_angle:
                return True
        return now >= self.phase_until

    def look(self, phase, seconds):
        """Stop, and accept only detections made after stopping."""
        self.stop()
        self.phase, self.look_since, self.phase_until = phase, time.monotonic(), time.monotonic() + seconds

    def bearing_of(self, s):
        """Bearing from the body to the marker now (+ = left): camera bearing at detection time + head
        yaw - what Misty has turned since the detection (detections arrive late)."""
        _, x, z, yaw_then = s
        head = self.head_yaw[1] if time.monotonic() - self.head_yaw[0] < 3.0 else 0.0
        return math.atan2(-x, z) + math.radians(head) - self.turned_since(yaw_then), head

    def tick(self):
        if self.state != 'FOLLOW':
            return
        self.center_head()
        mid = self.target()
        now = time.monotonic()
        s = self.seen.get(mid)
        rate = float(self.p('turn_rate'))                     # in-place turns

        # ---- running in-place turns: finish them, then stand still and look
        if self.phase in ('turn', 'search_turn'):
            if self.phase == 'search_turn' and s is not None and s[0] > self.look_since:
                self.look('settle', float(self.p('settle_s')))      # found it while turning
            elif self.turn_done(now):
                if self.phase == 'search_turn':
                    self.search_turned += abs(self.turned_since(self.turn_start_yaw)) or self.turn_angle
                    self.look('search_look', float(self.p('search_look_s')))
                else:
                    self.look('settle', float(self.p('settle_s')))
            else:
                self.drive(0.0, self.turn_dir * rate)
            return
        if self.phase in ('settle', 'search_look'):
            fresh = s is not None and s[0] > self.look_since
            if now < self.phase_until or (self.phase == 'settle' and not fresh
                                          and now < self.phase_until + float(self.p('lost_timeout_s'))):
                return                                           # stand still, wait for a new detection
            if self.phase == 'search_look' and not fresh:
                if self.search_turned >= 2 * math.pi - 0.1:
                    self.phase = 'wait'
                else:
                    self.start_turn(math.copysign(math.radians(float(self.p('search_step_deg'))),
                                                  float(self.p('search_direction'))), 'search_turn', rate)
                    self.set_status(f'SEARCH marker {mid}: turning to look '
                                    f'({math.degrees(self.search_turned):.0f} deg so far)')
                    return
            else:
                self.phase = 'track'

        # ---- target not in view
        lost = s is None or now - s[0] > float(self.p('lost_timeout_s'))
        if lost:
            recent = s is not None and now - s[0] < float(self.p('reacquire_s'))
            if self.phase == 'track' and recent and s[3] is not None and self.yaw is not None:
                bearing, _ = self.bearing_of(s)                  # overshoot: one turn back, then look
                if abs(bearing) > math.radians(float(self.p('center_deadband_deg'))):
                    self.start_turn(bearing * float(self.p('turn_fraction')), 'turn', 0.7 * rate)
                    self.set_status(f'REACQUIRE marker {mid}: turning back {math.degrees(bearing):+.0f} deg')
                    return
            if self.phase == 'track' and not self.searched and self.p('search_on_new_target') and not recent:
                self.searched, self.search_turned = True, 0.0
                self.look('search_look', 0.5)                    # first a short look, then turn in steps
                self.set_status(f'SEARCH marker {mid}: looking around')
                return
            if self.moving:
                self.stop()
            self.phase = 'wait'
            last = '' if s is None else f' (last seen {now - s[0]:.0f} s ago)'
            self.set_status(f'WAITING: marker {mid} not in view{last}. Steer Misty until she sees it')
            return

        # ---- target in view
        self.phase = 'track'
        bearing, head = self.bearing_of(s)
        dist = math.hypot(s[1], s[2])
        err = dist - float(self.p('standoff_m'))
        if err <= float(self.p('arrive_tol_m')):
            self.stop()
            return self.arrive(mid, dist)
        obstacle = self.blocked()
        if obstacle is not None:
            self.stop()
            self.set_status(f'BLOCKED: obstacle {obstacle:.2f} m ahead on the way to marker {mid}. '
                            'Steer Misty around it')
            return
        if abs(bearing) > math.radians(float(self.p('turn_only_deg'))):
            # far off-centre: turn part of the way in place, then stop and look again (no oscillation)
            self.start_turn(bearing * float(self.p('turn_fraction')), 'turn', rate)
            self.set_status(f'FOLLOW marker {mid}: turning {math.degrees(bearing):+.0f} deg')
            return
        # inside the drive window: drive, steering with at least min_angular (Misty's dead zone)
        if abs(bearing) < math.radians(float(self.p('center_deadband_deg'))):
            w = 0.0                                              # close enough to centre: straight
        else:
            w = math.copysign(clamp(abs(float(self.p('k_angular')) * bearing), float(self.p('min_angular')),
                                    float(self.p('max_angular'))), bearing)
        v = clamp(float(self.p('k_linear')) * err, float(self.p('min_linear')), float(self.p('max_linear')))
        self.drive(v * math.cos(bearing), w)
        self.set_status(f'FOLLOW marker {mid}: {err:.2f} m to go, {math.degrees(bearing):+.0f} deg'
                        + (f' (head {head:+.0f} deg)' if abs(head) > 3 else ''))

    def arrive(self, mid, dist):
        self.set_status(f'ARRIVED at marker {mid} ({dist:.2f} m)')
        if self.goto is not None:                       # study target: report, no reactions
            self.state = 'ARRIVED'
            self.arrived_pub.publish(String(data=str(mid)))
            return
        role = self.role(mid)
        react = lambda r: r and self.wizard.publish(String(data=f'REACT:{r}'))
        if role == 'start':
            react(self.p('start_reaction'))
            if self.p('wait_after_start'):
                self.state = 'WAIT'
                return self.set_status(f'WAIT at start marker {mid}: press NAV GO to continue')
        elif role == 'fail' and self.p('fail_trigger'):
            react(self.p('fail_reaction'))
            if self.p('wait_after_fail'):
                self.state = 'WAIT'
                return self.set_status(f'WAIT at fail marker {mid}: press NAV GO to continue')
        elif role == 'goal':
            react(self.p('goal_reaction'))
            self.state = 'DONE'
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
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
