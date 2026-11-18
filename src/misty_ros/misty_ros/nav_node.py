"""Autonomous marker-to-marker navigation for Misty with obstacle avoidance.

Drives to the ArUco markers listed in `mission`, in order. Each marker's role
decides what happens on arrival:

    start_markers     greet (start_reaction), then wait for NAV_GO from the wizard
    fail_markers      if the runtime parameter fail_trigger is true: run the failure
                      (fail_reaction) and wait for NAV_GO; otherwise drive on
    goal_markers      goal_reaction (arrival), mission done
    anything else     waypoint: drive on

Reactions are sent as REACT:<name> on /wizard/event, so they come from
reactions.json and are logged by the wizard like any button press.

Steering to the current target, in order of preference:
  1. it is in view right now (/misty/markers)  -> steer on the live detection
  2. it is in the SLAM map (/slam/landmarks)    -> steer on the map estimate (turns
                                                  towards it when out of view)
  3. unknown                                   -> search: turn in steps and look; after a
                                                  full circle without seeing it, drive a
                                                  short way to a new viewpoint (explore)
                                                  and search again
Obstacles: front time-of-flight sensors (/misty/tof) and, if enabled, the depth
scan (/misty/depth_scan): slow down near obstacles; if blocked, turn towards
the freer side and drive past before steering to the target again.

Commands on /wizard/event: NAV_START, NAV_GO (continue after a wait),
NAV_PAUSE, NAV_RESET; STOP pauses too. Output: /cmd_vel (same scale as the
teleop panels), /nav/status (latched text).
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MistyNav(Node):
    def __init__(self):
        super().__init__('misty_nav')
        P = self.declare_parameter
        P('mission', [1, 2, 3])                 # marker ids to visit, in order
        P('start_markers', [1])
        P('fail_markers', [2])
        P('goal_markers', [3])
        P('fail_trigger', False)                # set at runtime by the wizard
        P('start_reaction', 'GREETING')
        P('fail_reaction', 'FAILURE')
        P('goal_reaction', 'ARRIVE')
        P('waypoint_reaction', '')
        P('wait_after_start', True)
        P('wait_after_fail', True)
        P('head_pitch_deg', 0.0)                # head pose while navigating (camera centred)
        # motion (Twist units: 1.0 = full teleop speed, see misty_params.yaml linear/angular_scale)
        P('max_linear', 0.8)
        P('max_angular', 0.6)
        P('k_linear', 0.8)                      # per metre of remaining distance
        P('k_angular', 1.2)                     # per radian of bearing error
        P('turn_in_place_deg', 25.0)            # bigger bearing error: turn before driving
        P('standoff_m', 0.6)                    # stop this far from the marker
        P('arrive_tol_m', 0.12)
        P('marker_fresh_s', 0.8)                # live detection counts as "in view" this long
        P('search_turn_s', 0.8)                 # search: turn this long ...
        P('search_dwell_s', 1.2)                # ... then stand still to let detection catch up
        P('search_angular', 0.5)
        P('explore_turn_deg', 90.0)             # after a fruitless full circle: turn this much
        P('explore_distance_m', 0.8)            # (alternating left / right), drive this far, look again
        P('explore_timeout_s', 15.0)
        P('obstacle_stop_m', 0.35)
        P('obstacle_slow_m', 0.7)
        P('obstacle_clear_m', 0.6)
        P('avoid_forward_s', 1.5)
        P('avoid_max_turn_s', 4.0)
        P('sensor_fresh_s', 0.6)
        P('rate', 10.0)

        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.wizard = self.create_publisher(String, 'wizard/event', 10)
        self.head_goal = self.create_publisher(Vector3, 'misty/head_goal', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'nav/status', latched)
        self.create_subscription(PoseWithCovarianceStamped, 'slam/pose', self.on_pose, 10)
        self.create_subscription(String, 'slam/landmarks', self.on_landmarks, 5)
        self.create_subscription(String, 'misty/markers', self.on_markers, 20)
        self.create_subscription(Range, 'misty/tof', self.on_tof, 50)
        self.create_subscription(LaserScan, 'misty/depth_scan', self.on_scan, 5)
        self.create_subscription(String, 'wizard/event', self.on_event, 10)

        self.pose = None
        self.landmarks = {}
        self.seen = {}                          # id -> (time, range, bearing) from live detections
        self.tof = {}                           # position -> (time, range)
        self.scan = None
        self.state = 'IDLE'
        self.index = 0
        self.paused_from = None
        self.phase_until = 0.0
        self.search_turned = 0.0                # radians turned in the current search
        self.search_last_th = None
        self.explore = None                     # (phase, target heading, start xy)
        self.explore_side = 1.0
        self.search_phase = 'dwell'
        self.avoid_dir = 1.0
        self.last_status = ''
        self.create_timer(1.0 / float(self.p('rate')), self.tick)
        self.set_status('IDLE: press NAV START')

    def p(self, name):
        return self.get_parameter(name).value

    # ---------------------------------------------------------------- inputs
    def on_pose(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        if self.state == 'SEARCH':                  # measure how far we've turned while searching
            if self.search_last_th is not None:
                self.search_turned += abs(wrap(self.pose[2] - self.search_last_th))
            self.search_last_th = self.pose[2]

    def on_landmarks(self, msg):
        try:
            self.landmarks = {int(k): (v[0], v[1]) for k, v in json.loads(msg.data)['landmarks'].items()}
        except (ValueError, KeyError, TypeError):
            pass

    def on_markers(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        now = time.monotonic()
        for d in data.get('detections', []):
            if float(d['z']) <= 0:
                continue
            xr, yr = float(d['z']), -float(d['x'])     # camera -> robot frame (offset ignored)
            self.seen[int(d['id'])] = (now, math.hypot(xr, yr), math.atan2(yr, xr))

    def on_tof(self, msg):
        self.tof[msg.header.frame_id.removeprefix('tof_')] = (time.monotonic(), msg.range)

    def on_scan(self, msg):
        self.scan = (time.monotonic(), msg)

    def on_event(self, msg):
        e = msg.data.strip().upper()
        if e == 'NAV_START':
            if self.state in ('IDLE', 'DONE'):
                self.index = 0
            self.head_goal.publish(Vector3(x=float(self.p('head_pitch_deg')), y=0.0, z=0.0))
            self.begin_target()
        elif e == 'NAV_GO':
            if self.state == 'WAIT':
                self.index += 1
                self.begin_target()
            elif self.state == 'PAUSED':
                self.state = self.paused_from or 'NAVIGATE'
                self.set_status(f'resumed: {self.state}')
        elif e in ('NAV_PAUSE', 'STOP'):
            if self.state not in ('IDLE', 'DONE', 'PAUSED'):
                self.paused_from = self.state if self.state != 'WAIT' else 'WAIT'
                self.state = 'PAUSED'
                self.stop()
                self.set_status('PAUSED: NAV GO to resume')
        elif e == 'NAV_RESET':
            self.state, self.index = 'IDLE', 0
            self.stop()
            self.set_status('IDLE (reset): press NAV START')

    # --------------------------------------------------------------- helpers
    def set_status(self, text):
        if text != self.last_status:
            self.last_status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(text)

    def drive(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd.publish(t)

    def stop(self):
        self.cmd.publish(Twist())

    def target(self):
        mission = list(self.p('mission'))
        return mission[self.index] if self.index < len(mission) else None

    def role(self, mid):
        if mid in self.p('start_markers'):
            return 'start'
        if mid in self.p('fail_markers'):
            return 'fail'
        if mid in self.p('goal_markers'):
            return 'goal'
        return 'waypoint'

    def begin_target(self):
        mid = self.target()
        if mid is None:
            self.state = 'DONE'
            self.stop()
            self.set_status('DONE: mission complete')
            return
        self.state = 'NAVIGATE'
        self.search_turned = 0.0
        self.set_status(f'NAVIGATE to marker {mid} ({self.role(mid)})')

    def target_estimate(self, mid):
        """(range, bearing, source) to the target, or None if unknown."""
        now = time.monotonic()
        s = self.seen.get(mid)
        if s and now - s[0] < float(self.p('marker_fresh_s')):
            return s[1], s[2], 'seen'
        if mid in self.landmarks and self.pose is not None:
            x, y, th = self.pose
            lx, ly = self.landmarks[mid]
            return math.hypot(lx - x, ly - y), wrap(math.atan2(ly - y, lx - x) - th), 'map'
        return None

    def obstacles(self):
        """Closest obstacle distance (front) and free space left / right."""
        now = time.monotonic()
        fresh = float(self.p('sensor_fresh_s'))
        def tof(name):
            v = self.tof.get(name)
            return v[1] if v and now - v[0] < fresh else math.inf
        center = tof('center')
        left = min(tof('left'), tof('frontleft'))
        right = min(tof('right'), tof('frontright'))
        if self.scan and now - self.scan[0] < 2 * fresh:
            sc = self.scan[1]
            for i, r in enumerate(sc.ranges):
                if not math.isfinite(r):
                    continue
                a = sc.angle_min + i * sc.angle_increment
                if abs(a) < math.radians(12):
                    center = min(center, r)
                elif a > 0:
                    left = min(left, r)
                else:
                    right = min(right, r)
        return min(center, left, right), center, left, right

    # ------------------------------------------------------------- control
    def tick(self):
        st = self.state
        if st in ('IDLE', 'DONE', 'PAUSED', 'WAIT'):
            return
        mid = self.target()
        now = time.monotonic()
        front, center, left, right = self.obstacles()

        if st == 'AVOID_TURN':
            if center > float(self.p('obstacle_clear_m')) and min(left, right) > float(self.p('obstacle_stop_m')):
                self.state, self.phase_until = 'AVOID_FWD', now + float(self.p('avoid_forward_s'))
            elif now > self.phase_until:
                self.avoid_dir = -self.avoid_dir           # stuck: try the other way
                self.phase_until = now + float(self.p('avoid_max_turn_s'))
            self.drive(0.0, self.avoid_dir * float(self.p('max_angular')))
            return
        if st == 'AVOID_FWD':
            if front < float(self.p('obstacle_stop_m')):
                self.start_avoid(left, right)
            elif now > self.phase_until:
                self.state = 'NAVIGATE'
                self.set_status(f'NAVIGATE to marker {mid} (obstacle passed)')
            else:
                self.drive(0.5 * float(self.p('max_linear')), 0.0)
            return

        est = self.target_estimate(mid)
        if st == 'SEARCH':
            if est is not None:
                self.state = 'NAVIGATE'
            elif now < self.phase_until:
                if self.search_phase == 'turn':
                    self.drive(0.0, float(self.p('search_angular')))
                else:
                    self.stop()
                return
            else:                                          # next search phase
                if self.search_phase == 'turn':
                    self.search_phase, self.phase_until = 'dwell', now + float(self.p('search_dwell_s'))
                    self.stop()
                elif self.search_turned >= 2 * math.pi and self.pose is not None:  # looked all round
                    heading = wrap(self.pose[2] + self.explore_side * math.radians(float(self.p('explore_turn_deg'))))
                    self.explore_side = -self.explore_side
                    self.explore = ['turn', heading, self.pose[:2]]
                    self.state, self.phase_until = 'EXPLORE', now + float(self.p('explore_timeout_s'))
                    self.set_status(f'EXPLORE: marker {mid} not visible from here, moving to look elsewhere')
                else:
                    self.search_phase, self.phase_until = 'turn', now + float(self.p('search_turn_s'))
                return
        if st == 'EXPLORE':
            if est is not None:
                self.state = 'NAVIGATE'
            elif now > self.phase_until or self.pose is None:
                self.begin_search(mid, now)
                return
            elif self.explore[0] == 'turn':
                err = wrap(self.explore[1] - self.pose[2])
                if abs(err) > math.radians(8):
                    self.drive(0.0, clamp(float(self.p('k_angular')) * err,
                                          -float(self.p('max_angular')), float(self.p('max_angular'))))
                    return
                self.explore[0] = 'forward'
                self.explore[2] = self.pose[:2]
                return
            else:
                moved = math.hypot(self.pose[0] - self.explore[2][0], self.pose[1] - self.explore[2][1])
                if moved >= float(self.p('explore_distance_m')) or front < float(self.p('obstacle_clear_m')):
                    self.begin_search(mid, now)          # arrived (or blocked): look again
                else:
                    self.drive(0.7 * float(self.p('max_linear')), 0.0)
                return

        # NAVIGATE
        if est is None:
            self.begin_search(mid, now)
            return
        rng, bearing, source = est
        remaining = rng - float(self.p('standoff_m'))
        if remaining <= float(self.p('arrive_tol_m')) and abs(bearing) < math.radians(30):
            self.arrive(mid)
            return
        ang = clamp(float(self.p('k_angular')) * bearing, -float(self.p('max_angular')), float(self.p('max_angular')))
        if abs(bearing) > math.radians(float(self.p('turn_in_place_deg'))):
            self.drive(0.0, ang)
            self.set_status(f'NAVIGATE to marker {mid}: turning ({source}, {math.degrees(bearing):.0f} deg)')
            return
        if front < float(self.p('obstacle_stop_m')):
            self.start_avoid(left, right)
            return
        lin = clamp(float(self.p('k_linear')) * remaining, 0.15, float(self.p('max_linear')))
        slow = float(self.p('obstacle_slow_m'))
        if front < slow:                                   # scale down near obstacles
            stop_d = float(self.p('obstacle_stop_m'))
            lin *= clamp((front - stop_d) / (slow - stop_d), 0.3, 1.0)
        self.drive(lin, ang)
        self.set_status(f'NAVIGATE to marker {mid}: {rng:.2f} m ({source})')

    def begin_search(self, mid, now):
        self.state, self.search_phase = 'SEARCH', 'dwell'
        self.phase_until = now + float(self.p('search_dwell_s'))
        self.search_turned, self.search_last_th = 0.0, None
        self.stop()
        self.set_status(f'SEARCH for marker {mid}: turning to look')

    def start_avoid(self, left, right):
        self.avoid_dir = 1.0 if left >= right else -1.0    # turn towards the freer side
        self.state, self.phase_until = 'AVOID_TURN', time.monotonic() + float(self.p('avoid_max_turn_s'))
        self.set_status(f'AVOID obstacle: turning {"left" if self.avoid_dir > 0 else "right"}')

    def react(self, name):
        if name:
            self.wizard.publish(String(data=f'REACT:{name}'))

    def arrive(self, mid):
        self.stop()
        role = self.role(mid)
        if role == 'start':
            self.react(self.p('start_reaction'))
            if self.p('wait_after_start'):
                self.state = 'WAIT'
                self.set_status(f'WAIT at start marker {mid}: greeted, press NAV GO to continue')
                return
        elif role == 'fail':
            if self.p('fail_trigger'):
                self.react(self.p('fail_reaction'))
                if self.p('wait_after_fail'):
                    self.state = 'WAIT'
                    self.set_status(f'WAIT at fail marker {mid}: failure triggered, press NAV GO to continue')
                    return
            else:
                self.get_logger().info(f'fail marker {mid} reached, fail_trigger is false: driving on')
        elif role == 'goal':
            self.react(self.p('goal_reaction'))
            self.state = 'DONE'
            self.set_status(f'DONE: arrived at goal marker {mid}')
            return
        else:
            self.react(self.p('waypoint_reaction'))
        self.index += 1
        self.begin_target()


def main():
    rclpy.init()
    node = None
    try:
        node = MistyNav()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
