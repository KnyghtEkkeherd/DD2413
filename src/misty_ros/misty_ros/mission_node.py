"""Marker mission on top of Nav2: drive to the ArUco markers in `mission`, in order.

Same interface as nav_node (the custom stack), so the Foxglove buttons and
parameters keep working: NAV_START / NAV_GO / NAV_PAUSE / NAV_RESET (and STOP)
on /wizard/event, status text on /nav/status, reactions as REACT:<name>,
runtime parameter fail_trigger. Each marker's role decides what happens on arrival:

    start_markers   greet (start_reaction), wait for NAV_GO
    fail_markers    if fail_trigger: failure (fail_reaction), wait for NAV_GO; else drive on
    goal_markers    goal_reaction, mission done
    other ids       waypoint: drive on

All driving is Nav2's: each target is a NavigateToPose goal standoff_m in front of
the marker (from marker_map), facing it; Nav2 plans around obstacles, follows the
path and runs its recoveries. Until a marker has been seen (no position in the
map yet), and after a failed goal, Misty turns on the spot (Nav2 Spin) to look
for markers before trying again.

Single targets for the study state machine: NAV_GOTO:<marker id> drives to that
marker only, then publishes the id on /nav/arrived (or /nav/failed) and waits;
no reactions. NAV_PAUSE / NAV_GO / NAV_RESET work the same.
"""
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class Mission(Node):
    def __init__(self):
        super().__init__('misty_nav')            # same name as nav_node: same Foxglove parameters
        P = self.declare_parameter
        P('marker_map', '{}')
        P('mission', [1, 2, 3])
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
        P('head_pitch_deg', 0.0)                # head pose while navigating (camera forward)
        P('standoff_m', 0.6)                    # goal: this far in front of the marker
        P('fix_timeout_s', 20.0)                # no marker fix for this long -> look around first
        P('search_step_deg', 60.0)              # Spin this much, then wait search_dwell_s
        P('search_dwell_s', 1.5)
        P('max_retries', 3)                     # failed goals before giving up on a target

        self.map = {int(k): v for k, v in json.loads(self.p('marker_map') or '{}').items()}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'nav/status', latched)
        self.wizard = self.create_publisher(String, 'wizard/event', 10)
        self.head_goal = self.create_publisher(Vector3, 'misty/head_goal', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'nav/goal_pose', latched)
        self.arrived_pub = self.create_publisher(String, 'nav/arrived', 10)
        self.failed_pub = self.create_publisher(String, 'nav/failed', 10)
        self.goto = None                        # NAV_GOTO target (study), else None (mission)
        self.create_subscription(String, 'wizard/event', self.on_event, 10)
        for topic in ('localization/marker_pose', 'localization/marker_pose_xy'):
            self.create_subscription(PoseWithCovarianceStamped, topic, self.on_fix, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')

        self.state, self.index, self.retries = 'IDLE', 0, 0
        self.paused_from = None
        self.goal_handle = None
        self.last_fix = 0.0
        self.searched = 0.0                     # degrees turned in the current search
        self.dwell_until = 0.0
        self.token = 0                          # invalidates callbacks of cancelled goals
        self.status = ''
        self.create_timer(0.2, self.tick)
        self.set_status('IDLE: press NAV START')
        if not self.map:
            self.get_logger().error('marker_map is empty: no goals can be computed')

    def p(self, name):
        return self.get_parameter(name).value

    def set_status(self, text):
        if text != self.status:
            self.status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(text)

    def react(self, name):
        if name:
            self.wizard.publish(String(data=f'REACT:{name}'))

    def on_fix(self, _msg):
        self.last_fix = time.monotonic()

    def localized(self):
        return time.monotonic() - self.last_fix < float(self.p('fix_timeout_s'))

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

    # --------------------------------------------------------------- events
    def on_event(self, msg):
        e = msg.data.strip().upper()
        if e.startswith('NAV_GOTO:'):
            try:
                self.goto = int(e.split(':', 1)[1])
            except ValueError:
                self.get_logger().error(f'bad {e}')
                return
            self.paused_from = None
            self.head_goal.publish(Vector3(x=float(self.p('head_pitch_deg')), y=0.0, z=0.0))
            self.begin_target()
        elif e == 'NAV_START':
            self.goto = None
            if self.state in ('IDLE', 'DONE', 'FAILED'):
                self.index = 0
            self.head_goal.publish(Vector3(x=float(self.p('head_pitch_deg')), y=0.0, z=0.0))
            self.begin_target()
        elif e == 'NAV_GO':
            if self.state == 'WAIT':
                self.index += 1
                self.begin_target()
            elif self.state == 'PAUSED':
                self.state = self.paused_from or 'IDLE'
                if self.state == 'WAIT':
                    self.set_status('WAIT: press NAV GO to continue')
                else:
                    self.begin_target()
        elif e in ('NAV_PAUSE', 'STOP'):
            if self.state not in ('IDLE', 'DONE', 'PAUSED', 'FAILED'):
                self.paused_from = self.state
                self.cancel()
                self.state = 'PAUSED'
                self.set_status('PAUSED: NAV GO to resume')
        elif e == 'NAV_RESET':
            self.cancel()
            self.state, self.index, self.goto = 'IDLE', 0, None
            self.set_status('IDLE (reset): press NAV START')

    # ---------------------------------------------------------------- logic
    def begin_target(self):
        self.cancel()
        self.retries = 0
        mid = self.target()
        if mid is None:
            self.state = 'DONE'
            self.set_status('DONE: mission complete')
        elif mid not in self.map:
            self.state = 'FAILED'
            self.set_status(f'FAILED: marker {mid} is not in marker_map')
        else:
            self.go_or_search()

    def go_or_search(self):
        if self.localized():
            self.send_goal(self.target())
        else:
            self.state, self.searched, self.dwell_until = 'SEARCH', 0.0, 0.0
            self.set_status(f'LOCALIZE: looking around for a marker (target {self.target()})')

    def tick(self):
        if self.state != 'SEARCH' or self.goal_handle is not None:
            return
        if self.localized():
            self.send_goal(self.target())
        elif time.monotonic() >= self.dwell_until:
            if self.searched >= 360.0:
                self.set_status('LOCALIZE: no marker after a full turn; check markers / lighting, '
                                'or drive Misty by hand, then NAV START')
                self.state = 'FAILED'
                if self.goto is not None:
                    self.failed_pub.publish(String(data=str(self.goto)))
                return
            self.send_spin(math.radians(float(self.p('search_step_deg'))))

    def goal_pose(self, mid):
        x, y, facing = self.map[mid][:3]
        f = math.radians(facing)
        d = float(self.p('standoff_m'))
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x + d * math.cos(f), y + d * math.sin(f)
        yaw = f + math.pi                                    # face the marker
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def send_goal(self, mid):
        if not self.nav.wait_for_server(timeout_sec=0.5):
            self.set_status('WAITING for Nav2 (navigate_to_pose not available yet)')
            self.state = 'SEARCH'                            # tick() retries
            self.dwell_until = time.monotonic() + 2.0
            return
        self.state = 'NAVIGATE'
        goal = NavigateToPose.Goal()
        goal.pose = self.goal_pose(mid)
        self.goal_pub.publish(goal.pose)
        self.set_status(f'NAVIGATE to marker {mid} ({self.role(mid)})')
        self.token += 1
        token = self.token
        fut = self.nav.send_goal_async(goal, feedback_callback=lambda f: self.on_feedback(token, mid, f))
        fut.add_done_callback(lambda f: self.on_accepted(token, f, lambda r: self.on_nav_done(token, mid, r)))

    def send_spin(self, angle):
        if not self.spin.wait_for_server(timeout_sec=0.5):
            self.dwell_until = time.monotonic() + 2.0
            self.set_status('WAITING for Nav2 (spin not available yet)')
            return
        goal = Spin.Goal()
        goal.target_yaw = angle
        goal.time_allowance.sec = 15
        self.token += 1
        token = self.token
        fut = self.spin.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_accepted(token, f, lambda r: self.on_spin_done(token, angle, r)))
        self.goal_handle = 'pending'

    def on_accepted(self, token, fut, done_cb):
        if token != self.token:
            return
        handle = fut.result()
        if not handle.accepted:
            self.goal_handle = None
            done_cb(None)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: done_cb(f.result()))

    def on_feedback(self, token, mid, fb):
        if token == self.token and self.state == 'NAVIGATE':
            d = fb.feedback.distance_remaining
            self.set_status(f'NAVIGATE to marker {mid} ({self.role(mid)}): {d:.1f} m to go')

    def on_spin_done(self, token, angle, result):
        if token != self.token:
            return
        self.goal_handle = None
        self.searched += math.degrees(angle)
        self.dwell_until = time.monotonic() + float(self.p('search_dwell_s'))   # let detection catch up

    def on_nav_done(self, token, mid, result):
        if token != self.token:
            return
        self.goal_handle = None
        if self.state != 'NAVIGATE':
            return
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.arrive(mid)
            return
        self.retries += 1
        why = 'rejected' if result is None else f'status {result.status}, error {getattr(result.result, "error_code", "?")}'
        if self.retries > int(self.p('max_retries')):
            self.state = 'FAILED'
            self.set_status(f'FAILED to reach marker {mid} ({why}); drive closer by hand, then retry')
            self.failed_pub.publish(String(data=str(mid)))
            return
        self.get_logger().warning(f'goal to marker {mid} failed ({why}); looking around, retry {self.retries}')
        self.last_fix = 0.0                                   # re-localize before retrying
        self.go_or_search()

    def cancel(self):
        self.token += 1                                       # ignore callbacks of the old goal
        h, self.goal_handle = self.goal_handle, None
        if h is not None and h != 'pending':
            h.cancel_goal_async()

    def arrive(self, mid):
        if self.goto is not None:                   # study target: report, no reactions
            self.state = 'ARRIVED'
            self.set_status(f'ARRIVED at marker {mid}')
            self.arrived_pub.publish(String(data=str(mid)))
            return
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
    node = Mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
