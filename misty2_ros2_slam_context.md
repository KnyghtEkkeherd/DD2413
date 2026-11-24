# Context: ROS 2 + Misty II built-in mapping & navigation

Exported chat to use as context for Claude Code.

## Goal
I already have a ROS 2 wrapper around the Misty II Python scripts/SDK. I want to call Misty's built-in SLAM (mapping, localization/tracking) and navigation (FollowPath) through that wrapper and integrate them with ROS 2 (map topic, TF, goal poses, RViz).

## Key facts
- Misty II does not run ROS or ROS 2, and ROS 2 can't be installed on the robot (Android + Raspberry Pi/Windows IoT, locked down). ROS 2 runs off-board, on a PC or Jetson on the same network, and talks to Misty over REST (commands) and WebSockets (sensor streams).
- SLAM requires the Misty II version with the Occipital Structure Core depth sensor (Pro/Advanced). A base Misty II can't map.
- Misty has no lidar, only time-of-flight sensors and a depth camera.
- Misty's visual SLAM is finicky. It needs good lighting and textured surroundings, it struggles in large or featureless spaces, and relocalizing on a saved map can fail. Code should handle lost tracking (check the status, then reset and restart tracking).
- No maintained, official ROS 2 driver is known, so the bridge is custom.
- Misty Robotics was acquired a few years back, so long-term firmware and support are uncertain. Verify that the robot's firmware exposes the SLAM endpoints.

## Two integration options
1. **Keep Misty's own planner.** ROS 2 sends goals, and Misty plans and drives (FollowPath). This is the simplest option.
2. **Use Misty only for SLAM and run Nav2 off-board.** Nav2 uses Misty's map and pose and sends `cmd_vel` back. Wi-Fi and REST add latency, so keep speeds low. Depth data must be turned into a laser scan or point cloud for Nav2's costmaps.

## Misty API calls (REST; the Python SDK has a method for each)
| Purpose | REST call |
|---|---|
| Start / stop mapping | `POST /api/slam/map/start`, `POST /api/slam/map/stop` |
| Start / stop localizing on a saved map | `POST /api/slam/track/start`, `POST /api/slam/track/stop` |
| SLAM state (exploring, tracking, lost…) | `GET /api/slam/status` |
| Fetch the occupancy grid | `GET /api/slam/map` |
| List maps / pick the active one | `GET /api/slam/map/ids`, `POST /api/slam/map/current` |
| Drive a path on the map | `POST /api/drive/path` (FollowPath, path given as map cells `"x:y,x:y"`) |
| Reset after tracking is lost | `POST /api/slam/reset` |
| Robot pose on the map | `SelfState` WebSocket, `Location` field (x, y, yaw) |

**VERIFY on the actual robot:** exact field names (`originX`, `originY`, `metersPerCell`, `grid`, `width`, `height`), the FollowPath parameter names and units (cells or meters), and the grid's row/column order. These have varied between firmware versions. Print one raw `GET /api/slam/map` response and one `SelfState` message before finishing the conversion code.

## Typical flow
1. **Mapping:** start mapping, then drive Misty around slowly with teleop through the existing `cmd_vel` until the status reports enough coverage, then stop mapping. The map is saved on the robot.
2. **Navigation:** select the map, start tracking, and wait until the status says it's tracking. Then send a path and read the pose from `SelfState`.

## ROS 2 interface design
- Trigger services: `misty/start_mapping`, `misty/stop_mapping`, `misty/start_tracking`, `misty/stop_tracking`, `misty/reset_slam`
- `/map` (`nav_msgs/OccupancyGrid`), published on a timer or latched. Misty cell values (verify): 0 unknown → -1, 1 open → 0, 2 occupied → 100, 3 covered (robot footprint) → 0
- `goal_pose` (`geometry_msgs/PoseStamped`) subscriber that converts meters to cells and calls FollowPath. This works with RViz's "2D Goal Pose" tool.
- TF `map → base_link` from `SelfState.Location`, published only while the SLAM status says it's tracking. Use `map → odom` instead if wheel odometry from the drive encoders is also published.
- Optional: wrap FollowPath in a ROS 2 action with feedback and success or failure. FollowPath is fire-and-forget, so watch the pose and the drive or locomotion events, and finish the action when Misty reaches the goal cell or stops.
- If the wrapper uses the Python SDK object rather than raw `requests`, swap `_api(...)` for the matching SDK methods. The logic stays the same.

## Reference sketch node (untested; adapt to the existing wrapper)
```python
import requests, rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

class MistySlam(Node):
    def __init__(self):
        super().__init__('misty_slam')
        self.ip = self.declare_parameter('misty_ip', '192.168.1.50').value
        for name, path in [('start_mapping', 'slam/map/start'),
                           ('stop_mapping',  'slam/map/stop'),
                           ('start_tracking','slam/track/start'),
                           ('stop_tracking', 'slam/track/stop'),
                           ('reset_slam',    'slam/reset')]:
            self.create_service(Trigger, f'misty/{name}', self._trigger(path))
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 1)
        self.create_timer(2.0, self.publish_map)
        self.res, self.origin = None, (0.0, 0.0)

    def _api(self, method, path, **kw):
        r = requests.request(method, f'http://{self.ip}/api/{path}', timeout=5, **kw)
        r.raise_for_status()
        return r.json().get('result')

    def _trigger(self, path):
        def cb(req, res):
            try:
                self._api('POST', path); res.success = True
            except Exception as e:
                res.success, res.message = False, str(e)
            return res
        return cb

    def publish_map(self):
        try:
            m = self._api('GET', 'slam/map')
        except Exception:
            return
        if not m:
            return
        # Misty cells: 0 unknown, 1 open, 2 occupied, 3 covered -- verify on firmware
        lut = {0: -1, 1: 0, 2: 100, 3: 0}
        grid = m['grid']; h, w = m['height'], m['width']
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.res = m['metersPerCell']
        msg.info.width, msg.info.height = w, h
        self.origin = (m['originX'], m['originY'])
        msg.info.origin.position.x, msg.info.origin.position.y = self.origin
        # check row/column order in RViz -- Misty's grid may be [x][y] and need transposing
        msg.data = [lut.get(grid[r][c], -1) for r in range(h) for c in range(w)]
        self.map_pub.publish(msg)

    def on_goal(self, goal):
        if not self.res:
            self.get_logger().warn('no map yet'); return
        cx = int((goal.pose.position.x - self.origin[0]) / self.res)
        cy = int((goal.pose.position.y - self.origin[1]) / self.res)
        try:
            self._api('POST', 'drive/path', json={'Path': f'{cx}:{cy}'})
        except Exception as e:
            self.get_logger().error(f'FollowPath failed: {e}')

def main():
    rclpy.init(); rclpy.spin(MistySlam())
```

## Task for Claude Code
Integrate the SLAM and navigation functionality above into my existing ROS 2 Misty wrapper, following its structure and conventions rather than adding a standalone node if possible:
1. Inspect how the wrapper currently calls Misty (Python SDK or REST) and reuse that client.
2. Add the mapping and tracking services, the `/map` publisher, the `goal_pose` → FollowPath handling, and `map → base_link` TF from `SelfState`.
3. Add SLAM status monitoring with lost-tracking recovery.
4. Optionally, turn FollowPath into a ROS 2 action.
5. Add a small script that dumps raw `slam/map` and `SelfState` payloads, so field names and grid order can be checked on the real robot.
