# DD2413 — face tracking controls (ROS 2 Jazzy / WSL)

This integration uses Misty II **anonymous built-in face detection**, NOT
trained-face identity recognition. It moves the **head only** using the existing
`misty_teleop` `/misty/head_cmd` ROS topic. No facial images or names are saved.

## New Foxglove controls

- **Face tracking ON**: sends `FACE_TRACK_ON` on `/wizard/event`.
- **Face tracking OFF**: sends `FACE_TRACK_OFF` on `/wizard/event`.
- **Face tracking status**: displays `/misty/face_tracking/status`.
- **STOP all**, **Head centre**, **Look back** and **END session** automatically
  turn face tracking OFF so the wizard's head movement takes priority.
- Face tracking is **OFF by default** and remains OFF until you click ON.
- With tracking enabled, turn manual head teleop off (or don't touch it);
  they share the same motor driver.
- Detection selects the **latest visible face**, not a particular individual.
  Multiple people may cause the head to switch targets.

## Install and launch on your Ubuntu 24.04 / WSL2

From the DD2413 workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
ros2 launch misty_ros wizard.launch.py misty_ip:=192.168.1.237 participant_id:=P01 condition:=BREAKDOWN
```

In **Windows Foxglove** connect using **Foxglove WebSocket**, NOT Rosbridge,
with `ws://localhost:8765`. Re-import the updated
`foxglove/misty_wizard_layout.json` with **Layouts → Import from file**.

If camera polling and built-in face detection contend (e.g. HTTP 409 from
`take_picture`), try starting the wizard with `camera:=false`. This removes the
ROS image feed from Foxglove, but still runs Misty's built-in detector.

```bash
ros2 launch misty_ros wizard.launch.py misty_ip:=192.168.1.237 participant_id:=P01 condition:=BREAKDOWN camera:=false
```

## Offline test today — no robot needed

First, run the pure-Python tests from the workspace root:

```bash
python3 -m unittest discover -s tests -v
```

Then start ONLY the face tracker in **dry run** mode (not the wizard):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
ros2 run misty_ros face_tracker --ros-args -p dry_run:=true
```

In another Ubuntu terminal (also source the ROS and workspace setup):

```bash
# Check statuses and the commands that WOULD move the head:
ros2 topic echo /misty/face_tracking/status
ros2 topic echo /misty/head_cmd
```

In a third terminal (also source ROS and the workspace setup):

```bash
ros2 topic pub --once /wizard/event std_msgs/msg/String "{data: 'FACE_TRACK_ON'}"
ros2 topic pub --once /misty/face_tracking/test_face geometry_msgs/msg/Vector3 "{x: 20.0, y: 10.0, z: 0.0}"
ros2 topic pub --once /wizard/event std_msgs/msg/String "{data: 'FACE_TRACK_OFF'}"
```

You should see a status transition and one bounded Twist nudge for the test
face, even without the robot. Dry-run never makes HTTP or WebSocket calls.
You can import the updated layout into Foxglove with this standalone node, but
to use Foxglove buttons via network you also need `foxglove_bridge` running;
for offline CLI testing the three terminals above are sufficient.

## Live calibration tomorrow

1. Keep the robot stationary, clear its head travel, and point it toward **one** person.
2. Click **Face tracking ON**, watch the status, and make small sideways moves.
3. If the head turns **away** horizontally, set `yaw_sign: -1.0` under
   `misty_face_tracker` in `src/misty_ros/config/misty_params.yaml`.
   If it moves away vertically, set `pitch_sign: -1.0`.
4. Restart launch to apply parameter changes. The default is only an initial
   estimate; head direction is **not yet validated on your Misty**.
5. Start with a large deadband and small `max_command`. If it oscillates,
   increase `deadband_deg`, decrease `gain` or reduce `max_command`.
6. A stale or lost face stops generating new head commands. OFF does not
   automatically recenter the head.

**Limits:** This uses the robot's FaceRecognition WebSocket bearing/elevation
fields and `/api/faces/detection/start|stop`. Camera resource conflicts may
occur. These detection endpoints are robot-wide: another application's use of
them can interfere. Test before an experiment. It does not identify people,
record faces, or follow anyone by driving. Do not enroll participant faces
without appropriate consent and an approved study protocol.

Tuning values: `src/misty_ros/config/misty_params.yaml`, under
`misty_face_tracker`. The `head_step_deg` value there must match
`misty_teleop.head_step_deg`.
