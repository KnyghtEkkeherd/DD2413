# misty_ws

Workspace for controlling a Misty II robot: Python SDK (submodule), a venv for
plain-Python scripts, and a ROS 2 package (`src/misty_ros`) that streams
Misty's camera to Foxglove and lets you drive it from Foxglove's Teleop panel.

## Setup

```bash
git clone --recurse-submodules git@github.com:KnyghtEkkeherd/DD2413.git misty_ws
cd misty_ws

# Plain-Python SDK use
python3 -m venv misty_venv && source misty_venv/bin/activate
pip install -r requirements.txt && deactivate

# ROS 2 Lyrical + foxglove_bridge (Ubuntu 26.10; see script header for details)
sudo bash scripts/install_ros2.sh

# Build the ROS package (always from the workspace root, with --base-paths src
# so colcon doesn't pick up misty-python-sdk or the venv as packages)
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --base-paths src
```

## Camera view + remote control in Foxglove

```bash
source /opt/ros/lyrical/setup.bash && source install/setup.bash
ros2 launch misty_ros misty_foxglove.launch.py misty_ip:=<MISTY_IP>
```

Connect from your laptop's Foxglove app: **Open connection → Foxglove WebSocket**
→ `ws://<this-machine>:8765`. If the port isn't reachable (or over the
internet), tunnel it: `ssh -L 8765:localhost:8765 ubuntu@<this-machine>` and use
`ws://localhost:8765`.

Panels:

| Panel  | Topic                             | Notes |
|--------|-----------------------------------|-------|
| Image  | `/misty/camera/image/compressed`  | ~2 fps JPEG (`fps:=`, `width:=`, `height:=`) |
| Teleop | `/cmd_vel`                        | Drive. Set **Publish rate to 5 Hz**. |
| Teleop | `/misty/head_cmd`                 | Aim the head/camera (up/down = tilt, left/right = pan). |

Driving uses `drive_time`, so Misty stops on its own ~0.6 s after the last
command — releasing the button or losing the connection stops the robot.
Speed: `linear_scale:=30` means a Teleop value of 1.0 = 30 % of max speed.

## Wizard-of-Oz study: reaction menu

```bash
ros2 launch misty_ros wizard.launch.py misty_ip:=<MISTY_IP> participant_id:=P01 condition:=BREAKDOWN
```

Starts the camera, teleop, foxglove_bridge and the wizard menu. In Foxglove:
**Layouts → Import from file… → `foxglove/misty_wizard_layout.json`**
(don't drag the file in: that opens it as data and fails).

There is no state machine: the wizard chooses every reaction. Emotions
(mood, face, LEDs, gestures) are done in the **Misty phone app**.

| Area | What it does |
|------|--------------|
| ⚠ **FAILURE** | One press: "I'm lost" → random sad sound → "Can you help me?" (a sequence in `reactions.json`). **Stop talking / cancel** or **■ STOP all** cancels it |
| Reaction tabs | One tab per `group` in `reactions.json` (Escort, Breakdown, Responses, Sounds…): each button says a random sentence of that reaction in its voice (index logged), or plays a sound. **Custom text** tab: edit the text, optional voice prefix like `[sad] …`, then press |
| DRIVE / HEAD | Teleop panels: drive (`/cmd_vel`), head/camera tilt + pan (`/misty/head_cmd`). Head moves 13° per message at 10 Hz while held (~130°/s); tune in `misty_params.yaml` |
| Route + motion | **Next segment ▶** (next segment from `route` in `study.yaml`), Look back, Head centre, Route reset, Nudge L / fwd / R, **■ STOP all** |
| Session | Set `participant_id` + `condition` in the Parameters panel, **START session** … **END session** |

Status lines: Session (recording?), Misty said / played, Last action / head
touch, Next route segment.

### Xbox controller

The controller is read on the computer it's plugged into (your Mac) and sent
to the VM; `gamepad_teleop` starts with the launch (`gamepad:=false` to
disable). One-time setup on the Mac:

```bash
scp ubuntu@192.168.64.3:~/misty_ws/scripts/gamepad_sender.py ~/
python3 -m venv ~/misty_pad && ~/misty_pad/bin/pip install pygame-ce
```

Each session (on the Mac): `~/misty_pad/bin/python ~/gamepad_sender.py 192.168.64.3`
(add `--show` to see the stick values).

| Control | Action |
|---------|--------|
| Left stick | drive (up = forward, sideways = turn) |
| Right stick | head / camera (up/down = tilt, left/right = pan; speed follows deflection) |
| B | STOP driving, speech, sounds (driving stays off until the left stick is released) |
| A or Y | head centre |
| RT / LT | right / left arm: released = down, fully pressed = up, in between = in between (angles in `misty_params.yaml`: `arm_down_deg`, `arm_up_deg`) |
| LB / RB | slower / faster driving |

If the sender stops or the network drops, Misty stops within 0.5 s.
Speeds, deadzone, speed levels and axis inversion are in `misty_params.yaml`.

### Keyboard (in an SSH terminal)

`ros2 run misty_ros keyboard_teleop`: W/S forward/back, A/D turn, Q/E arcs,
I/K/J/L or arrow keys for the head, C head centre, Space STOP, Z/X speed.

### Camera and latency

The camera node uses Misty's **video stream** by default (`misty_camera` →
`mode: stream` in `misty_params.yaml`): ~15-20 fps instead of ~2 fps with
single photos, and it falls back to photos automatically if the stream isn't
available. `mode: rtsp` uses Misty's AV stream (H.264, highest frame rate) but
**disables Misty's camera service**, so on-board ArUco detection stops: set
`misty_sensors` → `aruco_source: opencv` with it. `rotate_deg` fixes a rotated
image.

Motor commands: drive, head and arms each have their own sender that always
sends the newest command, over kept-open connections, with up to
`parallel_requests` (default 2) in flight; the last command is re-sent once so
Misty always ends in the latest state. The Foxglove DRIVE panel sends at 10 Hz.

### Config and logs

**What Misty says and how: `src/misty_ros/config/reactions.json`** (plug and
play, re-read on every change, no restart):

```json
{
  "voices": {"sad": {"pitch": 0.8, "rate": 0.8, "voice": "", "language": ""}, ...},
  "reactions": [
    {"id": "LOST", "label": "Lost", "group": "Breakdown", "color": "#7a4fc9", "voice": "sad",
     "sentences": ["I think I'm lost.", {"text": "Where am I?", "pitch": 1.4}]},
    {"id": "SOUND_SAD", "group": "Sounds", "sounds": ["s_Sadness.wav", "s_Grief.wav"], "volume": 80},
    {"id": "FAILURE", "group": "Main", "steps": [{"say": "LOST"}, {"sound_from": "SOUND_SAD", "wait": 3}, {"say": "ASK_HELP"}]}
  ]
}
```

- **voices**: presets (pitch / rate 1.0 = normal; voice / language "" = Misty's default).
- **reactions**: as many sentences as you like; a press says one at random. `"each": true`
  gives one button per sentence. Per-sentence `pitch` / `rate` / `voice` override the
  reaction's voice. `sounds` plays one of the files; `steps` runs a sequence.
- Adding/removing reactions or changing labels/groups: run
  `python3 scripts/make_foxglove_layout.py` and re-import the layout. Sentence and
  voice edits need nothing.
- Check sound file names against the robot: `python3 scripts/check_misty_sounds.py <MISTY_IP>`.

**Speeds and gains: `src/misty_ros/config/misty_params.yaml`** (restart the launch):
drive / turn speed, head step and speed, controller rate / deadzone / head gain /
speed levels / axis inversion, keyboard settings, camera fps and resolution.
Use another file with `params_file:=/path/file.yaml`; for the keyboard:
`ros2 run misty_ros keyboard_teleop --ros-args --params-file src/misty_ros/config/misty_params.yaml`.

**Route and look-back: `src/misty_ros/config/study.yaml`** (reloaded at each START).

One CSV per session in `~/misty_ws/sessions/` (git-ignored): `timestamp,
participant_id, condition, event_source, event, utterance_variant_index,
detail`. `event_source` is `wizard` (button), `failure` (step of the FAILURE
sequence), `sensor` (head touch) or `auto`.

Dry run without the robot: `python3 scripts/fake_misty.py`, then launch with
`misty_ip:=127.0.0.1:8099` (no touch sensor: use **Log touch (sim)**).

## Autonomous marker navigation (EKF-SLAM)

```bash
ros2 launch misty_ros wizard.launch.py misty_ip:=<MISTY_IP> nav:=true      # real robot
ros2 launch misty_ros nav.launch.py sim:=true                              # simulator only
```

Misty drives to the ArUco markers in `mission` (in `misty_params.yaml`,
section `misty_nav`), in order, avoiding obstacles. The marker's role decides
what happens on arrival:

| Role | Default ids | On arrival |
|------|-------------|------------|
| start | 1 | greeting (`start_reaction`), then waits for **NAV GO** |
| fail | 2 | only if `fail_trigger` is true: failure (`fail_reaction`), waits for **NAV GO** |
| goal | 3 | arrival (`goal_reaction`), mission done |
| other ids in `mission` | | waypoint, drives on |

The wizard sets `fail_trigger` at runtime in Foxglove's Parameters panel
(`/misty_nav` → `fail_trigger`), or `ros2 param set /misty_nav fail_trigger true`.
Foxglove **Navigation** tab: NAV START, NAV GO, NAV PAUSE, NAV RESET and a 3D
view of the SLAM map; **■ STOP all** / controller **B** also pause navigation.

How it works:
- `misty_sensors`: Misty's wheel encoders, IMU, time-of-flight sensors and
  on-board ArUco detector → ROS (`aruco_source: opencv` detects markers in the
  VM from the camera images instead; optional depth-camera obstacle scan).
- `misty_slam`: EKF-SLAM. Wheel dead reckoning (+ IMU heading) predicts the
  motion; each ArUco sighting (range + bearing) corrects the pose and maps the
  marker. Optional fixed marker positions: `known_markers`.
- `misty_nav`: steers to the live detection, else to the SLAM map estimate
  (turning towards it), else searches by turning in steps; after a full circle
  without seeing it, moves to a new viewpoint and looks again. Front ToF
  sensors (and depth scan): slow down, stop, steer round obstacles.
- `nav_sim` (`sim:=true`): simulated Misty with noisy sensors, obstacles and
  markers visible only from the front, for testing without the robot.

### Gazebo simulation

Tests the whole chain (rendered camera → OpenCV ArUco detection → EKF-SLAM →
navigation, plus ToF and depth obstacle sensing) without the robot. Install
once: `sudo apt install ros-lyrical-ros-gz`.

```bash
ros2 launch misty_ros gazebo.launch.py              # headless; Foxglove on ws://<vm-ip>:8765
ros2 launch misty_ros gazebo.launch.py gui:=true    # with the Gazebo window (needs a desktop)
ros2 launch misty_ros gazebo.launch.py render_engine:=ogre depth:=false   # lighter, for a slow VM
```

- World: `misty_gz` → `world` in `misty_params.yaml` (markers, obstacles,
  walls; same format as `nav_sim`). Marker size and dictionary come from
  `misty_sensors`, so the simulated markers match the printed ones.
- `gz_adapter` stands in for Misty: it turns `/cmd_vel` into wheel speeds
  (same scaling and drive timeout as `misty_teleop`) and Gazebo's sensors into
  the real topics (`/misty/encoders`, `/misty/imu`, `/misty/tof`,
  `/misty/markers`, `/misty/depth_scan`, `/misty/camera/image/compressed`).
- `/sim/true_pose` is the true pose: compare it with `/slam/pose` in Foxglove's
  3D panel. `/sim/world` draws the markers and obstacles.
- Drive by hand with the Foxglove DRIVE panel, the controller or
  `keyboard_teleop`, or run the mission with NAV START / NAV GO.

Markers: `python3 scripts/make_aruco_markers.py` writes printable PNGs for the
mission ids (dictionary and size from `misty_sensors`); print at the width it
reports and set `marker_size_m` to the measured black square.

Keep the head centred while navigating (the camera bearing assumes it).

**First run on the robot, check:** `ros2 topic echo /misty/markers` shows
detections (else `aruco_source: opencv`); turning left increases the SLAM
heading (else `imu_yaw_sign: -1.0`); `ros2 topic echo /misty/tof` shows the
front sensors; drive 1 m and compare `/slam/odom_pose` (else `encoder_scale`).
