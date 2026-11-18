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

## Running the study (one launch)

```bash
ros2 launch misty_ros study.launch.py misty_ip:=<MISTY_IP> participant_id:=P01 condition:=BREAKDOWN
ros2 launch misty_ros study.launch.py sim:=2d      # dry run: 2D simulator + fake Misty, no robot
```

Starts everything: camera, teleop, Xbox controller, twist_mux, foxglove_bridge,
the wizard (speech, emotions, gestures, session log), the **study state
machine** and **Nav2 navigation**. Needs `bash scripts/install_nav2.sh` once.
In Foxglove: **Layouts → Import from file… → `foxglove/misty_wizard_layout.json`**
(don't drag the file in: that opens it as data and fails).

**Who does what**

| | |
|---|---|
| Study state machine (`config/study.yaml`, from the study plan) | drives the session: APPROACH (Nav2 to marker 1) → GREETING → CONSENT (YES / NO, nudges) → ESCORT_A (Nav2 to marker 2, automatic look-backs) → HALF_PAUSE → CONTROL: CHECK_ROUTE / BREAKDOWN: CONFUSION → FRUSTRATION → LOST_APPEAL → HELP_WAIT → RECOVERY → ESCORT_B (Nav2 to marker 3) → ARRIVE → END. Each state sets Misty's emotion and says its line; transitions on arrival, speech finished, timeouts, head touch and wizard answers |
| Wizard (Foxglove / controller) | answers (**YES, NO, Direction OK / wrong, RECOVER, Distracted**), **emotions** and **hand gestures** (any time), and the **state buttons**: jump to any state, back or forward (logged) |
| Driving | Nav2 between markers. The wizard can take over any time: Foxglove DRIVE panel or controller left stick **override Nav2** (twist_mux) |
| **■ STOP all** / controller B | stops speech + wheels and **pauses** the study and Nav2 (timers freeze); the wizard can still drive by hand. **▶ RESUME** / controller START continues |

The condition (from `condition` / the Parameters panel at START) picks the
branch at the halfway marker. Everything is logged in the session CSV:
state changes (`STATE`, with the reason), wizard buttons, emotions, gestures,
every sentence with its variant index, head touches.

**Edit without rebuilding** (applies at the next START): `src/misty_ros/config/study.yaml`
(states, transitions, timings, emotions = face image + LEDs + arms, gestures) and
`reactions.json` (what Misty says). After adding states / emotions / gestures,
re-run `python3 scripts/make_foxglove_layout.py` and re-import the layout.

### Motor control: twist_mux

Every driving source publishes to its own twist_mux input (`config/twist_mux.yaml`);
only the winner reaches Misty (`/misty/cmd_vel` → teleop_node):

| Input | Priority |
|---|---|
| Foxglove DRIVE panel, Xbox controller | 100 (always wins) |
| keyboard_teleop | 90 |
| study node | 50 |
| Nav2 | 10 |
| lock `/wizard/estop` (STOP) | blocks everything below 95 |
| lock `/study/hold_nav` (states that don't drive, look-backs) | blocks Nav2 |

Speeds: `/**` in `misty_params.yaml` (`linear_scale`, `angular_scale` for
manual driving; `max_linear_mps`, `max_angular_dps` = Misty at 100 %, calibrate).

## Wizard menu only (no state machine)

```bash
ros2 launch misty_ros wizard.launch.py misty_ip:=<MISTY_IP> participant_id:=P01 condition:=BREAKDOWN
```

Camera, teleop, controller, foxglove_bridge and the wizard, without the study
node (add `nav:=true` for Nav2). The wizard chooses every reaction.

| Area | What it does |
|------|--------------|
| ⚠ **FAILURE** | One press: "I'm lost" → random sad sound → "Can you help me?" (a sequence in `reactions.json`). **Stop talking / cancel** or **■ STOP all** cancels it |
| Reaction tabs | One tab per `group` in `reactions.json` (Escort, Breakdown, Responses, Sounds…): each button says a random sentence of that reaction in its voice (index logged), or plays a sound. **Custom text** tab: edit the text, optional voice prefix like `[sad] …`, then press |
| Emotions & hands | Emotion presets (face + LEDs + arms) and arm gestures from `study.yaml` |
| DRIVE / HEAD | Teleop panels: drive (`/cmd_vel`, via twist_mux), head/camera tilt + pan (`/misty/head_cmd`) |
| Session | Set `participant_id` + `condition` in the Parameters panel, **START session** … **END session** |

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
| B | STOP: wheels, speech, sounds; pauses the study + Nav2 (driving stays off until the left stick is released) |
| START | RESUME the study + Nav2 after a STOP |
| X | Look back at the participant |
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

## Autonomous navigation with Nav2 (default)

Built from standard ROS 2 packages: **Nav2** (planning, Regulated Pure Pursuit
path following, recoveries, collision monitor), **robot_localization** (EKF),
**aruco_opencv** (marker detection), **depthimage_to_laserscan**. Install once
(sudo; apt packages + builds cv_bridge and aruco_opencv from source, because
their apt builds need Ubuntu 26.04's Boost):

```bash
bash scripts/install_nav2.sh
```

```bash
ros2 launch misty_ros wizard.launch.py misty_ip:=<MISTY_IP> nav:=true   # real robot
ros2 launch misty_ros nav2.launch.py sim:=2d                            # 2D simulator, no robot
```

Options for `wizard.launch.py`: `markers:=json` uses Misty's on-board ArUco
detector instead of aruco_opencv on the camera stream; `depth:=true` adds the
depth camera as a laser scan; `nav_stack:=custom` runs the older custom stack
(below) instead.

**Before the first real run:**
1. Measure the markers: `marker_map` at the top of `misty_params.yaml`,
   `{"id": [x, y, facing_deg]}` in metres. Choose any origin and x direction
   on the floor; `facing_deg` is where the printed side faces. Misty's start
   pose doesn't matter: the first marker she sees localizes her.
2. Calibrate speeds (`misty_teleop`: `max_linear_mps`, `max_angular_dps`):
   drive at 100 % for 2 s, measure. Nav2 commands real m/s; these convert them.
3. Check `imu_yaw_sign` (`misty_sensors`): turning left must increase yaw.
4. Measure the camera position on your Misty and fix `camera_joint` in
   `urdf/misty.urdf` if needed; keep the head centred while navigating.

The route, roles and Foxglove buttons are unchanged: `mission`,
`start_markers` / `fail_markers` / `goal_markers`, `fail_trigger`, NAV START /
NAV GO / NAV PAUSE / NAV RESET. Each target becomes a Nav2 goal `standoff_m`
in front of its marker, facing it. Until a marker has been seen, Misty turns on
the spot to find one.

How the pieces connect:

| Node | Package | Does |
|---|---|---|
| `misty_sensors`, `misty_camera`, `misty_teleop` | misty_ros | Misty's HTTP / websocket API ↔ ROS (no open-source Misty driver exists) |
| `robot_state_publisher` | standard | `urdf/misty.urdf`: camera, ToF, depth frames |
| `misty_wheel_odom` | misty_ros | encoders → `nav_msgs/Odometry` |
| `ekf_odom`, `ekf_map` | robot_localization | odom + IMU → `odom→base_link`; + marker fixes → `map→odom` (`config/localization.yaml`) |
| `aruco_tracker` | aruco_opencv | marker poses from the camera stream |
| `misty_marker_localizer` | misty_ros | marker pose + `marker_map` → robot pose fix |
| Nav2 servers | nav2_* | `config/nav2_params.yaml`; output `/nav/cmd_vel` (m/s) → `misty_teleop` |
| `misty_nav` (mission) | misty_ros | NAV buttons → `NavigateToPose` goals, reactions |

Single-marker heading is only trusted within 2 m and when the solved marker
faces the camera: OpenCV 4.10's IPPE solver (used by aruco_opencv) sometimes
returns the flipped solution; those sightings give position only.

## Custom navigation stack (fallback: `nav_stack:=custom`)

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
