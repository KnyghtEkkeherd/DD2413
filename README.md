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

## Wizard-of-Oz study (state machine from the study plan PDF)

```bash
ros2 launch misty_ros wizard.launch.py misty_ip:=<MISTY_IP> participant_id:=P01 condition:=BREAKDOWN
```

This starts the camera, teleop, foxglove_bridge and `misty_wizard` (the S0–S9
state machine). In Foxglove, **Layouts → Import from file →
`foxglove/misty_wizard_layout.json`** gives you the camera, drive/head teleop,
the wizard buttons and the status panels.

- **Before each participant:** set `participant_id` and `condition`
  (`CONTROL`/`BREAKDOWN`) in the Parameters panel (`/misty_wizard`), then press
  **START**. START is refused until both are valid.
- **Status panels:** `/wizard/state` (current state), `/wizard/allowed` (the
  buttons that do something now; others are logged and ignored), `/wizard/cue`
  (what to set in the Misty app now: mood, LEDs, gestures), `/wizard/said`.
- **Emotions are manual**: moods, LEDs and expressive head gestures are done
  in the Misty app, following `/wizard/cue`. Speech, route driving, look-backs,
  nudges and head-touch sensing are automatic.
- Buttons pressed while Misty is still talking are queued and handled once
  she's done (e.g. YES during "Are you ready?").
- **ABORT** stops driving and speech and ends the session at any time.

Config (reloaded at every START, no restart needed):
- `src/misty_ros/config/study.yaml`: timings, nudge sizes, look-back, optional
  control-pause padding, and the **route** (placeholder values; tune on the
  taped floor).
- `src/misty_ros/config/speech_bank.yaml`: all utterance variants.

Session logs: one CSV per session in `~/misty_ws/sessions/` (git-ignored),
columns `timestamp, participant_id, condition, state, event_source,
event, utterance_variant_index, detail`. A press made while Misty was busy
appears twice: `deferred (Misty busy)` (time of the press) and
`same press as deferred row above` (when it was handled). Count presses
without the second kind.

Dry run without the robot: `python3 scripts/fake_misty.py` and launch with
`misty_ip:=127.0.0.1:8099` (no touch sensor; use the TOUCH (sim) button).
