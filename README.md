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

## Wizard-of-Oz study

```bash
ros2 launch misty_ros wizard.launch.py misty_ip:=<MISTY_IP> participant_id:=P01 condition:=BREAKDOWN
```

Starts the camera, teleop, foxglove_bridge and the wizard. In Foxglove:
**Layouts → Import from file → `foxglove/misty_wizard_manual_layout.json`**.

### Manual mode (default): the wizard does everything

Nothing happens unless the wizard presses something. Emotions (mood, face,
LEDs, expressive gestures) are done in the **Misty phone app**.

| Area | Buttons / panels |
|------|------------------|
| Camera | Image panel |
| Drive | **DRIVE** Teleop panel (`/cmd_vel`): arrows = forward / back / turn |
| Head / camera | **HEAD / CAMERA** Teleop panel (`/misty/head_cmd`): up/down = tilt, left/right = pan. Hold an arrow to keep moving (5° per message at 5 Hz). Stays in sync with Look back / Head centre. Tune with launch args `head_step_deg:=` and `head_velocity:=` |
| Route | **Next segment** (drives the next segment from `study.yaml`), Look back, Head centre, Route reset, Nudge L / fwd / R, **■ STOP** (driving + speech) |
| Phase | Greeting … Arrive: marks the study phase in the log and shows the matching emotion cue |
| Speech | One button per speech-bank category (random variant, index logged), **Stop talking**, **Say custom text** (edit the text in the panel, then press) |
| Session | Set `participant_id` + `condition` in the Parameters panel, then **START session**; **END session** saves the CSV |

Status lines: `/wizard/state` (phase, participant, recording), `/wizard/cue`
(what to set in the phone app + which line to say), `/wizard/said`,
`/wizard/last` (last action, head touches), `/wizard/allowed` (next route segment).

### Auto mode: state machine from the study plan

`mode:=auto` runs the S0–S9 state machine (speech, route, timeouts automatic;
the wizard answers YES / NO / END SUCCESS / END FAIL / DIR CORRECT … ). Use
`foxglove/misty_wizard_auto_layout.json`. Buttons pressed while Misty is
talking are handled when she's done; `/wizard/allowed` shows valid buttons.

### Config and logs

Reloaded at every START (no restart needed):
- `src/misty_ros/config/study.yaml`: route segments (placeholders: tune on the
  taped floor), nudge sizes, look-back, timings (auto mode).
- `src/misty_ros/config/speech_bank.yaml`: utterance variants. After adding or
  renaming a category, run `python3 scripts/make_foxglove_layout.py` and
  re-import the layout.

One CSV per session in `~/misty_ws/sessions/` (git-ignored): `timestamp,
participant_id, condition, state, event_source, event,
utterance_variant_index, detail`. Head touches are logged automatically from
Misty's sensors. In auto mode a press made while Misty was busy appears twice
(`deferred (Misty busy)`, then `same press as deferred row above`); count only
the first.

Dry run without the robot: `python3 scripts/fake_misty.py`, then launch with
`misty_ip:=127.0.0.1:8099` (no touch sensor: use **Log touch (sim)**).
