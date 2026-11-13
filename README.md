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
| ⚠ **FAILURE** | One press: "I'm lost" → random sad sound → "Can you help me?". Steps are in `failure_sequence` in `study.yaml`. **Cancel / stop talking** or **■ STOP all** cancels it |
| Speech tab | One button per speech-bank category (random variant, index logged), **Stop talking**, **Say custom text** (edit the text in the panel, then press) |
| Sounds tab | Misty's built-in sounds grouped by emotion (`config/sounds.yaml`) |
| DRIVE / HEAD | Teleop panels: drive (`/cmd_vel`), head/camera tilt + pan (`/misty/head_cmd`). Head moves 10° per message at 10 Hz while held (~100°/s); tune with launch args `head_step_deg:=` (degrees per message) and `head_velocity:=` (% of max head speed) |
| Route + motion | **Next segment ▶** (next segment from `route` in `study.yaml`), Look back, Head centre, Route reset, Nudge L / fwd / R, **■ STOP all** |
| Session | Set `participant_id` + `condition` in the Parameters panel, **START session** … **END session** |

Status lines: Session (recording?), Misty said / played, Last action / head
touch, Next route segment.

### Config and logs

Reloaded at every START (no restart needed):
- `src/misty_ros/config/study.yaml`: FAILURE sequence, route segments
  (placeholders: tune on the taped floor), nudge sizes, look-back.
- `src/misty_ros/config/speech_bank.yaml`: speech buttons and their variants.
- `src/misty_ros/config/sounds.yaml`: sound buttons. Check the names against
  the robot with `python3 scripts/check_misty_sounds.py <MISTY_IP>`.

After changing speech categories or sounds, run
`python3 scripts/make_foxglove_layout.py` and re-import the layout.

One CSV per session in `~/misty_ws/sessions/` (git-ignored): `timestamp,
participant_id, condition, event_source, event, utterance_variant_index,
detail`. `event_source` is `wizard` (button), `failure` (step of the FAILURE
sequence), `sensor` (head touch) or `auto`.

Dry run without the robot: `python3 scripts/fake_misty.py`, then launch with
`misty_ip:=127.0.0.1:8099` (no touch sensor: use **Log touch (sim)**).
