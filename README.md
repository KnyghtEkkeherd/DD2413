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
| Y | head centre |
| LB / RB | slower / faster driving |

If the sender stops or the network drops, Misty stops within 0.5 s.
Speeds, deadzone, speed levels and axis inversion are in `misty_params.yaml`.

### Keyboard (in an SSH terminal)

`ros2 run misty_ros keyboard_teleop`: W/S forward/back, A/D turn, Q/E arcs,
I/K/J/L or arrow keys for the head, C head centre, Space STOP, Z/X speed.

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
