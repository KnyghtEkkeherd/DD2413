# Misty eye expressions from Foxglove buttons

This patch changes only two existing files:
- `src/misty_ros/misty_ros/wizard_node.py`
- `src/misty_ros/config/study.yaml`

It is based on the **faster face-tracking patch** provided earlier. If you changed
`wizard_node.py` or `study.yaml` locally, compare/merge `eye_expressions.diff`
instead of blindly replacing either file.

## Behavior

- `REACT:FAILURE` (Breakdown): show `e_Sadness.jpg` for the **whole sequence**;
  return to `e_DefaultContent.jpg` after the sequence completes or is canceled.
- Other mapped reaction buttons: display the configured eyes for
  `one_shot_hold_s` seconds (default 4), then return to default eyes.
  This is a fixed hold time, **not** exact speech-completion timing.
- A mapped standalone event (e.g. `LOOK_BACK`) also displays its mapped eyes.
- `STOP`, `STOP_TALKING`, `END`, and node shutdown restore neutral eyes.
- A newer expression takes precedence over an older timer or finished sequence;
  older timers cannot reset a newer expression.
- Unmapped buttons keep the previous behavior (no new expression).
- The `condition:=BREAKDOWN` argument only chooses the experiment condition;
  it does **not** trigger the `FAILURE` button's eyes automatically.

## Edit expressions

Edit `src/misty_ros/config/study.yaml`:

```yaml
eyes:
  default: neutral
  one_shot_hold_s: 4.0
  emotions:
    neutral: e_DefaultContent.jpg
    sad: e_Sadness.jpg
    happy: e_Joy.jpg
  reactions:
    FAILURE: sad
    GREETING: happy
  buttons:
    LOOK_BACK: happy
```

The keys of `reactions:` must match IDs from
`src/misty_ros/config/reactions.json`, e.g. `FAILURE` or `GREETING`.
For standalone button events, use `buttons:`, e.g. `LOOK_BACK`.
You do not need to change or re-import the Foxglove layout if those buttons
already exist. YAML is reloaded at session `START` or on wizard restart.

## Test without Misty

```bash
python3 -m py_compile src/misty_ros/misty_ros/wizard_node.py
python3 -c "import yaml; print(yaml.safe_load(open('src/misty_ros/config/study.yaml'))['eyes'])"
```

## Install / test with Misty

```bash
cd '/mnt/c/Users/lucfi/Documents/Github Local/DD2413'
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
ros2 launch misty_ros wizard.launch.py misty_ip:=192.168.1.237 participant_id:=P01 condition:=BREAKDOWN
```

In Foxglove, press `FAILURE` (Breakdown). Check for sad eyes during the
reaction and neutral eyes afterward. Then test `STOP_TALKING` mid-sequence;
eyes should reset immediately. Verify all image assets exist on the robot;
using an unknown image name yields a Misty API error.

To test only the display endpoint from your computer, while connected to Misty:

```bash
curl -sS -X POST 'http://192.168.1.237/api/images/display' \
  -H 'Content-Type: application/json' \
  -d '{"FileName":"e_Sadness.jpg","Alpha":1}'
```

If your study needs blink/overlay behavior beyond switching images, these
commands can be extended with display layers or blink mappings later.
