"""Pure helpers for the Misty face tracker; no ROS or robot required.

Misty FaceRecognition events report angular *errors* relative to the camera.
Commands below are incremental geometry_msgs/Twist values for this project's
misty_teleop node, NOT motor velocities in rad/s. Direction signs need a live
calibration, because Misty firmware/camera conventions may differ.
"""
import json
import math


def parse_face_event(raw, event_name='misty_face_tracker'):
    """Return (bearing_deg, elevation_deg) or None for other/invalid events."""
    try:
        event = json.loads(raw) if isinstance(raw, (str, bytes)) else raw
        if not isinstance(event, dict):
            return None
        event = {str(k).lower(): v for k, v in event.items()}
        if str(event.get('eventname', '')).lower() != event_name.lower():
            return None
        if 'type' in event and str(event['type']).lower() != 'facerecognition':
            return None
        payload = event.get('message')
        if isinstance(payload, str):
            payload = json.loads(payload)
        if not isinstance(payload, dict):
            return None
        payload = {str(k).lower(): v for k, v in payload.items()}
        bearing = float(payload['bearing'])
        elevation = float(payload['elevation'])
        if not (math.isfinite(bearing) and math.isfinite(elevation)):
            return None
        return bearing, elevation
    except (ValueError, KeyError, TypeError, OverflowError):
        return None


def compute_head_command(bearing, elevation, *, deadband_deg=5.0,
                         gain=0.35, head_step_deg=13.0, max_command=0.25,
                         yaw_sign=1.0, pitch_sign=1.0):
    """Return (linear_x, angular_z) for /misty/head_cmd.

    The existing teleop applies head_step_deg * each field per message.
    Limit commands to small nudges; one new face event -> at most one nudge.
    """
    if head_step_deg <= 0 or max_command <= 0:
        raise ValueError('head_step_deg and max_command must be positive')

    def axis(error, sign):
        if not math.isfinite(error) or abs(error) <= deadband_deg:
            return 0.0
        raw = sign * error * gain / head_step_deg
        return max(-max_command, min(max_command, raw))

    return axis(elevation, pitch_sign), axis(bearing, yaw_sign)
