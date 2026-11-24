"""Run without ROS or a Misty robot: python3 -m unittest discover -s tests."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/misty_ros'))
from misty_ros.face_tracking_math import compute_head_command, parse_face_event


class FaceTrackingTests(unittest.TestCase):
    def test_misty_documented_event(self):
        raw = {'EventName': 'misty_face_tracker', 'Type': 'FaceRecognition',
               'Message': {'Bearing': -12, 'Elevation': 8, 'PersonName': 'unknown_person'}}
        self.assertEqual(parse_face_event(raw), (-12.0, 8.0))

    def test_lowercase_and_message_string(self):
        raw = '{"eventName":"misty_face_tracker","message":"{\\"bearing\\":-3,\\"elevation\\":5}"}'
        self.assertEqual(parse_face_event(raw), (-3.0, 5.0))

    def test_ignores_other_event_or_bad_payload(self):
        self.assertIsNone(parse_face_event({'eventName': 'Other', 'message': {'bearing': 5, 'elevation': 1}}))
        self.assertIsNone(parse_face_event({'eventName': 'misty_face_tracker', 'message': 'subscribed'}))
        self.assertIsNone(parse_face_event({'eventName': 'misty_face_tracker', 'message': {'bearing': 'nan', 'elevation': 1}}))
        self.assertIsNone(parse_face_event('not json'))

    def test_deadband(self):
        self.assertEqual(compute_head_command(4, -3), (0.0, 0.0))

    def test_small_nudge_and_clamp(self):
        linear, angular = compute_head_command(20, -10, gain=0.35, head_step_deg=13)
        self.assertLess(linear, 0)
        self.assertGreater(angular, 0)
        self.assertLessEqual(abs(linear), 0.25)
        self.assertLessEqual(abs(angular), 0.25)
        self.assertEqual(compute_head_command(9999, -9999), (-0.25, 0.25))

    def test_invert_directions(self):
        self.assertEqual(compute_head_command(100, 100, yaw_sign=-1, pitch_sign=-1), (-0.25, -0.25))


if __name__ == '__main__':
    unittest.main()
