"""Locate and import the Misty Python SDK (the misty-python-sdk submodule).

ROS nodes run on the system Python, not in misty_venv, so the SDK installed in
the venv isn't importable. We only need mistyPy.RobotCommands (pure REST,
depends on `requests` only), so we find the submodule on disk and import it.
Search order: already importable -> $MISTY_SDK_PATH -> a `misty-python-sdk`
folder in any parent directory of this file (i.e. the workspace root).
"""
import os
import sys
from pathlib import Path


def _candidates():
    env = os.environ.get('MISTY_SDK_PATH')
    if env:
        yield Path(env)
    here = Path(__file__)
    for base in {here.absolute(), here.resolve()}:
        for parent in base.parents:
            yield parent / 'misty-python-sdk'


def robot_commands_class():
    try:
        from mistyPy.RobotCommands import RobotCommands
        return RobotCommands
    except ImportError:
        pass
    for path in _candidates():
        if (path / 'mistyPy' / 'RobotCommands.py').is_file():
            sys.path.insert(0, str(path))
            from mistyPy.RobotCommands import RobotCommands
            return RobotCommands
    raise ImportError(
        'Could not find the Misty Python SDK. Set MISTY_SDK_PATH to the '
        'misty-python-sdk directory.')


def make_robot(ip: str, timeout: float):
    """Return an SDK RobotCommands with a request timeout and kept-alive connections.

    - The SDK's helpers pass no timeout to `requests`, so a robot that drops off
      the network would block a node forever; every request gets `timeout`.
    - The SDK opens a new HTTP connection per command (~270 ms per request on
      Misty vs ~115 ms on a reused one), so each thread gets its own persistent
      requests.Session (Sessions aren't guaranteed thread-safe).
    """
    import threading

    import requests

    base = robot_commands_class()
    local = threading.local()

    class FastRobot(base):
        def _generic_request(self, verb, endpoint, **kwargs):
            kwargs.setdefault('timeout', timeout)
            session = getattr(local, 'session', None)
            if session is None:
                session = local.session = requests.Session()
            return session.request(verb, f'http://{self.ip}/api/{endpoint}', **kwargs)

    return FastRobot(ip)
