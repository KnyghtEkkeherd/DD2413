#!/usr/bin/env python3
"""Minimal fake Misty REST API for dry-running the ROS nodes without the robot.

    python3 scripts/fake_misty.py [port] [request_log]
    ros2 launch misty_ros wizard.launch.py misty_ip:=127.0.0.1:8099 ...

Serves a placeholder camera image, accepts every command, and logs each
request. It has no event websocket, so the wizard falls back to estimated
speech timing and touch must be simulated with the TOUCH (sim) button.
"""
import base64
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8099
LOG = open(sys.argv[2] if len(sys.argv) > 2 else '/dev/stdout', 'w', buffering=1)
IMG = b'\xff\xd8\xff\xe0FAKEJPEG-' + bytes(range(256)) + b'\xff\xd9'


class Handler(BaseHTTPRequestHandler):
    def _body(self):
        n = int(self.headers.get('Content-Length') or 0)
        try:
            return json.loads(self.rfile.read(n) or b'{}')
        except ValueError:
            return {}

    def _send(self, obj):
        b = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        body = self._body()
        if self.path != '/api/cameras/rgb':
            LOG.write(f'GET {self.path} {json.dumps(body)}\n')
        if self.path == '/api/cameras/rgb':
            self._send({'result': {'base64': base64.b64encode(IMG).decode(), 'contentType': 'image/jpeg',
                                   'width': body.get('width'), 'height': body.get('height')},
                        'status': 'Success'})
        else:
            self._send({'result': True, 'status': 'Success'})

    def do_POST(self):
        LOG.write(f'POST {self.path} {json.dumps(self._body())}\n')
        self._send({'result': True, 'status': 'Success'})

    def log_message(self, *args):
        pass


print(f'Fake Misty on 127.0.0.1:{PORT}', file=sys.stderr)
ThreadingHTTPServer(('127.0.0.1', PORT), Handler).serve_forever()
