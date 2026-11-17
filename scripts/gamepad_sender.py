#!/usr/bin/env python3
"""Send an Xbox (or any SDL-supported) controller from your Mac/PC to the VM.

Runs on the computer the controller is connected to, NOT on the VM:

    python3 -m venv ~/misty_pad && ~/misty_pad/bin/pip install pygame-ce
    ~/misty_pad/bin/python gamepad_sender.py 192.168.64.3          # VM IP
    ~/misty_pad/bin/python gamepad_sender.py 192.168.64.3 --show   # also print values

Reads the controller with SDL2's GameController API (standard Xbox layout on
every platform) and sends the state as JSON over UDP to the gamepad_teleop
node (default port 9876) about 30 times a second. If this script stops, the
robot stops within half a second.
"""
import argparse
import json
import os
import socket
import sys
import time

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
try:
    import pygame
    from pygame._sdl2 import controller as sdl_controller
except ImportError:
    sys.exit('pygame is required: python3 -m venv ~/misty_pad && ~/misty_pad/bin/pip install pygame-ce')

AXES = {'lx': 'CONTROLLER_AXIS_LEFTX', 'ly': 'CONTROLLER_AXIS_LEFTY',
        'rx': 'CONTROLLER_AXIS_RIGHTX', 'ry': 'CONTROLLER_AXIS_RIGHTY',
        'lt': 'CONTROLLER_AXIS_TRIGGERLEFT', 'rt': 'CONTROLLER_AXIS_TRIGGERRIGHT'}
BUTTONS = {'a': 'CONTROLLER_BUTTON_A', 'b': 'CONTROLLER_BUTTON_B',
           'x': 'CONTROLLER_BUTTON_X', 'y': 'CONTROLLER_BUTTON_Y',
           'lb': 'CONTROLLER_BUTTON_LEFTSHOULDER', 'rb': 'CONTROLLER_BUTTON_RIGHTSHOULDER',
           'back': 'CONTROLLER_BUTTON_BACK', 'start': 'CONTROLLER_BUTTON_START'}


def open_controller():
    sdl_controller.init()
    for i in range(sdl_controller.get_count()):
        if sdl_controller.is_controller(i):
            return sdl_controller.Controller(i)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('host', help='IP of the VM running gamepad_teleop (e.g. 192.168.64.3)')
    ap.add_argument('--port', type=int, default=9876)
    ap.add_argument('--rate', type=float, default=30.0)
    ap.add_argument('--show', action='store_true', help='print the values being sent')
    args = ap.parse_args()

    pygame.init()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pad = None
    print(f'Sending controller to {args.host}:{args.port}. Ctrl+C to quit.')
    try:
        while True:
            pygame.event.pump()                  # keeps SDL's controller state fresh
            for ev in pygame.event.get():
                if ev.type in (pygame.CONTROLLERDEVICEREMOVED, pygame.CONTROLLERDEVICEADDED):
                    pad = None                   # re-open on the next loop
            if pad is None:
                pad = open_controller()
                if pad is None:
                    print('\rNo controller found, waiting...', end='', flush=True)
                    time.sleep(1.0)
                    continue
                print(f'\rController: {pad.name}' + ' ' * 20)
            state = {'name': pad.name, 't': time.time()}
            for key, const in AXES.items():
                state[key] = round(pad.get_axis(getattr(pygame, const)) / 32767.0, 3)
            for key, const in BUTTONS.items():
                state[key] = int(pad.get_button(getattr(pygame, const)))
            sock.sendto(json.dumps(state).encode(), (args.host, args.port))
            if args.show:
                pressed = ' '.join(k.upper() for k in BUTTONS if state[k])
                print(f"\rL ({state['lx']:+.2f},{state['ly']:+.2f})  R ({state['rx']:+.2f},{state['ry']:+.2f})  "
                      f"{pressed:<20}", end='', flush=True)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print('\nstopped')


if __name__ == '__main__':
    main()
