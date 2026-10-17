"""
Stand-in for the pilot's RC: streams the Betaflight SITL's UDP RC packet with the MSP OVERRIDE switch (CH7) on

While betaflight_interface sends MSP_SET_RAW_RC, its frames replace CH1-6 (see betaflight-2026.6.cli)
If they stop (e.g., betaflight_interface restarting), Betaflight falls back to these values: i.e. a hands-off pilot, armed in POS HOLD
If this script stops (the radio is lost), betaflight_interface keeps flying (msp_override_failsafe = ON in betaflight-2026.6.cli)
On the ground, Betaflight does not arm on them (the throttle is not low)

With a gamepad (e.g. an Xbox controller connected via USB on /dev/input/js0), it allows manual flight:
    Each press of the "View" button toggles MSP OVERRIDE: high (default at the start), betaflight_interface flies; low, the gamepad flies (back to high, betaflight_interface resumes from its current state: if it never armed, it disarms the drone)
    Each press of the "Menu" button toggles ARM: it is high by default so that a flying drone stays armed
        Betaflight arms on a low-to-high flip, with the throttle down, not in POS HOLD; to arm:
        Press the "View" button (stop MSP OVERRIDE), press X or Y (ACRO/ANGLE modes), hold the left stick (throttle) down, press the "Menu" button twice to cycle it
        (optional, for beginners) once armed, press A to start flying in POS HOLD mode
    Left stick: throttle and yaw; right stick: pitch and roll
    Press X for ACRO, Y for ANGLE, A for POS HOLD (i.e., ANGLE + ALTHOLD + POS HOLD)

Use as:
    python3 betaflight_simulated_rc.py <port> (9004 + port offset) <betaflight-2026.6.cli> [/dev/input/js0]
"""
import os
import re
import socket
import struct
import sys
import time

# Check with $ jstest /dev/input/js0
AXIS_YAW, AXIS_THROTTLE, AXIS_ROLL, AXIS_PITCH = 0, 1, 3, 4 # Linux xpad driver numbering
MAX_AXIS_TRAVEL = 32767
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80 # Event types of the Linux joystick API (linux/joystick.h)
BUTTON_A, BUTTON_X, BUTTON_Y, BUTTON_VIEW, BUTTON_MENU = 0, 2, 3, 6, 7
DEADBAND = 0.1 # Of the stick travel, gamepad sticks do not center exactly
FULL_STICK_YAW = 120 / 670 # 120 deg/s at full stick, Betaflight's linear yaw rate is 670 deg/s (yaw_rc_rate in betaflight-2026.6.cli)
FULL_STICK_ACRO = 200 / 670 # 200 deg/s at full roll or pitch stick in ACRO, from Betaflight's linear 670 deg/s

port = int(sys.argv[1])
with open(sys.argv[2]) as cli_file:
    cli = cli_file.read()
hover = int(re.search(r'^set ap_hover_throttle = (\d+)', cli, re.MULTILINE).group(1))
min_check = int(re.search(r'^set min_check = (\d+)', cli, re.MULTILINE).group(1))
throttle = round(min_check + (hover - 1000) * (2000 - min_check) / 1000) # ALTHOLD's center: ap_hover_throttle before Betaflight's min_check rescaling (as betaflight_interface's hold_throttle(0))
channels = [1500, 1500, throttle, 1500, 2000, 2000, 2000] + [1000] * 9 # Roll, pitch, throttle, yaw, CH5 ARM, CH6 flight mode, CH7 MSP OVERRIDE, CH8-16 unused

# Check for a connected USB gamepad
gamepad = None
if len(sys.argv) > 3 and os.path.exists(sys.argv[3]):
    gamepad = os.open(sys.argv[3], os.O_RDONLY | os.O_NONBLOCK)
    print(f'Gamepad {sys.argv[3]}, Xbox controller mapping: the View button toggles MSP OVERRIDE, the Menu button toggles ARM, X/Y/A select ACRO/ANGLE/POS HOLD modes', flush=True)
roll_stick = pitch_stick = yaw_stick = throttle_stick = 0.0 # From -1 to 1, right and up positive

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:

    # Handle gamepad inputs
    while gamepad is not None:
        try:
            # I: time in ms (unused, _), h: axis/button value, B: whether an axis or button, B: which axis or button
            _, value, event_type, number = struct.unpack('<IhBB', os.read(gamepad, 8))
        except BlockingIOError: # No new event
            break
        except (OSError, struct.error): # Unplugged: center the sticks, keep the switches
            print('Gamepad lost', flush=True)
            roll_stick = pitch_stick = yaw_stick = throttle_stick = 0.0
            gamepad = None
            break
        stick = value / MAX_AXIS_TRAVEL if abs(value) > DEADBAND * MAX_AXIS_TRAVEL else 0.0 # Up and left are negative
        if event_type & JS_EVENT_INIT:
            continue # Ignore startup state (non-centered sticks, pressed buttons)
        if event_type == JS_EVENT_AXIS:
            if number == AXIS_ROLL:
                roll_stick = stick
            elif number == AXIS_PITCH:
                pitch_stick = -stick
            elif number == AXIS_THROTTLE:
                throttle_stick = -stick
            elif number == AXIS_YAW:
                yaw_stick = stick
        elif event_type == JS_EVENT_BUTTON and value == 1: # Button pressed (0 is released)
            if number == BUTTON_VIEW:
                channels[6] = 1000 if channels[6] == 2000 else 2000 # Toggle
                print('MSP OVERRIDE', 'on: betaflight_interface flies' if channels[6] == 2000 else 'off: the gamepad flies', flush=True)
            elif number == BUTTON_MENU:
                channels[4] = 1000 if channels[4] == 2000 else 2000 # Toggle
                print('ARM', 'on' if channels[4] == 2000 else 'off', flush=True)
            elif number == BUTTON_X:
                channels[5] = 1000
                print('ACRO', flush=True)
            elif number == BUTTON_Y:
                channels[5] = 1500
                print('ANGLE', flush=True)
            elif number == BUTTON_A:
                channels[5] = 2000
                print('POS HOLD', flush=True)
    # Roll and pitch: rates in ACRO (scaled down), angles in ANGLE (60 deg at full stick), velocities in POS HOLD (5 m/s)
    scale = FULL_STICK_ACRO if channels[5] == 1000 else 1.0
    channels[0] = round(1500 + 500 * scale * roll_stick)
    channels[1] = round(1500 + 500 * scale * pitch_stick)
    # Throttle: centered, the hover throttle; full up, 2000; full down, min_check in POS HOLD, else 1000 (to arm)
    full_down = min_check if channels[5] == 2000 else 1000
    if throttle_stick > 0:
        channels[2] = round(throttle + throttle_stick * (2000 - throttle))
    else:
        channels[2] = round(throttle + throttle_stick * (throttle - full_down))
    # Yaw: always a rate
    channels[3] = round(1500 + 500 * FULL_STICK_YAW * yaw_stick)

    # SITL rc_packet: timestamp (unused) and 16 channels
    sock.sendto(struct.pack('<d16H', 0.0, *channels), ('127.0.0.1', port))
    time.sleep(0.01) # 100 Hz, wall clock (RTF is locked to 1)
