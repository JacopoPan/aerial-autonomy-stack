"""
Stand-in for the pilot's RC: streams the Betaflight SITL's UDP RC packet with the MSP OVERRIDE switch (CH7) on

While betaflight_interface sends MSP_SET_RAW_RC, its frames replace CH1-6 (see betaflight-2026.6.cli)
If they stop (e.g., betaflight_interface restarting), Betaflight falls back to these values: i.e. a hands-off pilot, armed in POS HOLD
If this script stops (the radio is lost), betaflight_interface keeps flying (msp_override_failsafe = ON in betaflight-2026.6.cli)
On the ground, Betaflight does not arm on them (the throttle is not low)

Use as:
    python3 betaflight_simulated_rc.py <port> (9004 + port offset) <betaflight-2026.6.cli>
"""
import re
import socket
import struct
import sys
import time

port = int(sys.argv[1])
with open(sys.argv[2]) as cli_file:
    cli = cli_file.read()
hover = int(re.search(r'^set ap_hover_throttle = (\d+)', cli, re.MULTILINE).group(1))
min_check = int(re.search(r'^set min_check = (\d+)', cli, re.MULTILINE).group(1))
throttle = round(min_check + (hover - 1000) * (2000 - min_check) / 1000) # ALTHOLD's center: ap_hover_throttle before Betaflight's min_check rescaling (as betaflight_interface's hold_throttle(0))

channels = [1500, 1500, throttle, 1500, 2000, 2000, 2000] + [1000] * 9 # Roll, pitch, throttle, yaw, CH5 ARM, CH6 flight mode, CH7 MSP OVERRIDE, CH8-16 unused
packet = struct.pack('<d16H', 0.0, *channels) # SITL rc_packet: timestamp (unused) and 16 channels
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    sock.sendto(packet, ('127.0.0.1', port))
    time.sleep(0.01) # 100 Hz, wall clock (RTF is locked to 1)
