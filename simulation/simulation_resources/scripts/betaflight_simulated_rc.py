"""
Stand-in for the pilot's RC: streams the Betaflight SITL's UDP RC packet with the MSP OVERRIDE switch (CH7) on

While betaflight_interface sends MSP_SET_RAW_RC, its frames replace CH1-6 (see betaflight-2026.6.cli)
If they stop (e.g., betaflight_interface restarting), Betaflight falls back to these values: a hands-off pilot, armed in POS HOLD
If this script stops (the radio is lost), betaflight_interface keeps flying (msp_override_failsafe = ON in betaflight-2026.6.cli)
On the ground, Betaflight does not arm on them (the throttle is not low)

Use as:
    python3 betaflight_simulated_rc.py <port> (9004 + port offset)
"""
import socket
import struct
import sys
import time

# Roll, pitch, throttle, yaw, CH5 ARM, CH6 flight mode, CH7 MSP OVERRIDE, CH8-16 unused
# The throttle is ALTHOLD's center: ap_hover_throttle 1595 before Betaflight's min_check rescaling (as betaflight_interface's hold_throttle(0))
CHANNELS = [1500, 1500, 1615, 1500, 2000, 2000, 2000] + [1000] * 9
packet = struct.pack('<d16H', 0.0, *CHANNELS) # SITL rc_packet: timestamp (unused) and 16 channels
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    sock.sendto(packet, ('127.0.0.1', int(sys.argv[1])))
    time.sleep(0.01) # 100 Hz, wall clock (RTF is locked to 1)
