"""Minimal RC sender for Betaflight SITL

Streams the SITL rc_packet (double timestamp, uint16 channels[16]) over UDP at a fixed rate
Channel map (see betaflight-2026.6.cli):
    CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw, CH5 ARM, CH6 AUTOPILOT, CH7 MSP OVERRIDE (ch 7 2000 hands CH1-4 to betaflight_interface)
Commands:
    arm | disarm | ap | noap | thr <1000-2000> | ch <1-16> <1000-2000> | q

Betaflight's RX failsafe triggers when the stream stops, so keep this running while flying
"""
import argparse
import os
import socket
import struct
import threading
import time

SIM_IP = f"{os.environ['SIM_SUBNET']}.90.{os.environ['SIM_ID']}" if 'SIM_ID' in os.environ else '127.0.0.1'
LOW, HIGH = 1000, 2000
THR, ARM, AUTOPILOT = 2, 4, 5 # 0-based channel indices
HELP = 'Commands: arm, disarm, ap, noap, thr N, ch I N, q'


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ip', default=SIM_IP, help='Betaflight SITL address (default: simulation container)')
    parser.add_argument('--port', type=int, default=9004, help='Betaflight SITL RC port')
    parser.add_argument('--rate', type=float, default=50.0, help='Send rate in Hz (wall clock)')
    args = parser.parse_args()

    channels = [1500] * 16
    channels[THR] = channels[ARM] = channels[AUTOPILOT] = LOW # Throttle low, disarmed, autopilot off
    stop = threading.Event()

    def stream():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while not stop.is_set():
            sock.sendto(struct.pack('<d16H', time.time(), *channels), (args.ip, args.port))
            time.sleep(1.0 / args.rate)

    threading.Thread(target=stream, daemon=True).start()
    print(f'Sending RC to {args.ip}:{args.port} at {args.rate:g} Hz. {HELP}')
    switches = {'arm': (ARM, HIGH), 'disarm': (ARM, LOW), 'ap': (AUTOPILOT, HIGH), 'noap': (AUTOPILOT, LOW)}
    while True:
        try:
            words = input('rc> ').split()
        except (EOFError, KeyboardInterrupt):
            words = ['q']
        if not words:
            continue
        if words[0] == 'q':
            break
        try:
            if words[0] == 'arm': # Betaflight only arms on a low-to-high switch edge (e.g., after ARM_SWITCH blocks)
                channels[ARM] = LOW
                time.sleep(0.2)
            if words[0] in switches:
                index, value = switches[words[0]]
            elif words[0] == 'thr':
                index, value = THR, int(words[1])
            elif words[0] == 'ch':
                index, value = int(words[1]) - 1, int(words[2])
            else:
                raise ValueError(words[0])
            if not 0 <= index < len(channels):
                raise ValueError(index)
            channels[index] = max(LOW, min(HIGH, value))
            print('CH1-8:', channels[:8])
        except (IndexError, ValueError):
            print(HELP)
    channels[ARM] = channels[AUTOPILOT] = LOW # Disarm before leaving (Betaflight would failsafe anyway)
    time.sleep(0.2)
    stop.set()


if __name__ == '__main__':
    main()
