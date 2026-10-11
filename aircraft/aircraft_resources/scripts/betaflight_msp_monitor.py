"""
Print the Betaflight SITL's state from UART1, read-only (betaflight_interface uses UART2 and is not affected)
Every period: active modes, arming disable flags, RC channels (as Betaflight uses them: roll, pitch, yaw, throttle, AUX1-4 = CH5-8),
altitude and vertical speed (from the arming point, once armed), GPS fix and satellites

Use as:
    python3 /aas/aircraft_resources/scripts/betaflight_msp_monitor.py --ip ${SIM_SUBNET}.90.${SIM_ID} --port $((5761 + 10 * (DRONE_ID - 1))) --period 0.5

Note: requires the Betaflight App to be disconnected from the SITL instance
"""
import argparse
import socket
import struct
import time

MSP_RC, MSP_RAW_GPS, MSP_ALTITUDE, MSP_BOXIDS, MSP_STATUS_EX = 105, 106, 109, 119, 150
BOX_NAMES = {0: 'ARM', 1: 'ANGLE', 2: 'HORIZON', 3: 'ALTHOLD', 11: 'POSHOLD', 27: 'FAILSAFE', 28: 'AIRMODE', 46: 'GPSRESCUE',
             50: 'MSPOVERRIDE', 56: 'AUTOPILOT'} # Permanent IDs (msp_box.c), others print as numbers
ARMING_DISABLE_FLAGS = ['NOGYRO', 'FAILSAFE', 'RXLOSS', 'NOT_DISARMED', 'BOXFAILSAFE', 'RUNAWAY', 'CRASH', 'THROTTLE', 'ANGLE',
                        'BOOTGRACE', 'NOPREARM', 'LOAD', 'CALIB', 'CLI', 'CMS', 'BST', 'MSP', 'PARALYZE', 'GPS', 'RESCUE_SW',
                        'DSHOT_TELEM', 'REBOOT_REQD', 'DSHOT_BBANG', 'NO_ACC_CAL', 'MOTOR_PROTO', 'FLIP_SWITCH', 'ALT_HOLD_SW',
                        'POS_HOLD_SW', 'AUTOPILOT_SW', 'ARM_SWITCH'] # Bit order (runtime_config.c)

def request(sock, cmd): # MSP v1 request without payload: $M< size cmd checksum (size ^ cmd)
    sock.sendall(b'$M<' + bytes([0, cmd, cmd]))

def split_replies(buffer): # MSP v1 replies $M> size cmd payload checksum: returns [(cmd, payload)] and the incomplete rest
    replies = []
    while (start := buffer.find(b'$M>')) >= 0:
        buffer = buffer[start:]
        if len(buffer) < 6 or len(buffer) < 6 + buffer[3]:
            return replies, buffer
        replies.append((buffer[4], buffer[5:5 + buffer[3]]))
        buffer = buffer[6 + buffer[3]:]
    return replies, buffer[-2:] # Keep a split header

def main():
    parser = argparse.ArgumentParser(description='Print the Betaflight SITL state from MSP on UART1')
    parser.add_argument('--ip', required=True, help='Betaflight SITL address')
    parser.add_argument('--port', type=int, required=True, help='UART1 port (5761 + port offset)')
    parser.add_argument('--period', type=float, default=0.5, help='Print period in s (wall clock)')
    args = parser.parse_args()

    sock = socket.create_connection((args.ip, args.port), timeout=5.0)
    print(f'MSP on {args.ip}:{args.port}', flush=True)
    box_ids, buffer, state, t0 = [], b'', {}, time.monotonic()
    request(sock, MSP_BOXIDS) # Once, to name the bits of the active modes
    while True:
        for cmd in (MSP_STATUS_EX, MSP_RC, MSP_ALTITUDE, MSP_RAW_GPS):
            request(sock, cmd)
        deadline = time.monotonic() + args.period
        while (left := deadline - time.monotonic()) > 0:
            sock.settimeout(left)
            try:
                data = sock.recv(4096)
            except socket.timeout:
                break
            if not data:
                raise SystemExit('MSP link closed')
            replies, buffer = split_replies(buffer + data)
            for cmd, p in replies:
                if cmd == MSP_BOXIDS:
                    box_ids = list(p)
                elif cmd == MSP_STATUS_EX and len(p) >= 16 and len(p) >= 21 + p[15]: # Active modes (32 bits, then p[15] more bytes), arming disable flags
                    modes = struct.unpack_from('<I', p, 6)[0] | int.from_bytes(p[16:16 + p[15]], 'little') << 32
                    flags = struct.unpack_from('<I', p, 17 + p[15])[0]
                    state['modes'] = '+'.join(BOX_NAMES.get(b, str(b)) for i, b in enumerate(box_ids) if modes >> i & 1) or '-'
                    state['flags'] = '+'.join(name for i, name in enumerate(ARMING_DISABLE_FLAGS) if flags >> i & 1) or '-'
                elif cmd == MSP_RC and len(p) >= 16: # Roll, pitch, yaw, throttle, AUX1-4
                    v = struct.unpack_from('<8H', p)
                    state['rc'] = f'roll {v[0]} pitch {v[1]} yaw {v[2]} throttle {v[3]} aux {v[4]} {v[5]} {v[6]} {v[7]}'
                elif cmd == MSP_ALTITUDE and len(p) >= 6: # Altitude in cm, vertical speed in cm/s
                    state['alt'] = f"{struct.unpack_from('<i', p, 0)[0] / 100:.2f} m, {struct.unpack_from('<h', p, 4)[0] / 100:+.2f} m/s"
                elif cmd == MSP_RAW_GPS and len(p) >= 2: # Fix, satellites
                    state['gps'] = f"{'fix' if p[0] else 'no fix'}, {p[1]} sats"
        print(f"{time.monotonic() - t0:7.1f} s | modes {state.get('modes', '?')} | arming disable flags {state.get('flags', '?')} | "
              f"rc {state.get('rc', '?')} | alt {state.get('alt', '?')} | gps {state.get('gps', '?')}", flush=True)

if __name__ == '__main__':
    main()
