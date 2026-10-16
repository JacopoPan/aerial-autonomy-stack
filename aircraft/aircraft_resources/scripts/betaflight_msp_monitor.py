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

MSP_RC = 105
MSP_RAW_GPS = 106
MSP_ALTITUDE = 109
MSP_BOXIDS = 119
MSP_STATUS_EX = 150
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
    start = buffer.find(b'$M>')
    while start >= 0:
        buffer = buffer[start:]
        if len(buffer) < 6 or len(buffer) < 6 + buffer[3]: # Incomplete
            return replies, buffer
        size, cmd = buffer[3], buffer[4]
        replies.append((cmd, buffer[5:5 + size]))
        buffer = buffer[6 + size:]
        start = buffer.find(b'$M>')
    return replies, buffer[-2:] # Keep a split header

def bit_names(value, names): # The names of the bits set in value, '-' if none
    set_names = [name for bit, name in enumerate(names) if (value >> bit) & 1]
    return '+'.join(set_names) if set_names else '-'

def main():
    parser = argparse.ArgumentParser(description='Print the Betaflight SITL state from MSP on UART1')
    parser.add_argument('--ip', required=True, help='Betaflight SITL address')
    parser.add_argument('--port', type=int, required=True, help='UART1 port (5761 + port offset)')
    parser.add_argument('--period', type=float, default=0.5, help='Print period in s (wall clock)')
    args = parser.parse_args()

    sock = socket.create_connection((args.ip, args.port), timeout=5.0)
    print(f'MSP on {args.ip}:{args.port}', flush=True)
    box_names = [] # Of the active modes bits, from MSP_BOXIDS
    buffer = b''
    modes = flags = rc = alt = gps = '?' # Until their replies arrive
    t0 = time.monotonic()
    request(sock, MSP_BOXIDS) # Once, to name the bits of the active modes
    while True:
        for cmd in (MSP_STATUS_EX, MSP_RC, MSP_ALTITUDE, MSP_RAW_GPS):
            request(sock, cmd)
        deadline = time.monotonic() + args.period
        left = args.period
        while left > 0: # Replies until the next requests
            sock.settimeout(left)
            try:
                data = sock.recv(4096)
            except socket.timeout:
                break
            if not data:
                raise SystemExit('MSP link closed')
            replies, buffer = split_replies(buffer + data)
            for cmd, payload in replies:
                if cmd == MSP_BOXIDS:
                    box_names = [BOX_NAMES.get(box_id, str(box_id)) for box_id in payload]
                elif cmd == MSP_STATUS_EX and len(payload) >= 16 and len(payload) >= 21 + payload[15]: # Active modes (32 bits, then payload[15] more bytes), arming disable flags
                    extra = payload[15]
                    active_modes = struct.unpack_from('<I', payload, 6)[0] | (int.from_bytes(payload[16:16 + extra], 'little') << 32)
                    modes = bit_names(active_modes, box_names)
                    flags = bit_names(struct.unpack_from('<I', payload, 17 + extra)[0], ARMING_DISABLE_FLAGS)
                elif cmd == MSP_RC and len(payload) >= 16: # Roll, pitch, yaw, throttle, AUX1-4
                    v = struct.unpack_from('<8H', payload)
                    rc = f'roll {v[0]} pitch {v[1]} yaw {v[2]} throttle {v[3]} aux {v[4]} {v[5]} {v[6]} {v[7]}'
                elif cmd == MSP_ALTITUDE and len(payload) >= 6: # Altitude in cm, vertical speed in cm/s
                    altitude, vertical_speed = struct.unpack_from('<ih', payload)
                    alt = f'{altitude / 100:.2f} m, {vertical_speed / 100:+.2f} m/s'
                elif cmd == MSP_RAW_GPS and len(payload) >= 2: # Fix, satellites
                    gps = f"{'fix' if payload[0] else 'no fix'}, {payload[1]} sats"
            left = deadline - time.monotonic()
        print(f'{time.monotonic() - t0:7.1f} s | modes {modes} | arming disable flags {flags} | rc {rc} | alt {alt} | gps {gps}', flush=True)

if __name__ == '__main__':
    main()
