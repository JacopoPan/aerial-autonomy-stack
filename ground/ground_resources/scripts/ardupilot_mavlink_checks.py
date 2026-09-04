"""
Sanity-check the MAVLink link created by mavlink-router for QGC (udpin:127.0.0.1:14550)
Note: remember to close QGroundControl first

Use as:
    python3 ardupilot_mavlink_checks.py --device="udpin:127.0.0.1:14550"

If necessary, remove MAVLink signature over USB (e.g. `/dev/ttyACM0`)

python3 -m venv ~/.venvs/mavlink
source ~/.venvs/mavlink/bin/activate
pip install pymavlink pyserial

python3 - <<'EOF'
import os; os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200, source_system=250)
m.wait_heartbeat()
print('system', m.target_system)
m.mav.setup_signing_send(m.target_system, m.target_component or 1, [0]*32, 0)
print('erase sent - power-cycle, then re-run the checks')
EOF
"""
import argparse
import os
import sys
import time
from collections import defaultdict

os.environ.setdefault("MAVLINK20", "1")  # MUST precede the pymavlink import
from pymavlink import mavutil

SIGNED = 0x01  # MAVLINK_IFLAG_SIGNED

def listen(m, seconds):
    # Tabulate traffic per (system, component)
    st = defaultdict(lambda: {"signed": 0, "unsigned": 0, "lost": 0, "last_seq": None, "types": defaultdict(int)})
    end = time.time() + seconds
    while time.time() < end:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        h = msg.get_header()
        s = st[(h.srcSystem, h.srcComponent)]
        s["signed" if h.incompat_flags & SIGNED else "unsigned"] += 1
        s["types"][msg.get_type()] += 1
        # Sequence loss must be tracked per source. mavlink-router uses one counter per endpoint, which reports nonsense with two vehicles
        if s["last_seq"] is not None:
            gap = (h.seq - s["last_seq"] - 1) % 256
            s["lost"] += gap
        s["last_seq"] = h.seq
    return st

def probe(m, sysid, compid, timeout=5):
    # One round trip: long command MAV_CMD_DO_SEND_BANNER + SYSID_THISMAV parameter read
    m.mav.command_long_send(sysid, compid, mavutil.mavlink.MAV_CMD_DO_SEND_BANNER, 0, 0, 0, 0, 0, 0, 0, 0)
    m.mav.param_request_read_send(sysid, compid, b"SYSID_THISMAV", -1)
    ack, uid, param = None, None, None
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT", "PARAM_VALUE"], blocking=True, timeout=1)
        if msg is None or msg.get_srcSystem() != sysid:
            continue
        t = msg.get_type()
        if t == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_DO_SEND_BANNER:
            ack = msg.result
        elif t == "STATUSTEXT" and "Pixhawk" in msg.text:
            uid = msg.text
        elif t == "PARAM_VALUE" and msg.param_id.strip("\x00") == "SYSID_THISMAV":
            param = int(msg.param_value)
    return ack, uid, param

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="udpin:127.0.0.1:14550")
    ap.add_argument("--listen_for", type=int, default=10, help="passive seconds")
    ap.add_argument("--source-system", type=int, default=250, help="not 255: that is SYSID_MYGCS and would fight QGC")
    args = ap.parse_args()

    print(f"Listening on {args.device} for {args.listen_for}s")
    m = mavutil.mavlink_connection(args.device, source_system=args.source_system)
    if m.wait_heartbeat(timeout=10) is None:
        print("FAIL: no heartbeat. Is mavlink-router up? Is QGC holding the port? Remember to close QGC")
        return 2

    st = listen(m, args.listen_for)
    autopilots = sorted({s for (s, c) in st if c == 1})

    print(f"\n{'sys.comp':<10} {'msgs':>7} {'signed':>7} {'lost':>6} {'loss%':>6}")
    for (s, c) in sorted(st):
        d = st[(s, c)]
        got = d["signed"] + d["unsigned"]
        total = got + d["lost"]
        pct = 100.0 * d["lost"] / total if got >= 100 else None
        shown = f"{pct:5.1f}%" if pct is not None else "    --"
        lost = f"{d['lost']}" if pct is not None else "--"
        print(f"{s}.{c:<8} {got:>7} {d['signed']:>7} {lost:>6} {shown:>6}")

    print("\nProbing each autopilot (MAV_CMD_DO_SEND_BANNER + SYSID_THISMAV)")
    problems = []
    for s in autopilots:
        ack, uid, param = probe(m, s, 1)
        d = st[(s, 1)]
        signing = d["signed"] > 0
        replied = ack == 0 or uid is not None or param is not None
        note = "  (reply seen, but no ack)" if replied and ack != 0 else ""
        print(f"\n  system {s}")
        print(f"    signing         : {'ACTIVE' if signing else 'off'}")
        print(f"    uplink          : {'ok' if replied else 'NO RESPONSE'}{note}")
        print(f"    param served    : {param if param is not None else 'NO RESPONSE'}")
        print(f"    board           : {uid or 'unknown'}")
        if signing:
            problems.append(f"system {s}: MAVLink2 signing is active; telemetry ports will ignore all commands")
        if not replied:
            problems.append(f"system {s}: no reply to banner or param request, uplink is not working")
        if param is not None and param != s:
            problems.append(f"system {s}: SYSID_THISMAV is {param}; aircraft container launches mavros with tgt_system=DRONE_ID")

    if not autopilots:
        problems.append("no autopilot (component 1) on the link")

    print("\nSummary")
    if problems:
        for p in problems:
            print(f"  FAIL  {p}")
        return 1
    print(f"  OK  {len(autopilots)} autopilot(s): {autopilots}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
