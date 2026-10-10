"""Fly a short Joy sequence and check the state that betaflight_interface publishes (Betaflight SITL)

Arm and hand CH1-4 to betaflight_interface in the betaflight_rc.py pane, with the drone on the ground:
    rc> arm
    rc> ch 7 2000

Then, from any pane of the same aircraft container:
    python3 /aas/aircraft_resources/scripts/betaflight_sitl_check.py --ros-args -p use_sim_time:=true

Checks gravity and vertical speed on the ground, climb against GPS altitude, the heading against the spawn heading,
yaw direction (gyro and heading), and horizontal velocity and GPS displacement against the heading
When the sequence ends, Betaflight returns to the betaflight_rc.py sticks (throttle low) within a second
"""
import argparse
import math
import os
import sys
from itertools import pairwise

# Phase, duration in s, Joy axes (roll, pitch, throttle, yaw sticks in [-1, 1], ANGLE mode)
PHASES = [
    ('ground', 2.0, [0.0, 0.0, -1.0, 0.0]),
    ('climb', 3.0, [0.0, 0.0, 0.3, 0.0]),
    ('hover', 3.0, [0.0, 0.0, 0.19, 0.0]),
    ('yaw_right', 3.0, [0.0, 0.0, 0.19, 0.2]),
    ('forward', 3.0, [0.0, 0.2, 0.19, 0.0]), # After a single yaw, so a mirrored heading cannot match the GPS
    ('stop', 2.0, [0.0, -0.2, 0.19, 0.0]),
    ('yaw_left', 3.0, [0.0, 0.0, 0.19, -0.2]),
    ('descend', 5.0, [0.0, 0.0, 0.1, 0.0]),
]


def angle_diff(a, b): # Degrees, in [-180, 180)
    return (a - b + 180.0) % 360.0 - 180.0


def evaluate(samples, expected_heading): # samples: (t, phase, state); returns (name, value, ok, expected) rows
    rows = {name: [(t, s) for t, n, s in samples if n == name] for name, _, _ in PHASES}
    if any(len(r) < 2 for r in rows.values()):
        return [('samples in every phase', min(len(r) for r in rows.values()), False, '>= 2')]
    def mean(r, k):
        return sum(s[k] for _, s in r) / len(r)
    def turned(r): # Heading change, clockwise positive
        return sum(angle_diff(b['heading'], a['heading']) for (_, a), (_, b) in pairwise(r))
    def gyro_turned(r): # Integral of the FLU gyro z, clockwise positive
        return sum(-math.degrees(a['gyro_z']) * (tb - ta) for (ta, a), (tb, _) in pairwise(r))
    g, c, f = rows['ground'], rows['climb'], rows['forward']
    up = c + rows['hover'] # Ends after the climb slowed down, where the ~0.2 s lag of the estimated vertical speed matters little
    climbed = sum(a['vu'] * (tb - ta) for (ta, a), (tb, _) in pairwise(up))
    gps_climbed = up[-1][1]['alt'] - up[0][1]['alt']
    course = math.degrees(math.atan2(f[-1][1]['ve'], f[-1][1]['vn'])) % 360.0
    d_north = (f[-1][1]['lat'] - f[0][1]['lat']) * 111320.0
    d_east = (f[-1][1]['lon'] - f[0][1]['lon']) * 111320.0 * math.cos(math.radians(f[0][1]['lat']))
    bearing = math.degrees(math.atan2(d_east, d_north)) % 360.0
    heading = f[-1][1]['heading']
    checks = [
        ('ground: GPS fix', g[-1][1]['fix'], g[-1][1]['fix'], 'True'),
        ('ground: accelerometer z (m/s^2)', mean(g, 'acc_z'), 9.3 < mean(g, 'acc_z') < 10.3, '9.81'),
        ('ground: vertical speed (m/s)', mean(g, 'vu'), abs(mean(g, 'vu')) < 0.3, '0'),
        ('ground: heading (deg)', g[-1][1]['heading'], abs(angle_diff(g[-1][1]['heading'], expected_heading)) < 10, f'{expected_heading:g}'),
        ('climb: vertical speed at the end (m/s)', c[-1][1]['vu'], c[-1][1]['vu'] > 0.5, '> 0.5'),
        ('climb + hover: GPS altitude gain (m)', gps_climbed, gps_climbed >= 1 and abs(gps_climbed - climbed) < 1.5, f'{climbed:.1f} (from vertical speed)'),
    ]
    for name, sign in (('yaw_right', 1), ('yaw_left', -1)):
        r = rows[name]
        checks.append((f'{name}: heading change (deg)', turned(r), sign * turned(r) > 20, f'{"> 20" if sign > 0 else "< -20"} (clockwise positive)'))
        checks.append((f'{name}: gyro z integral (deg)', gyro_turned(r), sign * gyro_turned(r) > 0 and abs(gyro_turned(r) - turned(r)) < 0.3 * abs(turned(r)), f'{turned(r):.0f} (heading change)'))
    checks.append(('forward: velocity course (deg)', course, abs(angle_diff(course, heading)) < 30, f'{heading:.0f} (heading)'))
    checks.append(('forward: GPS displacement bearing (deg)', bearing, math.hypot(d_north, d_east) > 1 and abs(angle_diff(bearing, heading)) < 30, f'{heading:.0f} (heading)'))
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--drone_id', type=int, default=int(os.environ.get('DRONE_ID', 1)))
    parser.add_argument('--expected_heading', type=float, default=90.0, help='Heading on the ground in deg (default 90: Gazebo spawns the drones facing East)')
    args, ros_args = parser.parse_known_args()

    import rclpy
    from geometry_msgs.msg import TwistStamped, Vector3Stamped
    from sensor_msgs.msg import Imu, Joy, NavSatFix

    rclpy.init(args=ros_args)
    node = rclpy.create_node('betaflight_sitl_check')
    ns, latest, samples, state = f'/Drone{args.drone_id}', {}, [], {'t0': None, 'done': False}
    joy_pub = node.create_publisher(Joy, f'{ns}/rc_override', 10)
    node.create_subscription(Imu, f'{ns}/imu', lambda m: latest.update(acc_z=m.linear_acceleration.z, gyro_z=m.angular_velocity.z), 10)
    node.create_subscription(Vector3Stamped, f'{ns}/attitude', lambda m: latest.update(heading=math.degrees(m.vector.z)), 10)
    node.create_subscription(TwistStamped, f'{ns}/velocity', lambda m: latest.update(ve=m.twist.linear.x, vn=m.twist.linear.y, vu=m.twist.linear.z), 10)
    node.create_subscription(NavSatFix, f'{ns}/global_position', lambda m: latest.update(lat=m.latitude, lon=m.longitude, alt=m.altitude, fix=m.status.status >= 0), 10)

    def step(): # 20 Hz on the node clock: send the Joy of the current phase and record the latest state
        t = node.get_clock().now().nanoseconds / 1e9
        if t == 0.0 or len(latest) < 10: # Wait for /clock and every topic
            return
        state['t0'] = state['t0'] or t
        end = 0.0
        for name, duration, axes in PHASES:
            end += duration
            if t - state['t0'] < end:
                joy_pub.publish(Joy(axes=axes))
                samples.append((t, name, dict(latest)))
                print(f'\r{name:10s} {t - state["t0"]:5.1f} s', end='', flush=True)
                return
        state['done'] = True

    node.create_timer(0.05, step)
    print(f'Waiting for /clock and {ns}/imu, attitude, velocity, global_position', flush=True)
    while rclpy.ok() and not state['done']:
        rclpy.spin_once(node, timeout_sec=0.1)
    print()
    checks = evaluate(samples, args.expected_heading)
    print(f"{'CHECK':42s} {'VALUE':>9s}  {'EXPECTED':32s} RESULT")
    for name, value, ok, expected in checks:
        print(f"{name:42s} {value:9.2f}  {expected:32s} {'PASS' if ok else 'FAIL'}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(ok for _, _, ok, _ in checks) else 1)


if __name__ == '__main__':
    main()
