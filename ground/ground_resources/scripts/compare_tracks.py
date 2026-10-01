"""
Live terminal comparison of /telemetry_tracks and /external_tracks, refreshed every second
When a drone's labels differ, a >> row compares the two (telemetry track <label t> vs external track <label e>)

Use as:
    python3 /aas/ground_resources/scripts/compare_tracks.py
    python3 /aas/ground_resources/scripts/compare_tracks.py --ros-args -p tolerance:=5.0
    python3 /aas/ground_resources/scripts/compare_tracks.py --ros-args -p use_sim_time:=true
"""
import math

import rclpy
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geographiclib.geodesic import Geodesic

from ground_system_msgs.msg import SwarmObs

def errors(a, b): 
    # Horizontal (WGS84 geodesic), vertical and velocity differences between two tracks
    h = Geodesic.WGS84.Inverse(a.latitude_deg, a.longitude_deg, b.latitude_deg, b.longitude_deg)['s12']
    v = abs(b.altitude_m - a.altitude_m)
    vel = math.hypot(b.velocity_n_m_s - a.velocity_n_m_s, b.velocity_e_m_s - a.velocity_e_m_s, b.velocity_d_m_s - a.velocity_d_m_s)
    return h, v, vel

class CompareTracks(Node):
    def __init__(self):
        super().__init__('compare_tracks')
        self.tolerance = self.declare_parameter('tolerance', 1.0).value # m: max horizontal and vertical error for OK
        self.msgs = {'telemetry': SwarmObs(), 'external': SwarmObs()} # Latest message per source (empty until received)
        self.rx = {'telemetry': None, 'external': None} # Receive time per source (s, node clock)

        self.create_subscription(SwarmObs, '/telemetry_tracks', lambda msg: self.store('telemetry', msg), qos_profile_sensor_data)
        self.create_subscription(SwarmObs, '/external_tracks', lambda msg: self.store('external', msg), qos_profile_sensor_data)
        self.create_timer(1.0, self.show, clock=Clock())  # Redraw every wall-clock second

    def store(self, source, msg):
        self.msgs[source] = msg
        self.rx[source] = self.get_clock().now().nanoseconds * 1e-9

    def show(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        age = {k: 'none yet' if t is None else f'{now - t:.1f} s ago' for k, t in self.rx.items()}
        tel = {t.id: t for t in self.msgs['telemetry'].tracks}
        ext = {t.id: t for t in self.msgs['external'].tracks}
        lines = [f"last msg on /telemetry_tracks: {age['telemetry']} | on /external_tracks: {age['external']} | OK = horiz and vert err <= {self.tolerance} m",
                 f'{"id":>4}  {"label t/e":>10}  {"horiz_m":>8}  {"vert_m":>7}  {"vel_m/s":>8}  status']
        for i in sorted(tel.keys() | ext.keys()):
            a, b = tel.get(i), ext.get(i)
            if a is None or b is None:
                lines.append(f'{i:>4}  {"external only" if a is None else "telemetry only"}')
                continue
            h, v, vel = errors(a, b)
            ok = h <= self.tolerance and v <= self.tolerance
            lines.append(f'{i:>4}  {f"{a.label}/{b.label}":>10}  {h:>8.2f}  {v:>7.2f}  {vel:>8.2f}  {"OK" if ok else "DIFF"}')
            if a.label != b.label: # Labels differ: compare the two
                ta, tb = tel.get(a.label), ext.get(b.label)
                if ta is None or tb is None:
                    lines.append(f'{">>":>4}  {f"{a.label} v. {b.label}":>10} label ID missing in {"telemetry" if ta is None else "external"}')
                else:
                    h, v, vel = errors(ta, tb)
                    ok = h <= self.tolerance and v <= self.tolerance
                    lines.append(f'{">>":>4}  {f"{a.label} v. {b.label}":>10}  {h:>8.2f}  {v:>7.2f}  {vel:>8.2f}  {"OK" if ok else "DIFF"}')
        print('\033[H\033[J' + '\n'.join(lines), flush=True) # Clear the terminal, then print: the table refreshes in place

def main():
    rclpy.init()
    try:
        rclpy.spin(CompareTracks())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass # Ctrl-C: exit without a traceback

if __name__ == '__main__':
    main()
