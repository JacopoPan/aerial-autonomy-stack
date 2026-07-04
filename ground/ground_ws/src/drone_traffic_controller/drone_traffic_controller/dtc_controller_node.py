import os, json, math, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ground_system_msgs.msg import SwarmObs
from geographiclib.geodesic import Geodesic

def gps_to_enu(lat, lon, lat_ref, lon_ref):
    geo = Geodesic.WGS84.Inverse(lat_ref, lon_ref, lat, lon)
    distance = geo['s12']
    azimuth = math.radians(geo['azi1'])
    east = distance * math.sin(azimuth)
    north = distance * math.cos(azimuth)
    return east, north

class DTCController(Node):
    def __init__(self):
        super().__init__('dtc_controller')

        # Mission Parameters
        self.quad_to_alt = 40.0
        self.vtol_to_alt = 40.0
        self.tail_to_alt = 40.0
        self.poly_alt = 90.0
        self.poly_radius = 40.0
        self.poly_center_e = 30.0
        self.poly_center_n = 150.0
        self.quad_step = 0
        self.loops_completed = 0
        self.enforcement_timer = 0

        self.pub = self.create_publisher(String, '/dtc_commands', 10)
        self.sub = self.create_subscription(SwarmObs, '/tracks', self.track_cb, 10)
        self.timer = self.create_timer(1.0, self.loop)

        self.nq = int(os.environ.get('num_quads', os.environ.get('NUM_QUADS', '1')))
        self.nv = int(os.environ.get('num_vtols', os.environ.get('NUM_VTOLS', '0')))
        self.nt = int(os.environ.get('num_tails', os.environ.get('NUM_TAILS', '0')))
        self.quad_ids = list(range(1, self.nq + 1))
        self.vtol_ids = list(range(self.nq + 1, self.nq + self.nv + 1))
        self.tail_ids = list(range(self.nq + self.nv + 1, self.nq + self.nv + self.nt + 1))
        self.expected_ids = self.quad_ids + self.vtol_ids + self.tail_ids
        self.drones = {i: {'home': None, 'curr': None, 'alt': 0.0, 'target_enu': None} for i in self.expected_ids}
        self.state = 'WAIT_HOMES'

        self.get_logger().info(f"DTC Active. Waiting for tracks from drones: {self.expected_ids}")

    def track_cb(self, msg):
        for t in msg.tracks:
            did = t.id
            if did in self.drones:
                self.drones[did]['curr'] = (t.latitude_deg, t.longitude_deg)
                self.drones[did]['alt'] = t.altitude_m

    def send_cmd(self, drone_id, action, **kwargs):
        payload = {"drone_id": drone_id, "action": action}
        payload.update(kwargs)
        self.pub.publish(String(data=json.dumps(payload)))

    def loop(self):
        if self.state == 'WAIT_HOMES':
            # Check if we have current tracking data for every expected drone
            if all(d['curr'] is not None for d in self.drones.values()):
                self.get_logger().info("All drones online. Locking homes and commanding takeoffs.")
                # Lock in the home position for all drones just before takeoff
                for did, d in self.drones.items():
                    d['home'] = (d['curr'][0], d['curr'][1], d['alt'])
                for did in self.quad_ids:
                    self.send_cmd(did, 'takeoff', alt=self.quad_to_alt)
                for did in self.vtol_ids:
                    self.send_cmd(did, 'takeoff', alt=self.vtol_to_alt)
                for did in self.tail_ids:
                    self.send_cmd(did, 'takeoff', alt=self.tail_to_alt)
                self.state = 'WAIT_TAKEOFF'

        elif self.state == 'WAIT_TAKEOFF':
            quads_ready = all(d['alt'] >= (d['home'][2] + self.quad_to_alt - 2.0) for d_id, d in self.drones.items() if d_id in self.quad_ids)
            vtols_ready = all(d['alt'] >= (d['home'][2] + self.vtol_to_alt - 2.0) for d_id, d in self.drones.items() if d_id in self.vtol_ids)
            tails_ready = all(d['alt'] >= (d['home'][2] + self.tail_to_alt - 2.0) for d_id, d in self.drones.items() if d_id in self.tail_ids)
            if quads_ready and (vtols_ready or self.nv == 0) and (tails_ready or self.nt == 0):
                self.get_logger().info("Takeoffs complete. Starting formation.")
                self.command_vtol_orbits()
                self.command_tail_orbits()
                self.command_quad_polygon()
                self.state = 'ENFORCE_POLYGON'

        elif self.state == 'ENFORCE_POLYGON':
            ref_lat, ref_lon, _ = self.drones[self.expected_ids[0]]['home']
            all_quads_arrived = True
            for did in self.quad_ids:
                if self.drones[did]['curr'] is None or self.drones[did]['target_enu'] is None:
                    all_quads_arrived = False
                    continue
                # Compare current ENU to target ENU
                c_lat, c_lon = self.drones[did]['curr']
                curr_e, curr_n = gps_to_enu(c_lat, c_lon, ref_lat, ref_lon)
                tgt_e, tgt_n = self.drones[did]['target_enu']
                dist_2d = math.hypot(curr_e - tgt_e, curr_n - tgt_n)
                alt_diff = abs(self.drones[did]['alt'] - (self.drones[did]['home'][2] + self.poly_alt))
                if dist_2d > 3.0 or alt_diff > 3.0: # HARDCODED: 3 meters tolerances
                    all_quads_arrived = False
            if all_quads_arrived:
                self.quad_step += 1
                self.enforcement_timer = 0 # Reset timer on success
                # A full loop is completed based on the number of vertices (minimum 3 for a triangle)
                if self.quad_step > 0 and self.quad_step % max(3, self.nq) == 0:
                    self.loops_completed += 1
                    self.get_logger().info(f"Loop {self.loops_completed} completed!")
                if self.loops_completed >= 2:
                    self.get_logger().info("Mission accomplished. Commanding Land.")
                    for did in self.expected_ids:
                        self.send_cmd(did, 'land')
                    self.state = 'IDLE'
                else:
                    self.get_logger().info("Waypoints reached. Shifting CCW.")
                    self.command_quad_polygon()
            else:
                self.enforcement_timer += 1
                if self.enforcement_timer >= 10: # Re-broadcast targets every 10 seconds if drones haven't arrived
                    self.get_logger().info("Still waiting for arrival. Re-broadcasting polygon targets.")
                    self.command_quad_polygon()
                    self.enforcement_timer = 0

    def command_vtol_orbits(self):
        ref_lat, ref_lon, _ = self.drones[self.expected_ids[0]]['home']
        for did in self.vtol_ids:
            h_lat, h_lon, _ = self.drones[did]['home']
            home_e, home_n = gps_to_enu(h_lat, h_lon, ref_lat, ref_lon)
            # Send orbit command, circumscribing the poly radius, 10m higher, 20m wider
            self.send_cmd(did, 'orbit', east=self.poly_center_e - home_e, north=self.poly_center_n - home_n, alt=self.poly_alt + 10.0, radius=self.poly_radius + 20.0)

    def command_tail_orbits(self):
        ref_lat, ref_lon, _ = self.drones[self.expected_ids[0]]['home']
        for did in self.tail_ids:
            h_lat, h_lon, _ = self.drones[did]['home']
            home_e, home_n = gps_to_enu(h_lat, h_lon, ref_lat, ref_lon)
            # Send orbit command, circumscribing the poly radius, 20m higher, 30m wider
            self.send_cmd(did, 'orbit', east=self.poly_center_e - home_e, north=self.poly_center_n - home_n, alt=self.poly_alt + 20.0, radius=self.poly_radius + 30.0)

    def command_quad_polygon(self):
        ref_lat, ref_lon, _ = self.drones[self.expected_ids[0]]['home']
        num_vertices = max(3, self.nq) # Guarantee at least a triangle
        for i, did in enumerate(self.quad_ids):
            # Calculate CCW vertex
            angle = 2 * math.pi * (i + self.quad_step) / num_vertices
            # Add the offset center to the target
            target_e = self.poly_center_e + self.poly_radius * math.cos(angle)
            target_n = self.poly_center_n + self.poly_radius * math.sin(angle)
            h_lat, h_lon, _ = self.drones[did]['home']
            home_e, home_n = gps_to_enu(h_lat, h_lon, ref_lat, ref_lon)
            self.drones[did]['target_enu'] = (target_e, target_n)
            self.send_cmd(did, 'reposition', east=target_e - home_e, north=target_n - home_n, alt=self.poly_alt)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DTCController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
