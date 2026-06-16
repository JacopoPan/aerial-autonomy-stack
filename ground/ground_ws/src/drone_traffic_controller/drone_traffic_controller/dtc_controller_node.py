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
        self.pub = self.create_publisher(String, '/dtc_commands', 10)
        self.sub = self.create_subscription(SwarmObs, '/tracks', self.track_cb, 10)

        nq = int(os.environ.get('num_quads', os.environ.get('NUM_QUADS', '1')))
        nv = int(os.environ.get('num_vtols', os.environ.get('NUM_VTOLS', '0')))
        self.expected_ids = list(range(1, nq + nv + 1))

        self.drones = {i: {'home': None, 'curr': None, 'alt': 0.0} for i in self.expected_ids}
        self.state = 'WAIT_HOMES'
        self.target_takeoff_alt = 40.0

        self.timer = self.create_timer(1.0, self.loop)
        self.get_logger().info(f"DTC Active. Waiting for tracks from drones: {self.expected_ids}")

    def track_cb(self, msg):
        for t in msg.tracks:
            did = t.id
            if did in self.drones:
                self.drones[did]['curr'] = (t.latitude_deg, t.longitude_deg)
                self.drones[did]['alt'] = t.altitude_m
                if self.drones[did]['home'] is None:
                    self.drones[did]['home'] = (t.latitude_deg, t.longitude_deg, t.altitude_m)
                    self.get_logger().info(f"Drone {did} home set.")

    def send_cmd(self, drone_id, action, **kwargs):
        payload = {"drone_id": drone_id, "action": action}
        payload.update(kwargs)
        self.pub.publish(String(data=json.dumps(payload)))

    def loop(self):
        if self.state == 'WAIT_HOMES':
            if all(d['home'] is not None for d in self.drones.values()):
                self.get_logger().info("All homes acquired. Commanding takeoff.")
                for did in self.expected_ids:
                    self.send_cmd(did, 'takeoff', alt=self.target_takeoff_alt)
                self.state = 'WAIT_TAKEOFF'

        elif self.state == 'WAIT_TAKEOFF':
            # Check if all drones have climbed to within 2 meters of the target takeoff altitude
            if all(d['alt'] >= (d['home'][2] + self.target_takeoff_alt - 2.0) for d in self.drones.values()):
                self.get_logger().info("Takeoffs complete. Commanding Reposition formation.")
                self.execute_formation()
                self.state = 'IDLE' 

    def execute_formation(self):
        ref_lat, ref_lon, _ = self.drones[self.expected_ids[0]]['home']

        for i, did in enumerate(self.expected_ids):
            h_lat, h_lon, _ = self.drones[did]['home']
            home_e, home_n = gps_to_enu(h_lat, h_lon, ref_lat, ref_lon)

            # Global formation target
            target_global_e = i * 30.0 
            target_global_n = 50.0

            # Local command
            cmd_e = target_global_e - home_e
            cmd_n = target_global_n - home_n

            self.get_logger().info(f"Drone {did} Reposition target: East {cmd_e:.1f}, North {cmd_n:.1f}")
            self.send_cmd(did, 'reposition', east=cmd_e, north=cmd_n, alt=60.0)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DTCController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
