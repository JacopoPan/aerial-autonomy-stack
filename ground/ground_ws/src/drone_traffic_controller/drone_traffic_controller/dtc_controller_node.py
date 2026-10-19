
import os, json, math, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ground_system_msgs.msg import SwarmObs
from geographiclib.geodesic import Geodesic


# ============================================================
# 1. 좌표 변환
# ============================================================
# GPS(위도, 경도)를 기준 위치로부터의 상대 거리(m)로 변환
# ENU 좌표계: East(동쪽), North(북쪽), Up(위쪽)
# 편대 목표 위치와 현재 위치를 비교할 때 사용
# ============================================================

def gps_to_enu(lat, lon, lat_ref, lon_ref):
    geo = Geodesic.WGS84.Inverse(lat_ref, lon_ref, lat, lon)
    distance = geo['s12']
    azimuth = math.radians(geo['azi1'])

    east = distance * math.sin(azimuth)
    north = distance * math.cos(azimuth)

    return east, north


class DTCController(Node):

    # ========================================================
    # 2. 시스템 초기화
    # ========================================================
    # - 비행 실험에 필요한 매개변수 설정
    # - ROS2 통신 설정
    # - 드론 수 및 ID 설정
    # - 드론별 상태 저장 공간 생성
    # ========================================================

    def __init__(self):
        super().__init__('dtc_controller')

        # ----------------------------------------------------
        # 2-1. 비행 및 편대 설정
        # ----------------------------------------------------

        # 기체 종류별 초기 이륙 고도(m)
        self.quad_to_alt = 40.0
        self.vtol_to_alt = 40.0
        self.tail_to_alt = 40.0

        # 다각형 편대의 비행 고도와 크기
        self.poly_alt = 90.0
        self.poly_radius = 40.0

        # 다각형 중심 위치(기준 GPS 위치로부터의 상대 거리)
        self.poly_center_e = 30.0
        self.poly_center_n = 150.0

        # 편대 이동 진행 상태
        self.quad_step = 0           # 현재 이동 단계
        self.loops_completed = 0     # 완료한 다각형 순환 횟수
        self.enforcement_timer = 0   # 목표 위치 재전송을 위한 카운터

        # ----------------------------------------------------
        # 2-2. ROS2 통신 설정
        # ----------------------------------------------------

        # 각 드론에 비행 명령 전송
        self.pub = self.create_publisher(
            String, '/dtc_commands', 10
        )

        # 모든 드론의 위치 및 상태 정보 수신
        self.sub = self.create_subscription(
            SwarmObs, '/tracks', self.track_cb, 10
        )

        # 1초마다 비행 단계 관리 함수 실행
        self.timer = self.create_timer(1.0, self.loop)

        # ----------------------------------------------------
        # 2-3. 드론 수 및 ID 설정
        # ----------------------------------------------------

        # 환경변수에서 기체 종류별 수량 읽기
        self.nq = int(os.environ.get(
            'num_quads', os.environ.get('NUM_QUADS', '1')
        ))
        self.nv = int(os.environ.get(
            'num_vtols', os.environ.get('NUM_VTOLS', '0')
        ))
        self.nt = int(os.environ.get(
            'num_tails', os.environ.get('NUM_TAILS', '0')
        ))

        # 기체 종류별 ID 할당
        self.quad_ids = list(range(1, self.nq + 1))
        self.vtol_ids = list(range(
            self.nq + 1, self.nq + self.nv + 1
        ))
        self.tail_ids = list(range(
            self.nq + self.nv + 1,
            self.nq + self.nv + self.nt + 1
        ))

        self.expected_ids = (
            self.quad_ids + self.vtol_ids + self.tail_ids
        )

        # ----------------------------------------------------
        # 2-4. 드론 상태 및 임무 상태 초기화
        # ----------------------------------------------------

        # home       : 이륙 직전 기준 위치(GPS + 고도)
        # curr       : 현재 GPS 위치
        # alt        : 현재 고도
        # target_enu : 현재 목표 위치(동쪽, 북쪽)
        self.drones = {
            i: {
                'home': None,
                'curr': None,
                'alt': 0.0,
                'target_enu': None
            }
            for i in self.expected_ids
        }

        # 초기 상태: 모든 드론의 위치 정보 수신 대기
        self.state = 'WAIT_HOMES'

        self.get_logger().info(
            f"DTC Active. Waiting for tracks from drones: {self.expected_ids}"
        )

    # ========================================================
    # 3. 드론 상태 정보 수신
    # ========================================================
    # /tracks 토픽에서 각 드론의 현재 위치와 고도를 수신
    # 수신한 정보를 self.drones에 저장
    #
    # 이 정보는 이륙 완료 여부 및 목표 지점 도착 판단에 사용
    # ========================================================

    def track_cb(self, msg):
        for t in msg.tracks:
            did = t.id

            if did in self.drones:
                self.drones[did]['curr'] = (
                    t.latitude_deg,
                    t.longitude_deg
                )
                self.drones[did]['alt'] = t.altitude_m

    # ========================================================
    # 4. 드론 비행 명령 전송
    # ========================================================
    # 명령을 JSON 문자열로 만들어 /dtc_commands에 발행
    #
    # 주요 명령:
    # takeoff    : 이륙
    # reposition : 목표 위치로 이동
    # orbit      : 지정된 중심을 기준으로 선회
    # land       : 착륙
    # ========================================================

    def send_cmd(self, drone_id, action, **kwargs):
        payload = {
            "drone_id": drone_id,
            "action": action
        }

        payload.update(kwargs)

        self.pub.publish(
            String(data=json.dumps(payload))
        )

    # ========================================================
    # 5. 전체 비행 임무 관리
    # ========================================================
    # 1초마다 실행되며 현재 임무 상태에 따라 동작
    #
    # WAIT_HOMES
    #     ↓ 모든 드론 위치 확인
    # WAIT_TAKEOFF
    #     ↓ 모든 드론 이륙 완료
    # ENFORCE_POLYGON
    #     ↓ 다각형 꼭짓점 이동을 2회 순환
    # IDLE
    #
    # 현재 코드는 충돌 회피 없이 목표 위치 도착 여부만 판단
    # ========================================================

    def loop(self):

        # ----------------------------------------------------
        # 5-1. 드론 연결 확인 및 이륙 명령
        # ----------------------------------------------------

        if self.state == 'WAIT_HOMES':

            # 모든 드론에서 위치 정보가 수신되었는지 확인
            if all(
                d['curr'] is not None
                for d in self.drones.values()
            ):
                self.get_logger().info(
                    "All drones online. Locking homes and commanding takeoffs."
                )

                # 이륙 직전 위치를 각 드론의 기준 위치로 저장
                for _did, d in self.drones.items():
                    d['home'] = (
                        d['curr'][0],
                        d['curr'][1],
                        d['alt']
                    )

                # 기체 종류별 이륙 명령
                for did in self.quad_ids:
                    self.send_cmd(
                        did, 'takeoff', alt=self.quad_to_alt
                    )

                for did in self.vtol_ids:
                    self.send_cmd(
                        did, 'takeoff', alt=self.vtol_to_alt
                    )

                for did in self.tail_ids:
                    self.send_cmd(
                        did, 'takeoff', alt=self.tail_to_alt
                    )

                self.state = 'WAIT_TAKEOFF'

        # ----------------------------------------------------
        # 5-2. 이륙 완료 확인 및 편대비행 시작
        # ----------------------------------------------------

        elif self.state == 'WAIT_TAKEOFF':

            # 기체별 현재 고도가 목표 이륙 고도에 도달했는지 확인
            # 목표 고도보다 최대 2m 낮아도 도착한 것으로 판단
            quads_ready = all(
                d['alt'] >= (
                    d['home'][2] + self.quad_to_alt - 2.0
                )
                for d_id, d in self.drones.items()
                if d_id in self.quad_ids
            )

            vtols_ready = all(
                d['alt'] >= (
                    d['home'][2] + self.vtol_to_alt - 2.0
                )
                for d_id, d in self.drones.items()
                if d_id in self.vtol_ids
            )

            tails_ready = all(
                d['alt'] >= (
                    d['home'][2] + self.tail_to_alt - 2.0
                )
                for d_id, d in self.drones.items()
                if d_id in self.tail_ids
            )

            # 모든 기체의 이륙이 완료되면 편대 목표 위치 전송
            if (
                quads_ready
                and (vtols_ready or self.nv == 0)
                and (tails_ready or self.nt == 0)
            ):
                self.get_logger().info(
                    "Takeoffs complete. Starting formation."
                )

                self.command_vtol_orbits()
                self.command_tail_orbits()
                self.command_quad_polygon()

                self.state = 'ENFORCE_POLYGON'

        # ----------------------------------------------------
        # 5-3. 다각형 편대 이동 및 목표 지점 도착 확인
        # ----------------------------------------------------

        elif self.state == 'ENFORCE_POLYGON':

            # 첫 번째 드론의 홈 위치를 공통 좌표 기준으로 사용
            ref_lat, ref_lon, _ = (
                self.drones[self.expected_ids[0]]['home']
            )

            all_quads_arrived = True

            # 모든 쿼드로터가 자신의 목표 꼭짓점에 도착했는지 확인
            for did in self.quad_ids:

                if (
                    self.drones[did]['curr'] is None
                    or self.drones[did]['target_enu'] is None
                ):
                    all_quads_arrived = False
                    continue

                # 현재 GPS 위치를 ENU 좌표로 변환
                c_lat, c_lon = self.drones[did]['curr']
                curr_e, curr_n = gps_to_enu(
                    c_lat, c_lon, ref_lat, ref_lon
                )

                # 현재 목표 위치
                tgt_e, tgt_n = (
                    self.drones[did]['target_enu']
                )

                # 수평 거리 오차와 고도 오차 계산
                dist_2d = math.hypot(
                    curr_e - tgt_e,
                    curr_n - tgt_n
                )

                alt_diff = abs(
                    self.drones[did]['alt']
                    - (
                        self.drones[did]['home'][2]
                        + self.poly_alt
                    )
                )

                # 수평 또는 고도 오차가 3m 초과이면 미도착
                if dist_2d > 3.0 or alt_diff > 3.0:
                    all_quads_arrived = False

            # ------------------------------------------------
            # 모든 쿼드로터가 목표 지점에 도착한 경우
            # ------------------------------------------------

            if all_quads_arrived:

                # 다음 다각형 꼭짓점으로 이동하기 위한 단계 증가
                self.quad_step += 1
                self.enforcement_timer = 0

                # 모든 꼭짓점 이동을 완료하면 1회 순환으로 계산
                if (
                    self.quad_step > 0
                    and self.quad_step % max(3, self.nq) == 0
                ):
                    self.loops_completed += 1

                    self.get_logger().info(
                        f"Loop {self.loops_completed} completed!"
                    )

                # 2회 순환 완료 시 전체 드론 착륙
                if self.loops_completed >= 2:

                    self.get_logger().info(
                        "Mission accomplished. Commanding Land."
                    )

                    for did in self.expected_ids:
                        self.send_cmd(did, 'land')

                    self.state = 'IDLE'

                # 아직 임무 진행 중이면 다음 꼭짓점 목표 전송
                else:
                    self.get_logger().info(
                        "Waypoints reached. Shifting CCW."
                    )

                    self.command_quad_polygon()

            # ------------------------------------------------
            # 일부 쿼드로터가 목표 지점에 도착하지 못한 경우
            # ------------------------------------------------

            else:
                self.enforcement_timer += 1

                # 10초 동안 목표에 도착하지 못하면 명령 재전송
                if self.enforcement_timer >= 10:

                    self.get_logger().info(
                        "Still waiting for arrival. Re-broadcasting polygon targets."
                    )

                    self.command_quad_polygon()
                    self.enforcement_timer = 0

    # ========================================================
    # 6. VTOL 기체 선회 비행
    # ========================================================
    # 쿼드로터 편대 중심을 기준으로 원형 선회 명령
    # - 쿼드로터 편대보다 고도 10m 높게 설정
    # - 쿼드로터 편대보다 반지름 20m 크게 설정
    # ========================================================

    def command_vtol_orbits(self):

        ref_lat, ref_lon, _ = (
            self.drones[self.expected_ids[0]]['home']
        )

        for did in self.vtol_ids:

            h_lat, h_lon, _ = self.drones[did]['home']

            home_e, home_n = gps_to_enu(
                h_lat, h_lon, ref_lat, ref_lon
            )

            self.send_cmd(
                did,
                'orbit',
                east=self.poly_center_e - home_e,
                north=self.poly_center_n - home_n,
                alt=self.poly_alt + 10.0,
                radius=self.poly_radius + 20.0
            )

    # ========================================================
    # 7. 테일시터 기체 선회 비행
    # ========================================================
    # 쿼드로터 편대 중심을 기준으로 원형 선회 명령
    # - 쿼드로터 편대보다 고도 20m 높게 설정
    # - 쿼드로터 편대보다 반지름 30m 크게 설정
    # ========================================================

    def command_tail_orbits(self):

        ref_lat, ref_lon, _ = (
            self.drones[self.expected_ids[0]]['home']
        )

        for did in self.tail_ids:

            h_lat, h_lon, _ = self.drones[did]['home']

            home_e, home_n = gps_to_enu(
                h_lat, h_lon, ref_lat, ref_lon
            )

            self.send_cmd(
                did,
                'orbit',
                east=self.poly_center_e - home_e,
                north=self.poly_center_n - home_n,
                alt=self.poly_alt + 20.0,
                radius=self.poly_radius + 30.0
            )

    # ========================================================
    # 8. 쿼드로터 다각형 편대 목표 위치 생성
    # ========================================================
    # 이 코드에서 편대 형태를 결정하는 핵심 함수
    #
    # ① 드론 수에 따라 정다각형의 꼭짓점 수 결정
    # ② 원의 중심과 반지름을 기준으로 꼭짓점 계산
    # ③ 각 드론에 하나의 꼭짓점 할당
    # ④ 목표 위치로 이동하도록 reposition 명령 전송
    #
    # quad_step이 증가하면 목표 꼭짓점을 반시계 방향으로
    # 한 칸씩 이동시켜 드론들이 다음 위치로 이동하게 함
    #
    # 참고: 이 함수는 충돌 회피 및 경로 생성은 수행하지 않음
    # ========================================================

    def command_quad_polygon(self):

        # 모든 드론이 공유하는 좌표 기준
        ref_lat, ref_lon, _ = (
            self.drones[self.expected_ids[0]]['home']
        )

        # 드론 수가 3대 미만이어도 최소 삼각형 꼭짓점 사용
        num_vertices = max(3, self.nq)

        for i, did in enumerate(self.quad_ids):

            # ------------------------------------------------
            # 8-1. 드론별 목표 꼭짓점 각도 계산
            # ------------------------------------------------
            # 예: 드론 3대, quad_step=0
            # 1번: 0도 / 2번: 120도 / 3번: 240도
            # quad_step=1이면 각각 120도씩 이동

            angle = (
                2 * math.pi * (i + self.quad_step)
                / num_vertices
            )

            # ------------------------------------------------
            # 8-2. 정다각형 꼭짓점의 ENU 좌표 계산
            # ------------------------------------------------

            target_e = (
                self.poly_center_e
                + self.poly_radius * math.cos(angle)
            )

            target_n = (
                self.poly_center_n
                + self.poly_radius * math.sin(angle)
            )

            # ------------------------------------------------
            # 8-3. 각 드론의 홈 위치 기준 상대 목표 계산
            # ------------------------------------------------

            h_lat, h_lon, _ = self.drones[did]['home']

            home_e, home_n = gps_to_enu(
                h_lat, h_lon, ref_lat, ref_lon
            )

            # 목표 지점 도착 여부를 확인하기 위해 저장
            self.drones[did]['target_enu'] = (
                target_e, target_n
            )

            # ------------------------------------------------
            # 8-4. 목표 위치로 이동 명령 전송
            # ------------------------------------------------
            # 공통 기준 좌표를 각 드론의 홈 기준 상대 좌표로 변환

            self.send_cmd(
                did,
                'reposition',
                east=target_e - home_e,
                north=target_n - home_n,
                alt=self.poly_alt
            )


# ============================================================
# 9. ROS2 노드 실행
# ============================================================
# DTCController 노드를 생성하고 ROS2 메시지를 계속 처리
# 프로그램 종료 시 ROS2 종료
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DTCController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
