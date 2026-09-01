#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Mode 1: 복도 주행 노드 (Corridor Navigation) - v4

v3에 이어 이번에 바뀐 것:
- 변기칸 정렬 회전을 "절대 각도"가 아니라 "위치 도달한 순간의 heading 기준
  상대 회전"으로 변경. (시계방향 stall_turn_deg 만큼 회전 -> 변기칸 정면을 봄)
- mode2/mode3가 끝나고 mode1이 다시 활성화되면, 곧바로 다음 웨이포인트로
  가지 않고 먼저 "정렬 직전 heading"으로 반시계 방향으로 정확히 되돌아가는
  복귀 회전(resume rotation)을 한 번 거친 뒤에 다음 웨이포인트로 이동.
  -> 시퀀스: (포인트 도착) -> 시계 90도 회전 -> [mode2->mode3->mode2] ->
             반시계 90도 회전(복귀) -> 다음 포인트로 주행

기존 기능(odom 구독, I-PD 제어, 회전/직진 블렌딩, 장애물 정지, odom 끊김
안전장치, 최종 원점 정렬)은 그대로 유지.
"""

import math
import time
import matplotlib
matplotlib.use('Agg')  # GUI 백엔드 없이 실행하기 위한 설정
import matplotlib.pyplot as plt
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String
from tf.transformations import euler_from_quaternion


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Mode1CorridorNav(object):

    MODE_NAME = "MODE1_NAV"

    def __init__(self):
        rospy.init_node("mode1_nav_node")

        # ---------------- 파라미터 ----------------
        self.obstacle_stop_dist = rospy.get_param("~obstacle_stop_dist", 0.35)   # m
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.05)           # m

        # [수정] /lidar/front_distance는 아무도 발행하지 않는 죽은 토픽이었음
        # (lidar_node.py는 /lidar/front_points만 발행) - mode2_stall.py와
        # 같은 토픽을 직접 구독해서 전방 최소거리를 여기서 계산하도록 변경.
        # use_lidar=False로 주면 라이다 관련 체크(lidar_is_stale/장애물 정지)를
        # 아예 건너뛰어서 라이다 없이(odom만으로) 테스트할 수 있다.
        self.use_lidar = rospy.get_param("~use_lidar", True)
        self.lidar_front_half_width = rospy.get_param("~lidar_front_half_width", 0.15)
        # /lidar/front_points 가 이 시간(초) 이상 안 오면 안전 정지 (use_lidar=True일 때만)
        self.lidar_timeout = rospy.get_param("~lidar_timeout", 0.5)

        self.spin_only_angle_thresh = math.radians(
            rospy.get_param("~spin_only_angle_thresh_deg", 10.0))

        # I-PD 게인
        self.Kp_v = rospy.get_param("~Kp_v", 0.4)
        self.Ki_v = rospy.get_param("~Ki_v", 0.05)
        self.Kd_v = rospy.get_param("~Kd_v", 0.1)
        self.Kp_w = rospy.get_param("~Kp_w", 1.2)
        self.Ki_w = rospy.get_param("~Ki_w", 0.02)
        self.Kd_w = rospy.get_param("~Kd_w", 0.05)

        self.max_v = rospy.get_param("~max_v", 0.2)
        self.max_w = rospy.get_param("~max_w", 0.8)

        self.odom_timeout = rospy.get_param("~odom_timeout", 0.5)

        # ---- 변기칸 정렬 관련 파라미터 ----
        # 위치 도달 순간의 heading을 기준으로 이만큼 "상대 회전"해서 변기칸을 정면으로 봄.
        # 음수 = 시계방향, 양수 = 반시계방향. 기본값: 시계방향 90도.
        self.stall_turn_deg = rospy.get_param("~stall_turn_deg", -90.0)
        self.stall_turn_rad = math.radians(self.stall_turn_deg)

        self.stall_align_tolerance = rospy.get_param("~stall_align_tolerance", 0.03)  # rad
        self.stall_align_angular_speed = rospy.get_param("~stall_align_angular_speed", 0.3)  # rad/s

        # 최종(초기 위치 복귀 후) 정렬 관련
        self.final_align_tolerance = rospy.get_param("~final_align_tolerance", 0.02)  # rad
        self.final_align_angular_speed = rospy.get_param("~final_align_angular_speed", 0.3)  # rad/s

        # ---------------- 웨이포인트 ----------------
        # [x, y, type] 형식. type: "nav" 또는 "align:<stall_id>"
        #
        # 주의: 아래 좌표는 "맵 기준" 좌표에서 로봇의 시작점(예: 0.250, 0.250)을
        # 뺀 "odom 기준" 좌표임. /odom 토픽은 로봇 전원이 켜진 자리를 (0,0)으로
        # 간주하므로, 맵 기준 좌표를 그대로 쓰면 안 되고 시작점 좌표만큼 빼줘야 함.
        #   조정된 좌표 = 맵 기준 좌표 - (0.250, 0.250)
        # 맵 기준 원래 좌표(참고용):
        #   포인트1=(0.670,0.250) 포인트2=(0.670,0.825)
        #   align1=(1.160,0.825) align2=(1.700,0.825)
        #   포인트3=(1.750,1.250) 포인트4=(0.250,1.250) 초기위치=(0.250,0.250)
        self.waypoints = rospy.get_param("~waypoints", [
            [0.420, 0.000, "nav"],      # 포인트1: 복도 꺾이는 지점
            [0.420, 0.575, "nav"],      # 포인트2: 그 다음 복도 지점(아일 진입)
            [0.910, 0.575, "align:1"],  # align1: 1번 변기칸 앞 (도착 + 시계 90도 회전)
            [1.450, 0.575, "align:2"],  # align2: 2번 변기칸 앞 (도착 + 시계 90도 회전)
            [1.500, 1.000, "nav"],      # 포인트3: 그 다음 복도 꼭짓점
            [0.000, 1.000, "nav"],      # 포인트4: 마지막 복도 꼭짓점
            [0.000, 0.000, "nav"],      # 초기위치: 최종 복귀 지점 (odom 원점)
        ])
        self.current_idx = 0

        # ---------------- 상태 변수 ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_received = False
        self.last_odom_time = None

        self.front_dist = None     # lidar_node가 발행하는 전방 최소거리 (m)
        self.last_lidar_time = None
        self.active = False

        # align 웨이포인트 서브 상태
        self.aligning = False               # 위치 도달 후 회전 정렬 중인지
        self.pre_align_theta = None         # 회전 시작 직전(=복귀 목표) heading
        self.align_target_theta = None      # 이번 정렬에서 도달해야 할 절대 각도

        # mode2/3 완료 대기 및 복귀 회전 관련
        self.awaiting_mode_switch = False   # mode2/3 끝나기를 기다리는 중
        self.resuming_alignment = False     # 재활성화 직후, 복귀 회전(반시계) 중

        # I-PD 내부 변수
        self.integral_v = 0.0
        self.integral_w = 0.0
        self.prev_dist_err = 0.0
        self.prev_theta = 0.0
        self.path_history = [] # 이동 경로 저장을 위한 리스트
        self.prev_loop_time = None

        self._last_status = None

        # ---------------- Publisher / Subscriber ----------------
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.status_pub = rospy.Publisher("/mode1_status", String, queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        if self.use_lidar:
            rospy.Subscriber("/lidar/front_points", Float32MultiArray, self.front_points_callback)
        else:
            rospy.loginfo("[Mode1] use_lidar=False -> 라이다 장애물 감지 비활성화 (odom만으로 주행)")
        rospy.Subscriber("/robot_mode", String, self.mode_callback)

        self.rate = rospy.Rate(20)

        rospy.loginfo("[Mode1] 노드 시작. 대기 중 (mode=%s 신호를 기다림)", self.MODE_NAME)

        # 노드 종료 시 경로 이미지 저장 함수를 호출하도록 등록
        rospy.on_shutdown(self.save_path_image)

    # ==================== Callbacks ====================
    def mode_callback(self, msg):
        was_active = self.active
        self.active = (msg.data == self.MODE_NAME)

        if was_active and not self.active:
            self.stop_robot()
            rospy.loginfo("[Mode1] 비활성화 -> 정지")
        elif not was_active and self.active:
            rospy.loginfo("[Mode1] 활성화됨. 현재 웨이포인트 인덱스=%d", self.current_idx)
            self.reset_integral()
            self.prev_loop_time = None

            if self.awaiting_mode_switch:
                # mode2 -> mode3 -> mode2 사이클이 끝나고 mode_manager가 다시
                # MODE1_NAV로 돌려준 것 -> 곧장 이동하지 말고 먼저 복귀 회전부터
                rospy.loginfo("[Mode1] 모드2/3 완료 확인 -> 복귀 회전(반시계) 시작")
                self.awaiting_mode_switch = False
                self.resuming_alignment = True

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_received = True
        self.last_odom_time = rospy.Time.now()
        self.path_history.append((self.x, self.y)) # 현재 위치를 경로에 추가

    def front_points_callback(self, msg):
        """/lidar/front_points(base_link 기준 x,y 평탄화 배열)에서 정면 좁은
        밴드(lidar_front_half_width) 안 최소 x값을 전방거리로 계산한다.
        mode2_stall.py의 raw_front_distance()와 같은 방식."""
        points = list(zip(msg.data[0::2], msg.data[1::2]))
        candidates = [x for (x, y) in points if x > 0 and abs(y) < self.lidar_front_half_width]
        self.front_dist = min(candidates) if candidates else float('inf')
        self.last_lidar_time = rospy.Time.now()

    # ==================== Helper ====================

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_status(self, text):
        if text != self._last_status:
            self.status_pub.publish(String(data=text))
            self._last_status = text

    def reset_integral(self):
        self.integral_v = 0.0
        self.integral_w = 0.0

    def odom_is_stale(self):
        if self.last_odom_time is None:
            return True
        return (rospy.Time.now() - self.last_odom_time).to_sec() > self.odom_timeout

    def lidar_is_stale(self):
        if not self.use_lidar:
            return False
        if self.last_lidar_time is None:
            return True
        return (rospy.Time.now() - self.last_lidar_time).to_sec() > self.lidar_timeout

    def is_obstacle_ahead(self):
        if not self.use_lidar:
            return False
        return (self.front_dist is not None) and (self.front_dist < self.obstacle_stop_dist)

    def compute_loop_dt(self):
        now = rospy.Time.now()
        if self.prev_loop_time is None:
            dt = 1.0 / 20.0
        else:
            dt = (now - self.prev_loop_time).to_sec()
        dt = max(dt, 1e-3)
        self.prev_loop_time = now
        return dt

    def rotate_in_place(self, target_theta, tolerance, angular_speed):
        """target_theta로 제자리 회전. 도달했으면 True, 아니면 회전 명령을 보내고 False."""
        angle_err = normalize_angle(target_theta - self.theta)
        if abs(angle_err) <= tolerance:
            self.stop_robot()
            return True

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = angular_speed if angle_err > 0 else -angular_speed
        self.cmd_pub.publish(cmd)
        return False

    def drive_toward(self, target_x, target_y):
        """I-PD 제어 + 회전/직진 블렌딩으로 target_x, target_y 방향 cmd_vel 계산 및 발행."""
        dt = self.compute_loop_dt()

        dx = target_x - self.x
        dy = target_y - self.y
        dist_err = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)

        # [수정] 목표 지점에 매우 근접했을 때 atan2가 불안정해지는 것을 방지.
        # dist_err가 goal_tolerance보다 작아지면 새로운 목표 각도를 계산하지 않고
        # 이전의 안정적인 각도를 유지하여 직진 중 방향이 튀는 현상을 막음.
        if dist_err > self.goal_tolerance:
            target_theta = math.atan2(dy, dx)
        else:
            target_theta = self.prev_target_theta  # 이전 목표 각도 유지
        angle_err = normalize_angle(target_theta - self.theta)

        self.integral_v += dist_err * dt
        v_raw = (self.Ki_v * self.integral_v) + (self.Kp_v * dist_err) + \
                (self.Kd_v * (dist_err - self.prev_dist_err) / dt)

        self.integral_w += angle_err * dt
        w_raw = (self.Ki_w * self.integral_w) + (self.Kp_w * angle_err) - \
                (self.Kd_w * (self.theta - self.prev_theta) / dt)

        self.prev_dist_err = dist_err
        self.prev_theta = self.theta
        self.prev_target_theta = target_theta # 현재 목표 각도를 다음 루프를 위해 저장

        if abs(angle_err) > self.spin_only_angle_thresh:
            v = 0.0
            w = max(-self.max_w, min(self.max_w, w_raw))
        else:
            v_weight = max(0.0, math.cos(angle_err))
            v = max(-self.max_v, min(self.max_v, v_raw * v_weight))
            w = max(-self.max_w, min(self.max_w, w_raw))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def save_path_image(self):
        """노드 종료 시, 현재까지의 주행 경로를 이미지 파일로 저장합니다."""
        if not self.path_history:
            rospy.loginfo("[Mode1] 저장할 경로 기록이 없습니다.")
            return

        rospy.loginfo("[Mode1] 주행 경로를 이미지 파일로 저장합니다...")
        try:
            plt.figure(figsize=(10, 10))

            # 웨이포인트 플로팅
            wp_x = [wp[0] for wp in self.waypoints]
            wp_y = [wp[1] for wp in self.waypoints]
            plt.plot(wp_x, wp_y, 'go-', label='Waypoints', markersize=10)
            plt.plot(wp_x[0], wp_y[0], 'bo', label='Start/End', markersize=12)

            # 실제 주행 경로 플로팅
            path_x = [p[0] for p in self.path_history]
            path_y = [p[1] for p in self.path_history]
            plt.plot(path_x, path_y, 'r-', label='Robot Path', linewidth=2)

            plt.title('Robot Trajectory')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.grid(True)
            plt.legend()
            plt.axis('equal')

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = "path_{}.png".format(timestamp)
            plt.savefig(filename)
            rospy.loginfo("[Mode1] 경로가 '{}' 파일로 저장되었습니다.".format(filename))
        except Exception as e:
            rospy.logerr("[Mode1] 경로 이미지 저장 중 오류 발생: %s", e)

    # ==================== Waypoint handlers ====================
    def step_nav(self, target_x, target_y):
        dx = target_x - self.x
        dy = target_y - self.y
        dist_err = math.hypot(dx, dy)

        if dist_err < self.goal_tolerance:
            rospy.loginfo("[Mode1] [도착] 내비게이션 웨이포인트 %d (%s) 도착. 다음으로 이동합니다.",
                          self.current_idx, self.waypoints[self.current_idx][:2])
            self.publish_status("MODE1_WAYPOINT_REACHED:%d" % self.current_idx)
            self.current_idx += 1
            self.stop_robot()
            self.reset_integral()
            return

        self.drive_toward(target_x, target_y)
        self.publish_status("MODE1_MOVING:%d" % self.current_idx)

    def step_align(self, target_x, target_y, stall_id):
        if not self.aligning:
            dx = target_x - self.x
            dy = target_y - self.y
            dist_err = math.hypot(dx, dy)

            if dist_err < self.goal_tolerance:
                rospy.loginfo("[Mode1] [도착] 변기칸 %s 앞 위치 도착. 시계방향 %.1f도 회전을 시작합니다.",
                              stall_id, -self.stall_turn_deg)
                # 위치 도달 -> 이 순간의 heading을 저장하고(복귀용), 회전 목표 계산
                self.aligning = True
                self.pre_align_theta = self.theta
                self.align_target_theta = normalize_angle(self.theta + self.stall_turn_rad)
                self.stop_robot()
                self.reset_integral()
                self.publish_status("MODE1_STALL_POS_REACHED:%s" % stall_id)
                return

            self.drive_toward(target_x, target_y)
            self.publish_status("MODE1_MOVING:%d" % self.current_idx)
            return

        # ---- 회전 정렬 단계 (시계 stall_turn_deg 만큼) ----
        reached = self.rotate_in_place(
            self.align_target_theta, self.stall_align_tolerance, self.stall_align_angular_speed)

        if not reached:
            self.publish_status("MODE1_STALL_ALIGNING:%s" % stall_id)
            return

        # 정렬 완료 -> mode_manager에 신호를 보내고 대기 (pre_align_theta는 복귀용으로 유지)
        self.publish_status("MODE1_STALL_REACHED:%s" % stall_id)
        rospy.loginfo("[Mode1] 변기칸 %s 앞 도착+정렬 완료 -> mode_manager 응답 대기", stall_id)
        self.current_idx += 1
        self.aligning = False
        self.awaiting_mode_switch = True

    def step_resume_rotation(self):
        """mode2/3 완료 후 재활성화된 직후: 정렬 직전 heading으로 반시계 복귀 회전."""
        if self.pre_align_theta is None:
            # 저장된 값이 없으면(비정상 상황) 그냥 건너뜀
            self.resuming_alignment = False
            return

        reached = self.rotate_in_place(
            self.pre_align_theta, self.stall_align_tolerance, self.stall_align_angular_speed)

        if not reached:
            self.publish_status("MODE1_RESUME_ALIGNING")
            return

        rospy.loginfo("[Mode1] 복귀 회전 완료 -> 다음 웨이포인트로 이동")
        self.resuming_alignment = False
        self.pre_align_theta = None

    def step_final_alignment(self):
        reached = self.rotate_in_place(
            0.0, self.final_align_tolerance, self.final_align_angular_speed)

        if not reached:
            self.publish_status("MODE1_FINAL_ALIGNING")
        else:
            rospy.loginfo("[Mode1] [최종완료] 모든 웨이포인트 주행 및 최종 정렬 완료. 임무를 마칩니다.")
            self.publish_status("MODE1_ALL_WAYPOINTS_DONE")

    # ==================== Main control loop ====================
    def step(self):
        if not self.active:
            return

        if self.awaiting_mode_switch:
            # 변기칸 정렬 끝내고 mode2/3가 끝나기를 기다리는 중
            return

        if not self.pose_received:
            rospy.loginfo_throttle(2.0, "[Mode1] odom 수신 대기 중...")
            return

        if self.odom_is_stale():
            self.stop_robot()
            self.publish_status("MODE1_ODOM_TIMEOUT")
            return

        if self.lidar_is_stale():
            self.stop_robot()
            self.publish_status("MODE1_LIDAR_TIMEOUT")
            return

        if self.resuming_alignment:
            # mode2/3 끝나고 재활성화된 직후: 다음 웨이포인트 이동보다 복귀 회전이 우선
            self.step_resume_rotation()
            return

        if self.current_idx >= len(self.waypoints):
            self.step_final_alignment()
            return

        if self.is_obstacle_ahead():
            self.stop_robot()
            self.reset_integral()
            self.publish_status("MODE1_OBSTACLE_STOP")
            return

        target_x, target_y, wp_type = self.waypoints[self.current_idx]

        if wp_type.startswith("align"):
            stall_id = wp_type.split(":")[1] if ":" in wp_type else "?"
            self.step_align(target_x, target_y, stall_id)
        else:
            self.step_nav(target_x, target_y)

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = Mode1CorridorNav()
        node.run()
    except rospy.ROSInterruptException:
        pass