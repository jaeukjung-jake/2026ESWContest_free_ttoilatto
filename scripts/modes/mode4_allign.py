#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
mode4_align_node

AprilTag(ID=1) 기반 위치 재정렬 노드.

- /robot_mode 가 "MODE4_INIT" 일 때만 동작 (다른 모드 노드들과 동일한 패턴)
- calibrate_offset.py가 미리 만들어둔 mode4_offset.yaml(목표 x,y,z,yaw raw값)을
  로드해서, 실시간 /apriltag/pose 와 "같은 변환 함수"를 통과시켜 비교함
- 카메라가 55도 아래로 기울어져 있어서, z_ground = z*cos(tilt) + y*sin(tilt)
  로 "바닥 평면 기준 전진거리"를 복원해서 씀. x는 틸트 축과 평행이라 그대로 사용.

핵심 아이디어 (look-then-move):
  태그는 고정되어 있고, 목표값(offset)은 "정확히 도킹됐을 때의 카메라 위치에서
  태그가 이렇게 보인다"는 기준값. 문제는 현재값과 목표값이 서로 다른(회전된)
  카메라 좌표계에서 측정된 벡터라는 점 -> 그냥 성분별로 빼면(dx=cur-tgt)
  dyaw≈0일 때만 우연히 맞고, dyaw가 크면 방향까지 틀어짐.
  compute_error()가 목표 벡터를 dyaw만큼 회전시켜 현재 좌표계로 맞춘 뒤 빼서
  "카메라(=로봇)가 지금 이 벡터만큼 더 이동해야 한다"는 정확한 오차(dx, dz_ground)를 계산.
  -> 이 오차를 로봇 로컬 좌표(전진/좌우)로 변환 -> odom 기준 절대 좌표
     웨이포인트로 변환 -> mode1과 같은 방식으로 회전+전진 -> 다시 멈춰서 확인.

상태머신:
  CAPTURE            : 정지 상태에서 태그를 N프레임 평균 -> 오차(dx, dzg, dyaw) 계산.
                       compute_error()에서 target 벡터를 dyaw만큼 회전시킨 뒤
                       빼므로(rotate_ground), dyaw가 커도(예: 30도+) 정확한
                       로컬 이동 벡터가 나온다 (예전엔 단순 성분별 subtraction이라
                       dyaw≈0일 때만 맞고 클수록 방향까지 틀어지는 버그가 있었음).
  ROTATE             : 계산된 방향으로 제자리 회전 (odom 기준)
  DRIVE              : 계산된 거리만큼 전진 (odom 기준, 회전 보정 포함)
  (반복: 위치 오차가 허용범위 밖이면 CAPTURE로 되돌아가 재시도)
  YAW_ALIGN_CAPTURE  : 위치가 맞춰지면, yaw 오차만 다시 측정
  YAW_ALIGN_ROTATE   : yaw 오차만큼 제자리 회전
  STABLE_CHECK       : 정지 상태로 실시간 /apriltag/pose를 계속 관찰,
                       10프레임 연속으로 오차가 전부 허용범위 안이면 완료
  TAG_LOST_SEARCH    : CAPTURE/YAW_ALIGN_CAPTURE/STABLE_CHECK 중 태그가 안 보이면
                       좌우로 천천히 훑어보는 탐색 회전
  DONE               : /mode4_status 에 "MODE4_INIT_DONE" 발행 후 대기
"""

import os
import math
import yaml
import rospy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_matrix


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def quat_to_yaw(q, tilt_rad=0.0):
    """카메라 좌표계(X right, Y down, Z forward) 기준 태그의 yaw 추출.
    calibrate_offset.py와 반드시 '동일한 정의'를 써야 함 (그대로 복사해옴).

    카메라가 tilt_rad만큼 아래로 기울어져 장착돼 있으면, 원본(기울어진)
    프레임의 회전행렬을 그대로 써서 atan2(r02, r22)를 구하면 안 됨 -> to_ground()가
    위치(x,y,z)에 적용하는 것과 '동일한' Rx(tilt) 보정을 회전행렬에도 적용해야
    실제 세계 기준(수평) yaw가 나옴. 유도해보면 r02는 그대로, r22만
    r12*sin(tilt) + r22*cos(tilt)로 바뀜 (tilt=0이면 원래 식과 동일).
    """
    mat = quaternion_matrix(q)
    r02 = mat[0][2]
    r12 = mat[1][2]
    r22 = mat[2][2]
    r22_ground = r12 * math.sin(tilt_rad) + r22 * math.cos(tilt_rad)
    return math.atan2(r02, r22_ground)


class Mode4AlignNode(object):

    MODE_NAME = "MODE4_INIT"

    def __init__(self):
        rospy.init_node("mode4_align_node")

        # ---------------- 파라미터 ----------------
        default_yaml = os.path.expanduser("~/catkin_ws/src/final_git/param/mode4_offset.yaml")
        self.offset_yaml_path = rospy.get_param("~offset_yaml_path", default_yaml)

        self.tilt_deg = rospy.get_param("~tilt_deg", 55.0)
        self.tilt_rad = math.radians(self.tilt_deg)

        self.pos_tolerance = rospy.get_param("~pos_tolerance", 0.005)          # m (=0.5cm)
        self.yaw_tolerance = math.radians(rospy.get_param("~yaw_tolerance_deg", 3.0))
        self.stable_count_required = rospy.get_param("~stable_count_required", 10)

        self.capture_samples = rospy.get_param("~capture_samples", 8)
        self.capture_timeout = rospy.get_param("~capture_timeout", 2.0)        # sec
        self.tag_lost_timeout = rospy.get_param("~tag_lost_timeout", 1.5)      # sec
        self.max_iterations = rospy.get_param("~max_iterations", 6)

        # 로봇 로컬 좌표 변환 시 부호가 반대로 나오면 이 파라미터만 뒤집으면 됨
        self.yaw_correction_sign = rospy.get_param("~yaw_correction_sign", 1.0)

        # odom 기반 회전/전진 제어 파라미터 (mode1과 동일한 성격)
        self.goal_tolerance = rospy.get_param("~odom_goal_tolerance", 0.02)     # m
        self.angle_tolerance = rospy.get_param("~odom_angle_tolerance", 0.03)   # rad
        self.linear_speed = rospy.get_param("~linear_speed", 0.10)
        self.angular_speed = rospy.get_param("~angular_speed", 0.3)
        self.search_angular_speed = rospy.get_param("~search_angular_speed", 0.15)
        self.search_max_angle = math.radians(rospy.get_param("~search_max_angle_deg", 30.0))

        # ---------------- 목표 offset 로드 ----------------
        self.target_x, self.target_y, self.target_z, self.target_yaw = self.load_offset()

        # ---------------- 상태 변수 ----------------
        self.active = False
        # __init__에서는 실제 상태 이름과 절대 겹치지 않는 placeholder를 씀.
        # set_state()는 "state가 실제로 바뀔 때만" 초기화를 수행하는 가드가 있어서,
        # 만약 여기서 곧바로 "CAPTURE"를 대입해두면 reset_to_capture()의
        # set_state("CAPTURE") 호출이 가드에 걸려 capture_start_time 초기화가
        # 스킵되고, 첫 진입 시 step_capture()에서 None - Time 연산으로 죽는다.
        self.state = "INIT"
        self.iteration_count = 0

        # odom
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.pose_received = False

        # apriltag 실시간 값 (콜백에서 계속 갱신)
        self.tag_x = None
        self.tag_y = None
        self.tag_z = None
        self.tag_yaw = None
        self.last_tag_time = None

        # CAPTURE 단계용 샘플 버퍼
        self.capture_buffer = []
        self.capture_start_time = None

        # ROTATE/DRIVE 단계용 목표값
        self.target_abs_theta = None
        self.target_wp_x = None
        self.target_wp_y = None

        # 탐색 회전용
        self.search_start_theta = None
        self.search_direction = 1.0
        self._search_return_state = "CAPTURE"

        # STABLE_CHECK용
        self.stable_count = 0

        self._last_status = None

        # ---------------- Publisher / Subscriber ----------------
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.status_pub = rospy.Publisher("/mode4_status", String, queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/apriltag/pose", PoseStamped, self.tag_callback)
        rospy.Subscriber("/robot_mode", String, self.mode_callback)

        self.rate = rospy.Rate(20)

        rospy.loginfo("[Mode4] 노드 시작. 목표 offset: x=%.4f y=%.4f z=%.4f yaw=%.2fdeg",
                      self.target_x, self.target_y, self.target_z, math.degrees(self.target_yaw))
        rospy.loginfo("[Mode4] 대기 중 (mode=%s 신호를 기다림)", self.MODE_NAME)

    # ==================== 초기화 ====================
    def load_offset(self):
        if not os.path.exists(self.offset_yaml_path):
            rospy.logerr("[Mode4] offset yaml을 찾을 수 없음: %s "
                         "(calibrate_offset.py를 먼저 실행해야 함)", self.offset_yaml_path)
            raise RuntimeError("mode4_offset.yaml not found")

        with open(self.offset_yaml_path, "r") as f:
            data = yaml.safe_load(f)

        return (data["x_offset"], data["y_offset"], data["z_offset"], data["yaw_offset"])

    # ==================== Callbacks ====================
    def mode_callback(self, msg):
        was_active = self.active
        self.active = (msg.data == self.MODE_NAME)

        if was_active and not self.active:
            self.stop_robot()
            rospy.loginfo("[Mode4] 비활성화 -> 정지")
        elif not was_active and self.active:
            rospy.loginfo("[Mode4] 활성화됨. 정렬 시작")
            self.reset_to_capture()

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.pose_received = True   # 메시지 수신 자체를 신뢰의 기준으로
        q = msg.pose.pose.orientation
        try:
            _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.odom_theta = theta
        except Exception as e:
            rospy.logwarn_throttle(1.0, "[Mode4] odom 쿼터니언 변환 실패: %s", e)

    def tag_callback(self, msg):
        p = msg.pose.position
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]

        self.tag_x = p.x
        self.tag_y = p.y
        self.tag_z = p.z
        self.tag_yaw = quat_to_yaw(q, self.tilt_rad)
        self.last_tag_time = rospy.Time.now()

        # CAPTURE / YAW_ALIGN_CAPTURE 단계에서만 샘플로 축적
        if self.state in ("CAPTURE", "YAW_ALIGN_CAPTURE"):
            self.capture_buffer.append((self.tag_x, self.tag_y, self.tag_z, self.tag_yaw))

        # STABLE_CHECK 단계에서는 매 프레임 바로바로 판정
        if self.state == "STABLE_CHECK":
            self.check_stability_frame()

    # ==================== Helper: 좌표 변환 ====================
    def to_ground(self, x, y, z):
        """카메라 틸트(55도)를 보정해서 (x, z_ground) 반환. y는 여기서 다 씀."""
        z_ground = z * math.cos(self.tilt_rad) + y * math.sin(self.tilt_rad)
        return x, z_ground

    def rotate_ground(self, x, zg, theta):
        """(x, z_ground) 평면 벡터를 theta만큼 회전. quat_to_yaw와 동일한 회전
        컨벤션(Y축 기준, r02=sin, r22=cos)을 따름 -> compute_error()에서
        서로 다른 카메라 자세 기준으로 측정된 두 벡터를 같은 기준으로
        맞추는 데 씀."""
        c = math.cos(theta)
        s = math.sin(theta)
        return x * c + zg * s, -x * s + zg * c

    def compute_error(self, x, y, z, yaw):
        """raw (x,y,z,yaw) 측정값 하나를 목표(offset)와 비교해 (dx, dzg, dyaw)를
        계산한다. cur/target은 서로 다른(회전된) 카메라 좌표계에서 측정된
        벡터이므로, target 벡터를 dyaw만큼 회전시켜 현재 좌표계로 맞춘 뒤
        빼야 한다 (단순 성분별 subtraction은 dyaw≈0일 때만 우연히 맞음).
        """
        cur_x, cur_zg = self.to_ground(x, y, z)
        tgt_x, tgt_zg = self.to_ground(self.target_x, self.target_y, self.target_z)
        dyaw = normalize_angle(yaw - self.target_yaw)

        rot_tgt_x, rot_tgt_zg = self.rotate_ground(tgt_x, tgt_zg, dyaw)
        dx = cur_x - rot_tgt_x
        dzg = cur_zg - rot_tgt_zg
        return dx, dzg, dyaw

    # ==================== Helper: 로봇 동작 ====================
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_status(self, text):
        if text != self._last_status:
            self.status_pub.publish(String(data=text))
            self._last_status = text

    def tag_is_fresh(self):
        if self.last_tag_time is None:
            return False
        return (rospy.Time.now() - self.last_tag_time).to_sec() < self.tag_lost_timeout

    # ==================== 상태 전환 ====================
    def set_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo("[Mode4] State: %s -> %s", self.state, new_state)
            self.state = new_state

            # 새 상태 진입 시 필요한 초기화 작업
            if new_state == "CAPTURE" or new_state == "YAW_ALIGN_CAPTURE":
                self.capture_buffer = []
                self.capture_start_time = rospy.Time.now()
            elif new_state == "STABLE_CHECK":
                self.stable_count = 0
            # DONE 상태는 여기서 자동으로 상태를 발행하지 않는다.
            # 성공(STABLE_CHECK 완료)과 실패(max_iterations 초과)가 각각
            # 서로 다른 상태 문자열을 발행해야 하므로, 호출부에서 명시적으로
            # publish_status()를 먼저 호출한 뒤 set_state("DONE")을 부른다.

    def reset_to_capture(self):
        self.set_state("CAPTURE")
        self.iteration_count = 0
        self.stop_robot()

    # ==================== CAPTURE ====================
    def step_capture(self):
        """정지 상태로 N개 샘플을 모아 평균 -> dx,dz_ground,dyaw 계산.
        결과는 (dx, dzg, dyaw, done) 튜플로 반환하고, 상태 전환은
        호출부(step())에서 처리한다.

        동시에, 태그의 odom(world) 절대 좌표(tag_world_x, tag_world_y)를
        역산해서 저장해둔다. DRIVE가 끝난 뒤 FACE_TAG 단계에서, 실제로
        도착한 위치 기준으로 이 좌표를 다시 바라보는 방향을 계산하는 데 쓴다.
        """
        self.stop_robot()

        elapsed = (rospy.Time.now() - self.capture_start_time).to_sec()

        if not self.tag_is_fresh():
            if elapsed > self.tag_lost_timeout:
                rospy.logwarn("[Mode4] 태그를 못 찾음 -> 탐색 회전 시작")
                self.start_search(self.state)  # 현재 상태(CAPTURE)로 복귀하도록 탐색 시작
            return None, None, None, False  # 아직 결과 없음

        if len(self.capture_buffer) < self.capture_samples and elapsed < self.capture_timeout:
            return None, None, None, False  # 더 모으기

        if len(self.capture_buffer) == 0:
            rospy.logwarn_throttle(1.0, "[Mode4] 캡처 버퍼가 비어있음, 재시도")
            self.capture_start_time = rospy.Time.now()
            return None, None, None, False  # 재시도

        n = len(self.capture_buffer)
        if n < self.capture_samples:
            rospy.logwarn("[Mode4] 캡처 타임아웃으로 샘플 부족 (%d/%d)", n, self.capture_samples)

        avg_x = sum(s[0] for s in self.capture_buffer) / n
        avg_y = sum(s[1] for s in self.capture_buffer) / n
        avg_z = sum(s[2] for s in self.capture_buffer) / n
        avg_yaw = sum(s[3] for s in self.capture_buffer) / n

        dx, dzg, dyaw = self.compute_error(avg_x, avg_y, avg_z, avg_yaw)

        rospy.loginfo("[Mode4] CAPTURE(n=%d) dx=%.4f dzg=%.4f dyaw=%.2fdeg",
                      n, dx, dzg, math.degrees(dyaw))

        return dx, dzg, dyaw, True

    def plan_move(self, dx, dzg):
        """계산된 오차를 바탕으로 로봇의 다음 odom 목표 지점을 계산합니다.
        반환값 True면 이동이 필요해서 target_abs_theta/target_wp_x/y가 세팅된 것,
        False면 이미 허용범위 안이거나(이동 불필요) 최대 시도 초과로 실패한 것."""
        if abs(dx) <= self.pos_tolerance and abs(dzg) <= self.pos_tolerance:
            return False  # 이동 불필요

        self.iteration_count += 1
        if self.iteration_count > self.max_iterations:
            rospy.logerr("[Mode4] 최대 시도 횟수(%d) 초과 -> 정렬 실패", self.max_iterations)
            self.publish_status("MODE4_ALIGN_FAILED")
            self.set_state("DONE")  # 더 이상 진행 안 함 (관리자가 개입해야 함)
            return False

        # ---- 오차 -> 로봇 로컬(전진/좌) -> odom 절대 웨이포인트로 변환 ----
        # 카메라 x(오른쪽)는 로봇 -y(오른쪽), 카메라 z_ground(전방)는 로봇 +x(전방)에 대응
        # (카메라가 로봇 진행방향과 같은 yaw로 장착되어 있다는 가정. 다르면 여기에
        #  장착 yaw 오프셋을 추가로 더해줘야 함)
        local_forward = dzg
        local_left = -dx

        bearing = math.atan2(local_left, local_forward)
        distance = math.hypot(dx, dzg)

        self.target_abs_theta = normalize_angle(self.odom_theta + bearing)
        self.target_wp_x = self.odom_x + distance * math.cos(self.target_abs_theta)
        self.target_wp_y = self.odom_y + distance * math.sin(self.target_abs_theta)

        rospy.loginfo("[Mode4] 이동 계획: bearing=%.1fdeg distance=%.3fm",
                      math.degrees(bearing), distance)

        return True  # 이동 필요

    # ==================== ROTATE / DRIVE (odom 기준, mode1과 동일한 방식) ====================
    def step_rotate(self, target_theta, next_state):
        angle_err = normalize_angle(target_theta - self.odom_theta)
        if abs(angle_err) <= self.angle_tolerance:
            self.stop_robot()
            self.set_state(next_state)
            return

        cmd = Twist()
        cmd.angular.z = self.angular_speed if angle_err > 0 else -self.angular_speed
        self.cmd_pub.publish(cmd)

    def step_drive(self, target_x, target_y, next_state):
        dx = target_x - self.odom_x
        dy = target_y - self.odom_y
        dist_err = math.hypot(dx, dy)

        if dist_err <= self.goal_tolerance:
            self.stop_robot()
            self.set_state(next_state)
            return

        target_theta = math.atan2(dy, dx)
        angle_err = normalize_angle(target_theta - self.odom_theta)

        # 각도 오차가 클수록 전진속도를 cos 가중치로 부드럽게 줄이고,
        # 회전속도는 각도 오차 크기에 "선형 비례"하게 준다.
        # (주의: (1 - cos(angle_err)) 가중치는 tolerance를 살짝 넘는 작은 각도
        #  구간에서 회전 보정력이 거의 0에 가까워져 로봇이 방향을 거의
        #  틀지 않은 채 그대로 직진해버리는 문제가 있어서 선형 비례로 대체함)
        v_weight = max(0.0, math.cos(angle_err))
        v = self.linear_speed * v_weight

        w_sign = 1.0 if angle_err > 0 else -1.0
        w_weight = min(1.0, abs(angle_err) / math.radians(45.0))
        w = w_sign * self.angular_speed * w_weight

        cmd = Twist()
        if abs(angle_err) > self.angle_tolerance:
            cmd.linear.x = v
            cmd.angular.z = w
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    # ==================== YAW ALIGN ====================
    def step_yaw_align_capture(self):
        self.stop_robot()
        elapsed = (rospy.Time.now() - self.capture_start_time).to_sec()

        if not self.tag_is_fresh():
            if elapsed > self.tag_lost_timeout:
                rospy.logwarn("[Mode4] yaw 정렬 중 태그를 못 찾음 -> 탐색 회전")
                self.start_search(self.state)
            return

        if len(self.capture_buffer) < self.capture_samples and elapsed < self.capture_timeout:
            return

        if len(self.capture_buffer) == 0:
            self.capture_start_time = rospy.Time.now()
            return

        n = len(self.capture_buffer)
        avg_yaw = sum(s[3] for s in self.capture_buffer) / n
        dyaw = normalize_angle(avg_yaw - self.target_yaw)

        rospy.loginfo("[Mode4] YAW_ALIGN_CAPTURE dyaw=%.2fdeg", math.degrees(dyaw))

        if abs(dyaw) <= self.yaw_tolerance:
            self.set_state("STABLE_CHECK")
            return

        self.target_abs_theta = normalize_angle(
            self.odom_theta - self.yaw_correction_sign * dyaw)
        self.set_state("YAW_ALIGN_ROTATE")

    # ==================== TAG LOST SEARCH ====================
    def start_search(self, return_state):
        self.search_start_theta = self.odom_theta
        self.search_direction = 1.0
        self._search_return_state = return_state
        self.state = "TAG_LOST_SEARCH"

    def step_search(self, return_state):
        if self.tag_is_fresh():
            rospy.loginfo("[Mode4] 태그 재발견 -> 캡처 재시작")
            self.set_state(return_state)
            return

        angle_from_start = normalize_angle(self.odom_theta - self.search_start_theta)

        if abs(angle_from_start) >= self.search_max_angle:
            self.search_direction *= -1.0  # 반대 방향으로 훑기

        cmd = Twist()
        cmd.angular.z = self.search_direction * self.search_angular_speed
        self.cmd_pub.publish(cmd)

    # ==================== STABLE CHECK (실시간 콜백에서 프레임마다 판정) ====================
    def check_stability_frame(self):
        if self.tag_x is None:
            return

        dx, dzg, dyaw = self.compute_error(self.tag_x, self.tag_y, self.tag_z, self.tag_yaw)

        within = (abs(dx) <= self.pos_tolerance and
                  abs(dzg) <= self.pos_tolerance and
                  abs(dyaw) <= self.yaw_tolerance)

        if within:
            self.stable_count += 1
        else:
            if self.stable_count > 0:
                rospy.loginfo("[Mode4] 안정성 체크 중 오차 이탈 -> 카운트 리셋 "
                              "(dx=%.4f dzg=%.4f dyaw=%.2fdeg)",
                              dx, dzg, math.degrees(dyaw))
            self.stable_count = 0

        if self.stable_count >= self.stable_count_required:
            rospy.loginfo("[Mode4] 정렬 완료 확인 (연속 %d프레임)", self.stable_count)
            self.publish_status("MODE4_INIT_DONE")
            self.set_state("DONE")

    # ==================== Main control loop ====================
    def step(self):
        if not self.active:
            return

        if not self.pose_received:
            rospy.loginfo_throttle(2.0, "[Mode4] odom 수신 대기 중...")
            return

        if self.state == "CAPTURE":
            dx, dzg, _, done = self.step_capture()
            if done:
                if self.plan_move(dx, dzg):
                    self.set_state("ROTATE")
                else:  # 이동 불필요(또는 실패로 DONE 진입) -> yaw 정렬로
                    if self.state != "DONE":
                        self.set_state("YAW_ALIGN_CAPTURE")

        elif self.state == "ROTATE":
            self.step_rotate(self.target_abs_theta, "DRIVE")

        elif self.state == "DRIVE":
            self.step_drive(self.target_wp_x, self.target_wp_y, "CAPTURE")

        elif self.state == "YAW_ALIGN_CAPTURE":
            self.step_yaw_align_capture()

        elif self.state == "YAW_ALIGN_ROTATE":
            self.step_rotate(self.target_abs_theta, "STABLE_CHECK")

        elif self.state == "TAG_LOST_SEARCH":
            # 탐색 재개 지점(CAPTURE/YAW_ALIGN_CAPTURE/STABLE_CHECK)을
            # 기억해뒀다가 복귀
            return_state = getattr(self, "_search_return_state", "CAPTURE")
            self.step_search(return_state)

        elif self.state == "STABLE_CHECK":
            # 판정 자체는 tag_callback에서 실시간으로 함. 여기선 정지 유지만.
            self.stop_robot()
            if not self.tag_is_fresh():
                # STABLE_CHECK는 이미 위치가 거의 맞춰진 상태이므로, 태그를
                # 놓쳤다가 재발견했을 때 처음(CAPTURE)부터가 아니라
                # YAW_ALIGN_CAPTURE로 복귀해서 각도부터 재확인하는 게 더
                # 효율적이고 안정적이다.
                rospy.logwarn("[Mode4] STABLE_CHECK 중 태그 놓침 -> Yaw 재정렬부터 재시도")
                self.start_search("YAW_ALIGN_CAPTURE")

        elif self.state == "DONE":
            self.stop_robot()

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = Mode4AlignNode()
        node.run()
    except rospy.ROSInterruptException:
        pass