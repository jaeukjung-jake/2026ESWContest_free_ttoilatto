#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
odom_node

엔코더 틱(/encoder_ticks) + IMU(/imu/data)를 받아서 로봇 위치(x, y, theta)를
dead reckoning 방식으로 계산하고, 표준 /odom (nav_msgs/Odometry) + TF(odom->base_link)로
발행하는 노드.

- IMU는 별도의 imu_node(Python3, UDP 등)가 sensor_msgs/Imu 형식으로 이미
  발행하고 있다고 가정 (파이썬 버전 문제는 그 노드 안에서 해결됨)
- 엔코더 틱은 serial_bridge_node가 std_msgs/Int32MultiArray([dL, dR])로
  발행하고 있다고 가정
- 실제 하드웨어 point-to-point 테스트 코드의 로직(초기 IMU 오프셋 보정,
  틱->거리 변환, dead reckoning 적분, IMU 끊김 안전 체크)을 그대로 이식함

주의: 이건 단순 dead reckoning이라 시간이 지날수록 오차가 누적됨. 나중에
정밀도가 더 필요하면 robot_localization 패키지(EKF)로 이 노드를 교체하는
것도 고려할 수 있음 -> 이 노드는 표준 /odom + TF만 발행하므로, 교체해도
mode1 등 상위 제어 노드는 코드를 손댈 필요가 없음.
"""

import math
import rospy
import tf

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class OdomNode(object):

    def __init__(self):
        rospy.init_node("odom_node")

        # ---------------- 파라미터 (하드웨어 스펙) ----------------
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.0415)   # m
        self.ticks_per_rev = rospy.get_param("~ticks_per_rev", 660)
        self.imu_timeout = rospy.get_param("~imu_timeout", 0.5)        # 초

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_rate = rospy.get_param("~publish_rate", 30)

        # ---------------- 상태 변수 ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.is_calibrated = False   # 첫 IMU 값으로 offset을 잡기 전까지 False
        self.offset_theta = 0.0

        self.last_imu_time = None

        # ---------------- Publisher / Subscriber ----------------
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()

        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/encoder_ticks", Int32MultiArray, self.encoder_callback)

        self.rate = rospy.Rate(self.publish_rate)

        rospy.loginfo("[Odom] 노드 시작. IMU 초기 보정 대기 중...")

    # ==================== Callbacks ====================
    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw_raw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.last_imu_time = rospy.Time.now()

        if not self.is_calibrated:
            # 로봇이 켜질 때의 방향을 0도로 잡는 초기 보정 (테스트 코드의 offset_theta와 동일)
            self.offset_theta = yaw_raw
            self.is_calibrated = True
            rospy.loginfo("[Odom] IMU 초기 보정 완료 (offset=%.1f deg)",
                          math.degrees(self.offset_theta))
            return

        self.theta = normalize_angle(yaw_raw - self.offset_theta)

    def encoder_callback(self, msg):
        if not self.is_calibrated:
            # 아직 방향(theta) 기준이 없으므로 이동량을 x,y에 반영하지 않고 건너뜀
            return

        if self.imu_is_stale():
            rospy.logwarn_throttle(1.0, "[Odom] IMU 끊김 상태 -> 엔코더 적분 보류")
            return

        if len(msg.data) < 2:
            return

        dl, dr = msg.data[0], msg.data[1]

        dist_l = 2.0 * math.pi * self.wheel_radius * (dl / float(self.ticks_per_rev))
        dist_r = 2.0 * math.pi * self.wheel_radius * (dr / float(self.ticks_per_rev))
        d_center = (dist_l + dist_r) / 2.0

        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

    # ==================== Safety ====================
    def imu_is_stale(self):
        if self.last_imu_time is None:
            return True
        return (rospy.Time.now() - self.last_imu_time).to_sec() > self.imu_timeout

    # ==================== Publish ====================
    def publish_odom(self):
        now = rospy.Time.now()
        q = quaternion_from_euler(0, 0, self.theta)

        # TF 발행 (odom -> base_link) : RViz 시각화, 다른 노드의 좌표 변환에 필요
        self.br.sendTransform(
            (self.x, self.y, 0.0),
            q,
            now,
            self.base_frame,
            self.odom_frame,
        )

        # /odom 메시지 발행
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*q)

        # 속도는 별도로 추정하지 않고 0으로 둠 (필요하면 dt 기반 근사치로 추가 가능)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

    def run(self):
        while not rospy.is_shutdown():
            if self.is_calibrated:
                self.publish_odom()
            else:
                rospy.loginfo_throttle(2.0, "[Odom] IMU 초기 보정 대기 중...")
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = OdomNode()
        node.run()
    except rospy.ROSInterruptException:
        pass