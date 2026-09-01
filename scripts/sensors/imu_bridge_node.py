#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
imu_bridge_node

imusensor.py(python3.8 venv, adafruit_bno08x)가 UDP로 쏘는 JSON을 받아서
sensor_msgs/Imu 형식으로 /imu/data 에 발행하는 노드.

- 이 노드 자체는 rospy만 쓰고 adafruit 라이브러리는 전혀 몰라도 됨
  -> 일반 ROS 파이썬 환경(rospy가 이미 동작하는 인터프리터)에서 그대로 실행 가능
  -> venv에 rospy 바인딩을 억지로 설치할 필요가 없어짐 (센서 판독은 imusensor.py가,
     ROS 발행은 이 노드가 나눠서 담당)
- imusensor.py는 계속 python3.8 venv에서 UDP 송신 전용 프로세스로 그대로 실행
"""
from __future__ import print_function  # Python 2/3 print 함수 호환성

import json
import socket

import rospy
from sensor_msgs.msg import Imu


class ImuBridgeNode(object):

    def __init__(self):
        rospy.init_node("imu_bridge_node", anonymous=True)

        # ---------------- 파라미터 ----------------
        self.udp_ip = rospy.get_param("~udp_ip", "127.0.0.1")
        self.udp_port = rospy.get_param("~udp_port", 5005)
        self.socket_timeout = rospy.get_param("~socket_timeout", 0.2)  # 초
        self.frame_id = rospy.get_param("~frame_id", "imu_link")
        self.verbose = rospy.get_param("~verbose", False)  # 터미널 실시간 출력 여부

        # ---------------- UDP 소켓 ----------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(self.socket_timeout)  # 무한 대기 방지 -> shutdown 신호가 바로 먹히게

        # ---------------- Publisher ----------------
        self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=10)

        rospy.loginfo("[ImuBridge] 노드 시작. UDP %s:%d 대기 중...", self.udp_ip, self.udp_port)

    def build_imu_msg(self, raw):
        acc = raw["acc"]
        gyro = raw["gyro"]
        q = raw["quat"]

        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc

        # 실측 분산 값 (대각선 요소만 사용)
        msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]  # 자세는 BNO08x 내부 필터 사용, 작은 값 유지
        msg.angular_velocity_covariance = [0.000001, 0, 0, 0, 0.000001, 0, 0, 0, 0.000001]
        msg.linear_acceleration_covariance = [0.000211, 0, 0, 0, 0.000057, 0, 0, 0, 0.000290]

        return msg

    def print_debug(self, raw):
        acc, gyro, q, stab = raw["acc"], raw["gyro"], raw["quat"], raw.get("stab", "")
        output = (
            "\033[H"
            "\033[K[ROS 1] Acc : [%6.2f, %6.2f, %6.2f]\n"
            "\033[K[ROS 2] Gyro: [%6.2f, %6.2f, %6.2f] | Quat : [%5.2f, %5.2f, %5.2f, %5.2f] | S:%s"
        ) % (
            acc[0], acc[1], acc[2],
            gyro[0], gyro[1], gyro[2], q[0], q[1], q[2], q[3], str(stab),
        )
        print(output, end="", flush=True)

    def run(self):
        if self.verbose:
            print("\033[H\033[J", end="", flush=True)

        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                # UDP가 잠깐 끊긴 상태. 여기서 계속 대기하되 rospy.is_shutdown()을
                # 주기적으로 다시 체크할 수 있게 넘어감 (무한 블로킹 방지)
                continue

            try:
                raw = json.loads(data.decode())
                msg = self.build_imu_msg(raw)
            # Python 2.7의 json 모듈에는 JSONDecodeError가 없으므로 ValueError로 처리
            except (KeyError, ValueError) as e:
                # 패킷 하나가 깨져도 노드 전체를 죽이지 않고 다음 패킷을 기다림
                rospy.logwarn_throttle(1.0, "[ImuBridge] 패킷 파싱 실패: %s", e)
                continue

            self.imu_pub.publish(msg)

            if self.verbose:
                self.print_debug(raw)

        rospy.loginfo("[ImuBridge] 노드 종료")


if __name__ == "__main__":
    try:
        node = ImuBridgeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass