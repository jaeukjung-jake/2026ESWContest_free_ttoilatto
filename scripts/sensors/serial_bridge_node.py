#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
serial_bridge_node

아두이노와 시리얼로 직접 통신하는 유일한 노드.

- 구독: /cmd_vel (geometry_msgs/Twist)
    -> "v,w\n" 문자열로 변환해서 시리얼로 전송
    -> 아두이노는 자체 엔코더로 모터 속도를 폐루프(PID) 제어함
       (이 노드는 그 내부 제어 로직을 몰라도 됨, v/w만 던져주면 됨)
- 발행: /encoder_ticks (std_msgs/Int32MultiArray, data=[dL, dR])
    -> 아두이노가 "dL,dR\n" 형식으로 보내주는 엔코더 틱 증분값을 파싱해서 발행
    -> odom_node가 이 값을 구독해서 위치 추정(dead reckoning)에 사용

시리얼 프로토콜은 실제 하드웨어 테스트 코드와 동일하게 맞춰둠:
    PC -> Arduino : "{v:.3f},{w:.3f}\n"
    Arduino -> PC : "{dL},{dR}\n"
아두이노 펌웨어 프로토콜이 다르면 write_velocity() / read_encoder_ticks()의
파싱-포맷 부분만 고치면 됨.
"""

import rospy
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class SerialBridgeNode(object):

    def __init__(self):
        rospy.init_node("serial_bridge_node")

        # ---------------- 파라미터 ----------------
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.serial_timeout = rospy.get_param("~serial_timeout", 0.1)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)  # /cmd_vel 끊김 판단 기준(초)

        # ---------------- 시리얼 연결 ----------------
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.serial_timeout)
            rospy.loginfo("[SerialBridge] 시리얼 연결 성공: %s @ %d", self.port, self.baudrate)
        except Exception as e:
            rospy.logerr("[SerialBridge] 시리얼 연결 실패: %s", e)
            raise

        # ---------------- 상태 변수 ----------------
        self.last_cmd_time = rospy.Time.now()
        self.last_v = 0.0
        self.last_w = 0.0

        # ---------------- Publisher / Subscriber ----------------
        self.tick_pub = rospy.Publisher("/encoder_ticks", Int32MultiArray, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.rate = rospy.Rate(50)

    # ==================== Callbacks ====================
    def cmd_vel_callback(self, msg):
        self.last_v = msg.linear.x
        self.last_w = msg.angular.z
        self.last_cmd_time = rospy.Time.now()
        self.write_velocity(self.last_v, self.last_w)

    # ==================== Serial I/O ====================
    def write_velocity(self, v, w):
        try:
            line = "%.3f,%.3f\n" % (v, w)
            self.ser.write(line.encode())
        except Exception as e:
            rospy.logwarn_throttle(1.0, "[SerialBridge] 시리얼 write 실패: %s", e)

    def read_encoder_ticks(self):
        """시리얼 입력 버퍼에 쌓인 라인을 모두 읽어서 dL, dR 합산값을 반환.
        (한 루프 주기 동안 여러 줄이 들어올 수 있으므로 합산 -> 유실 방지)
        """
        dl_sum, dr_sum = 0, 0
        got_data = False

        while self.ser.in_waiting > 0:
            line = ""
            try:
                line = self.ser.readline().decode().strip()
                if "," in line:
                    dl, dr = map(int, line.split(","))
                    dl_sum += dl
                    dr_sum += dr
                    got_data = True
            except Exception as e:
                rospy.logwarn_throttle(1.0, "[SerialBridge] 인코더 라인 파싱 실패: '%s' (%s)", line, e)

        return got_data, dl_sum, dr_sum

    # ==================== Safety ====================
    def check_cmd_timeout(self):
        # /cmd_vel 이 끊기면(상위 노드 다운 등) 안전을 위해 정지 명령을 계속 전송
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
            if self.last_v != 0.0 or self.last_w != 0.0:
                rospy.logwarn_throttle(1.0, "[SerialBridge] /cmd_vel 끊김 감지 -> 정지 명령 전송")
            self.last_v, self.last_w = 0.0, 0.0
            self.write_velocity(0.0, 0.0)

    # ==================== Main loop ====================
    def run(self):
        while not rospy.is_shutdown():
            self.check_cmd_timeout()

            got_data, dl, dr = self.read_encoder_ticks()
            if got_data:
                msg = Int32MultiArray()
                msg.data = [dl, dr]
                self.tick_pub.publish(msg)

            self.rate.sleep()

        # 노드 종료 시 안전 정지 (유실 대비 2번 전송)
        self.write_velocity(0.0, 0.0)
        self.write_velocity(0.0, 0.0)


if __name__ == "__main__":
    try:
        node = SerialBridgeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass