#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
pump_control_node

/pump_cmd 를 구독해서 젯슨 나노 GPIO로 릴레이(JQC-3FF-S-Z 모듈)를 직접 제어,
워터펌프를 켜고 끄는 노드. (mode3_clean_node가 /pump_cmd 를 발행함)

- 구독: /pump_cmd (std_msgs/String)
    "PUMP_ON"  -> 릴레이 ON  (active_high=True 기준 GPIO HIGH)
    "PUMP_OFF" -> 릴레이 OFF (GPIO LOW)

- 안전장치:
    PUMP_ON 이후 max_on_duration 초가 지나도 PUMP_OFF가 오지 않으면
    (상위 노드 다운, 메시지 유실 등) 자동으로 펌프를 꺼서 침수를 방지한다.

배선:
    릴레이 IN  -> 젯슨 GPIO (BOARD 번호, 기본 31 / GPIO06)
    릴레이 VCC -> 젯슨 5V, 릴레이 GND -> 젯슨 GND
    워터펌프 전원은 릴레이 COM/NO 접점을 통해 별도 외부 전원에서 공급한다
    (젯슨 5V 핀에서 직접 끌어오지 않음).
"""

import rospy
import Jetson.GPIO as GPIO

from std_msgs.msg import String


class PumpControlNode(object):

    def __init__(self):
        rospy.init_node("pump_control_node")

        # ---------------- 파라미터 ----------------
        self.pin = rospy.get_param("~pin", 31)  # BOARD 번호 (GPIO06)
        self.active_high = rospy.get_param("~active_high", True)
        self.max_on_duration = rospy.get_param("~max_on_duration", 10.0)  # 안전 자동 OFF (초)

        self.on_level = GPIO.HIGH if self.active_high else GPIO.LOW
        self.off_level = GPIO.LOW if self.active_high else GPIO.HIGH

        # ---------------- GPIO 초기화 ----------------
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=self.off_level)

        self.pump_is_on = False
        self.safety_timer = None

        # ---------------- Subscriber ----------------
        rospy.Subscriber("/pump_cmd", String, self.pump_cmd_callback)

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo(
            "[PumpControl] pump_control_node started. pin=%d(BOARD), "
            "active_high=%s, max_on_duration=%.1fs",
            self.pin, self.active_high, self.max_on_duration
        )

    # ==================== Callbacks ====================
    def pump_cmd_callback(self, msg):
        cmd = msg.data

        if cmd == "PUMP_ON":
            self.pump_on()
        elif cmd == "PUMP_OFF":
            self.pump_off()
        else:
            rospy.logwarn("[PumpControl] Unknown /pump_cmd: '%s'", cmd)

    # ==================== Pump control ====================
    def pump_on(self):
        GPIO.output(self.pin, self.on_level)
        self.pump_is_on = True
        rospy.loginfo("[PumpControl] PUMP ON")

        # 이전 안전 타이머가 남아있으면 취소하고 새로 시작
        if self.safety_timer is not None:
            self.safety_timer.shutdown()

        self.safety_timer = rospy.Timer(
            rospy.Duration(self.max_on_duration),
            self.safety_timeout_callback,
            oneshot=True
        )

    def pump_off(self):
        GPIO.output(self.pin, self.off_level)
        self.pump_is_on = False
        rospy.loginfo("[PumpControl] PUMP OFF")

        if self.safety_timer is not None:
            self.safety_timer.shutdown()
            self.safety_timer = None

    def safety_timeout_callback(self, event):
        if self.pump_is_on:
            rospy.logwarn(
                "[PumpControl] PUMP_OFF not received within %.1fs. "
                "Forcing pump OFF for safety.",
                self.max_on_duration
            )
            self.pump_off()

    # ==================== Shutdown ====================
    def shutdown_hook(self):
        try:
            GPIO.output(self.pin, self.off_level)
        except Exception:
            pass
        GPIO.cleanup()
        rospy.loginfo("[PumpControl] Node shutting down, pump forced OFF, GPIO cleaned up.")

    # ==================== Main loop ====================
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = PumpControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
