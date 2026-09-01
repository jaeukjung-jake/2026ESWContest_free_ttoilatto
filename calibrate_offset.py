#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
calibrate_offset.py

로봇을 스테이션(초기 위치)에 최대한 반듯하게 놓은 뒤 1회 실행.
/apriltag/pose 를 2초간 구독해서 평균을 낸 raw x, y, z, yaw를
mode4의 "목표 정렬값(offset)"으로 확정하여 yaml 파일에 저장한다.

여기서는 좌표 변환(카메라 틸트 보정) 없이 raw 값만 저장한다.
-> 실제 바닥 기준 좌표로의 변환은 mode4_align_node.py 쪽에서,
   저장된 목표값과 실시간 값 양쪽에 '동일하게' 적용한다.
   (변환을 여기서 미리 해버리면 나중에 틸트 각도를 재보정할 때마다
    다시 캡처해야 하므로, raw로 남겨두는 게 더 유연함)

이후 mode4_align_node.py는 이 yaml만 읽어서 쓰고, 다시 캡처하지 않는다.
카메라 위치가 바뀌거나 스테이션을 옮기면 이 스크립트를 다시 실행해서
값을 갱신하면 된다.
"""

import os
import math
import yaml
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped

CAPTURE_DURATION = 2.0  # sec, 평균낼 구간
OUTPUT_PATH = os.path.expanduser("~/catkin_ws/src/final_git/param/mode4_offset.yaml")


def quat_to_yaw(q, tilt_rad=0.0):
    """카메라 좌표계(X right, Y down, Z forward) 기준 좌우 회전각(yaw) 추출.
    mode4_align_node.py에서도 반드시 이 함수와 '같은 정의'를 써야 함
    (tilt_rad 보정 포함, 그대로 복사해옴).
    """
    mat = tft.quaternion_matrix(q)
    r02 = mat[0][2]
    r12 = mat[1][2]
    r22 = mat[2][2]
    r22_ground = r12 * math.sin(tilt_rad) + r22 * math.cos(tilt_rad)
    return math.atan2(r02, r22_ground)


class OffsetCalibrator(object):
    def __init__(self):
        rospy.init_node("calibrate_offset")
        # mode4_align_node.py의 ~tilt_deg 기본값(55.0)과 반드시 동일해야 함
        # (다르면 yaw_offset이 mode4가 실시간 계산하는 값과 어긋나게 됨)
        self.tilt_rad = math.radians(rospy.get_param("~tilt_deg", 55.0))
        self.samples = []  # (x, y, z, yaw) raw 값 그대로 저장
        self.start_time = None
        self.sub = rospy.Subscriber("/apriltag/pose", PoseStamped, self.cb)
        rospy.loginfo("[Calib] 로봇이 스테이션에 반듯하게 놓여있는지 확인 후 대기 중...")
        rospy.loginfo("[Calib] %.1f초간 태그를 안정적으로 봐야 캡처가 완료됩니다.", CAPTURE_DURATION)

    def cb(self, msg):
        now = rospy.Time.now().to_sec()
        if self.start_time is None:
            self.start_time = now

        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        yaw = quat_to_yaw(q, self.tilt_rad)
        self.samples.append((x, y, z, yaw))

        elapsed = now - self.start_time
        if elapsed >= CAPTURE_DURATION:
            self.finish()

    def finish(self):
        self.sub.unregister()
        n = len(self.samples)
        if n == 0:
            rospy.logerr("[Calib] 샘플이 하나도 없습니다. 태그가 보이는 상태인지 확인하세요.")
            return

        xs = [s[0] for s in self.samples]
        ys = [s[1] for s in self.samples]
        zs = [s[2] for s in self.samples]
        yaws = [s[3] for s in self.samples]

        x_avg = sum(xs) / n
        y_avg = sum(ys) / n
        z_avg = sum(zs) / n
        yaw_avg = sum(yaws) / n  # 각도 랩어라운드는 여기선 값 범위가 작아 단순평균으로 충분

        x_std = (sum((v - x_avg) ** 2 for v in xs) / n) ** 0.5
        y_std = (sum((v - y_avg) ** 2 for v in ys) / n) ** 0.5
        z_std = (sum((v - z_avg) ** 2 for v in zs) / n) ** 0.5

        rospy.loginfo("[Calib] 캡처 완료 (샘플 %d개)", n)
        rospy.loginfo("[Calib] x=%.4f(std=%.4f) y=%.4f(std=%.4f) z=%.4f(std=%.4f) yaw=%.2fdeg",
                       x_avg, x_std, y_avg, y_std, z_avg, z_std, math.degrees(yaw_avg))

        if x_std > 0.01 or y_std > 0.01 or z_std > 0.01:
            rospy.logwarn("[Calib] 캡처 중 흔들림이 큽니다 (std > 1cm). "
                           "로봇/카메라가 완전히 정지한 상태였는지 다시 확인하고 재실행을 권장합니다.")

        data = {
            "x_offset": float(x_avg),
            "y_offset": float(y_avg),
            "z_offset": float(z_avg),
            "yaw_offset": float(yaw_avg),
        }

        out_dir = os.path.dirname(OUTPUT_PATH)
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)

        with open(OUTPUT_PATH, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False)

        rospy.loginfo("[Calib] 저장 완료: %s", OUTPUT_PATH)
        rospy.loginfo("[Calib] 이 값이 마음에 안 들면 로봇 위치를 다시 잡고 이 스크립트를 재실행하세요.")
        rospy.signal_shutdown("calibration done")


if __name__ == "__main__":
    try:
        OffsetCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass