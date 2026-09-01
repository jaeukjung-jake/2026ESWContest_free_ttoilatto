#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
lidar_node.py

LiDAR 원본 스캔
    /scan (sensor_msgs/LaserScan)

을 받아 로봇 기준 전방 180도(-90 ~ +90도)의 포인트만 추출한다.

각 LiDAR 측정값을 로봇 기준 XY 좌표로 변환하여
Float32MultiArray 형태로 발행한다.

좌표계:
    +X : 로봇 정면
    +Y : 로봇 왼쪽
    -Y : 로봇 오른쪽

발행 토픽:
    /lidar/front_points

메시지 형식:
    std_msgs/Float32MultiArray

    data = [
        x0, y0,
        x1, y1,
        x2, y2,
        ...
    ]

단위:
    meter

현재 LiDAR는 로봇 기준으로 180도 반대로 장착되어 있으므로
기본 angle_offset_deg = 180.0 으로 설정한다.
"""

import math
import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarFrontPointsNode(object):

    def __init__(self):

        rospy.init_node("lidar_node")

        # ============================================================
        # Parameter
        # ============================================================

        self.scan_topic = rospy.get_param(
            "~scan_topic",
            "/scan"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/lidar/front_points"
        )

        # 로봇 정면 기준 ±90도
        # → 총 전방 180도
        self.front_half_angle = math.radians(
            rospy.get_param(
                "~front_half_angle_deg",
                90.0
            )
        )

        # 사용할 거리 범위
        self.min_range = rospy.get_param(
            "~min_range",
            0.03
        )

        self.max_range = rospy.get_param(
            "~max_range",
            5.0
        )

        # ============================================================
        # LiDAR 장착 방향 보정
        #
        # 현재 LiDAR가 로봇 정면과 180도 반대로 장착되어 있음.
        #
        # 따라서:
        # 센서 기준 180도
        #       ↓
        # 로봇 기준 0도(정면)
        #
        # 기본값 = 180도
        # ============================================================

        self.angle_offset = math.radians(
            rospy.get_param(
                "~angle_offset_deg",
                180.0
            )
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.pub = rospy.Publisher(
            self.output_topic,
            Float32MultiArray,
            queue_size=1
        )

        # ============================================================
        # Subscriber
        # ============================================================

        rospy.Subscriber(
            self.scan_topic,
            LaserScan,
            self.scan_callback,
            queue_size=1
        )

        rospy.loginfo(
            "[LidarNode] started"
        )

        rospy.loginfo(
            "[LidarNode] input  : %s",
            self.scan_topic
        )

        rospy.loginfo(
            "[LidarNode] output : %s",
            self.output_topic
        )

        rospy.loginfo(
            "[LidarNode] front FOV : %.1f deg",
            math.degrees(self.front_half_angle) * 2.0
        )

        rospy.loginfo(
            "[LidarNode] angle offset : %.1f deg",
            math.degrees(self.angle_offset)
        )

    # ================================================================
    # LaserScan Callback
    # ================================================================

    def scan_callback(self, msg):

        flat = []

        angle = msg.angle_min

        for r in msg.ranges:

            # --------------------------------------------------------
            # 센서 기준 각도 → 로봇 기준 각도
            # --------------------------------------------------------

            robot_angle = self.normalize_angle(
                angle + self.angle_offset
            )

            angle += msg.angle_increment

            # --------------------------------------------------------
            # 잘못된 거리값 제거
            # --------------------------------------------------------

            if math.isnan(r):
                continue

            if math.isinf(r):
                continue

            # LaserScan 자체 측정 범위도 고려
            effective_min_range = max(
                self.min_range,
                msg.range_min
            )

            effective_max_range = min(
                self.max_range,
                msg.range_max
            )

            if r < effective_min_range:
                continue

            if r > effective_max_range:
                continue

            # --------------------------------------------------------
            # 로봇 기준 전방 ±90도만 사용
            # --------------------------------------------------------

            if abs(robot_angle) > self.front_half_angle:
                continue

            # --------------------------------------------------------
            # Polar → Cartesian
            #
            # x : 로봇 전방
            # y : 로봇 왼쪽
            # --------------------------------------------------------

            x = r * math.cos(robot_angle)
            y = r * math.sin(robot_angle)

            flat.append(x)
            flat.append(y)

        # ============================================================
        # Publish
        # ============================================================

        out = Float32MultiArray()
        out.data = flat

        self.pub.publish(out)

        rospy.loginfo_throttle(
            1.0,
            "[LidarNode] front points: %d",
            len(flat) // 2
        )

    # ================================================================
    # Angle normalization
    # ================================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


# ====================================================================
# Main
# ====================================================================

if __name__ == "__main__":

    try:
        node = LidarFrontPointsNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass