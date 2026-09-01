#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
mode_manager_node

전체 시스템의 상태머신(FSM). 각 모드 노드(mode1~4)는 자기 작업이 끝나면
상태 토픽에 완료 신호만 보내고, 실제로 "다음엔 어떤 모드로 바꿀지"는 전부
이 노드가 결정해서 /robot_mode 로 발행한다. 개별 모드 노드는 서로의 존재를
몰라도 되고, /robot_mode 가 자기 이름과 같을 때만 활성화되면 됨.

상태 흐름:
    MODE4_INIT --(MODE4_INIT_DONE)--> MODE1_NAV
    MODE1_NAV  --(MODE1_STALL_REACHED:<id>)--> MODE2_ENTER   (stall_id 기억)
    MODE2_ENTER --(MODE2_ENTER_DONE)--> MODE3_CLEAN
    MODE3_CLEAN --(MODE3_CLEAN_DONE)--> MODE2_EXIT
    MODE2_EXIT  --(MODE2_EXIT_DONE)--> MODE1_NAV   (mode1이 알아서 복귀회전 후 다음 웨이포인트로)
    MODE1_NAV  --(MODE1_ALL_WAYPOINTS_DONE)--> MISSION_DONE  (모든 변기칸 완료 + 원점 복귀)

이 노드가 기대하는 각 모드 노드의 상태 메시지 규약 (아직 안 만든 노드들은
반드시 이 문자열 그대로 맞춰서 발행해야 함):
    /mode1_status : "MODE1_STALL_REACHED:<id>", "MODE1_ALL_WAYPOINTS_DONE" (이미 구현됨)
    /mode2_status : "MODE2_ENTER_DONE:<id>", "MODE2_EXIT_DONE:<id>"
    /mode3_status : "MODE3_CLEAN_DONE:<id>"
    /mode4_status : "MODE4_INIT_DONE"
"""

import rospy
from std_msgs.msg import String


class ModeManagerNode(object):

    def __init__(self):
        rospy.init_node("mode_manager_node")

        # mode4(AprilTag 위치 초기화)를 아직 안 만들었으면 False로 두고
        # mode4를 같이 띄우고 싶다면 "rosrun final_git mode_manager.py _use_mode4_init:=true"을 사용한다.
        # mode4를 기본으로 사용하도록 True로 변경
        self.use_mode4_init = rospy.get_param("~use_mode4_init", False)
        
        # [테스트용] mode2,3가 없을 때 시뮬레이션을 위한 파라미터
        self.simulate_other_modes = rospy.get_param("~simulate_other_modes", False)
        self.simulation_delay = rospy.get_param("~simulation_delay", 2.0) # 초

        self.current_mode = "MODE4_INIT" if self.use_mode4_init else "MODE1_NAV"
        self.current_stall_id = None

        # /robot_mode 는 latch=True로 발행 -> 나중에 뜨는 노드도 마지막 모드값을 바로 받음
        self.mode_pub = rospy.Publisher("/robot_mode", String, queue_size=10, latch=True)

        # mode2,3에 현재 작업 대상 stall_id를 알려주기 위한 토픽 (latch=True)
        self.stall_id_pub = rospy.Publisher("/current_stall_id", String, queue_size=10, latch=True)

        rospy.Subscriber("/mode1_status", String, self.mode1_status_callback)
        rospy.Subscriber("/mode2_status", String, self.mode2_status_callback)
        rospy.Subscriber("/mode3_status", String, self.mode3_status_callback)
        rospy.Subscriber("/mode4_status", String, self.mode4_status_callback)

        # 구독자들이 연결될 시간을 살짝 준 뒤 초기 모드 발행 (latch라 늦게 붙어도 받긴 함)
        rospy.sleep(0.5)
        self.publish_mode(self.current_mode, reason="시스템 시작")

    # ==================== 상태 전환 ====================
    def publish_mode(self, new_mode, reason=""):
        prev_mode = self.current_mode
        self.current_mode = new_mode
        self.mode_pub.publish(String(data=new_mode))
        rospy.loginfo("[ModeManager] %s -> %s (%s)", prev_mode, new_mode, reason)

    def is_current(self, mode_name):
        """지금 이 콜백이 반응해야 할 모드가 맞는지 확인.
        늦게 도착한/중복된 상태 메시지 때문에 잘못된 전환이 일어나는 것을 방지."""
        return self.current_mode == mode_name

    # ==================== Mode1 (복도 주행) ====================
    def mode1_status_callback(self, msg):
        data = msg.data

        if not self.is_current("MODE1_NAV"):
            return  # mode1 차례가 아닐 때 온 메시지는 무시

        if data.startswith("MODE1_STALL_REACHED:"):
            stall_id = data.split(":")[1]
            self.current_stall_id = stall_id
            self.stall_id_pub.publish(String(data=stall_id)) # stall_id 발행
            self.publish_mode("MODE2_ENTER", reason="변기칸 %s 앞 도착+정렬 완료" % stall_id)
            
            # [테스트용] mode2/3 시뮬레이션 활성화 시, 일정 시간 후 강제로 다음 단계로 전환
            if self.simulate_other_modes:
                rospy.loginfo("[ModeManager] 테스트 모드: %.1f초 후 mode2/3 완료를 시뮬레이션합니다.", self.simulation_delay)
                # 타이머를 사용해 simulation_delay초 후에 _simulate_mode_completion 함수를 한 번만 실행
                rospy.Timer(rospy.Duration(self.simulation_delay), self._simulate_mode_completion, oneshot=True)


        elif data == "MODE1_ALL_WAYPOINTS_DONE":
            self.publish_mode("MISSION_DONE", reason="모든 웨이포인트 완료 + 원점 정렬 완료")

        # MODE1_MOVING, MODE1_OBSTACLE_STOP 등은 전환 없이 참고용으로만 흘려보냄

    # ==================== Mode2 (변기칸 진입/탈출) ====================
    def mode2_status_callback(self, msg):
        data = msg.data

        if data.startswith("MODE2_ENTER_DONE"):
            if not self.is_current("MODE2_ENTER"):
                return
            self.publish_mode("MODE3_CLEAN", reason="변기칸 %s 진입 완료" % self.current_stall_id)

        elif data.startswith("MODE2_EXIT_DONE"):
            if not self.is_current("MODE2_EXIT"):
                return
            self.publish_mode("MODE1_NAV", reason="변기칸 %s 탈출 완료" % self.current_stall_id)

    # ==================== Mode3 (청소) ====================
    def mode3_status_callback(self, msg):
        data = msg.data

        if data.startswith("MODE3_CLEAN_DONE"):
            if not self.is_current("MODE3_CLEAN"):
                return
            self.publish_mode("MODE2_EXIT", reason="변기칸 %s 청소 완료" % self.current_stall_id)

    # ==================== Mode4 (AprilTag 위치 초기화) ====================
    def mode4_status_callback(self, msg):
        data = msg.data

        if data == "MODE4_INIT_DONE":
            if not self.is_current("MODE4_INIT"):
                return
            self.publish_mode("MODE1_NAV", reason="위치 초기화 완료")

    # ==================== 테스트용 시뮬레이션 ====================
    def _simulate_mode_completion(self, event):
        """타이머에 의해 호출되는 함수. mode2/3가 완료된 것처럼 신호를 보낸다."""
        if self.current_mode == "MODE2_ENTER":
            rospy.loginfo("[ModeManager] 시뮬레이션: MODE2_ENTER -> MODE3_CLEAN -> MODE2_EXIT -> MODE1_NAV 전환 실행")
            # 실제로는 MODE2_EXIT_DONE 메시지를 mode2_status_callback이 받아야 하지만,
            # 테스트 목적 상 직접 다음 상태인 MODE1_NAV로 전환시킨다.
            self.publish_mode("MODE1_NAV", reason="변기칸 %s 작업 완료 (시뮬레이션)" % self.current_stall_id)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ModeManagerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass