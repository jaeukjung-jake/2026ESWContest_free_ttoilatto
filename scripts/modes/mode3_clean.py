#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
mode3_clean_node

변기 청소 작업을 수행하는 Mode3 노드.

동작 순서:
    1. /robot_mode == "MODE3_CLEAN" 대기
    2. Hose1 replay 실행
    3. Hose1 정상 종료
    4. /pump_cmd 로 "PUMP_ON" 발행
    5. 2초 동안 물 분사
    6. /pump_cmd 로 "PUMP_OFF" 발행
    7. Hose2 replay 실행
    8. Hose2 정상 종료
    9. Brush replay 실행
    10. Brush replay 정상 종료
    11. /mode3_status 로 MODE3_CLEAN_DONE:<stall_id> 발행

ROS Melodic 노드는 Python 2.7에서 실행하고,
SO-100 replay 코드는 Python 3.8 subprocess로 별도 실행한다.
"""

import subprocess
import threading

import rospy
from std_msgs.msg import String


# ============================================================
# 경로 설정
# ============================================================

PYTHON3 = "/usr/bin/python3.8"

HOSE1_REPLAY = (
    "/home/gungoose/catkin_ws/src/final_git/"
    "standalone_hose1_replay_jetson.py"
)

HOSE2_REPLAY = (
    "/home/gungoose/catkin_ws/src/final_git/"
    "standalone_hose2_replay_jetson.py"
)

BRUSH_REPLAY = (
    "/home/gungoose/catkin_ws/src/final_git/"
    "standalone_brush_replay_jetson.py"
)


# ============================================================
# 물 분사 설정
# ============================================================

# PUMP_ON 이후 PUMP_OFF까지의 시간
WATER_WAIT_TIME = 2.0


class Mode3CleanNode(object):

    def __init__(self):

        rospy.init_node("mode3_clean_node")

        # 현재 stall ID
        self.current_stall_id = None

        # 현재 Mode3 작업 중인지 확인
        self.cleaning = False

        # 동일한 MODE3_CLEAN 메시지에 의해
        # 청소가 두 번 실행되는 것을 방지
        self.completed_for_current_activation = False

        # ----------------------------------------------------
        # Mode3 상태 발행
        # ----------------------------------------------------

        self.status_pub = rospy.Publisher(
            "/mode3_status",
            String,
            queue_size=10
        )

        # ----------------------------------------------------
        # 물 펌프 제어 명령 발행
        #
        # PUMP_ON
        # PUMP_OFF
        # ----------------------------------------------------

        self.pump_pub = rospy.Publisher(
            "/pump_cmd",
            String,
            queue_size=10
        )

        # ----------------------------------------------------
        # 전체 시스템 모드 구독
        # ----------------------------------------------------

        rospy.Subscriber(
            "/robot_mode",
            String,
            self.mode_callback
        )

        # ----------------------------------------------------
        # 현재 stall ID 확인
        # ----------------------------------------------------

        rospy.Subscriber(
            "/mode1_status",
            String,
            self.mode1_status_callback
        )

        rospy.loginfo("[Mode3] mode3_clean_node started.")


    # ========================================================
    # Stall ID 저장
    # ========================================================

    def mode1_status_callback(self, msg):

        data = msg.data

        if data.startswith("MODE1_STALL_REACHED:"):

            try:

                self.current_stall_id = data.split(":", 1)[1]

                rospy.loginfo(
                    "[Mode3] Current stall ID = %s",
                    self.current_stall_id
                )

            except Exception:

                rospy.logwarn(
                    "[Mode3] Failed to parse stall ID: %s",
                    data
                )


    # ========================================================
    # /robot_mode callback
    # ========================================================

    def mode_callback(self, msg):

        mode = msg.data

        if mode == "MODE3_CLEAN":

            if self.cleaning:

                rospy.logwarn(
                    "[Mode3] Cleaning already running. "
                    "Duplicate MODE3_CLEAN ignored."
                )

                return

            if self.completed_for_current_activation:
                return

            rospy.loginfo(
                "[Mode3] MODE3_CLEAN received. "
                "Starting cleaning sequence."
            )

            self.cleaning = True

            # Subscriber callback을 block하지 않도록
            # 별도 thread에서 청소 sequence 실행
            thread = threading.Thread(
                target=self.run_cleaning_sequence
            )

            thread.daemon = True
            thread.start()

        else:

            # MODE3에서 빠져나간 뒤
            # 다음 변기칸을 위해 reset
            if not self.cleaning:
                self.completed_for_current_activation = False


    # ========================================================
    # Replay 실행
    # ========================================================

    def run_replay(self, script_path, name):

        rospy.loginfo(
            "[Mode3] Starting %s replay.",
            name
        )

        try:

            process = subprocess.Popen(
                [
                    PYTHON3,
                    script_path
                ]
            )

            # 해당 replay가 완전히 종료될 때까지 기다림
            return_code = process.wait()

        except Exception as e:

            rospy.logerr(
                "[Mode3] Failed to execute %s replay: %s",
                name,
                str(e)
            )

            return False

        if return_code != 0:

            rospy.logerr(
                "[Mode3] %s replay exited with code %d.",
                name,
                return_code
            )

            return False

        rospy.loginfo(
            "[Mode3] %s replay completed.",
            name
        )

        return True


    # ========================================================
    # 펌프 제어
    # ========================================================

    def pump_on(self):

        rospy.loginfo(
            "[Mode3] PUMP ON"
        )

        self.pump_pub.publish(
            String(data="PUMP_ON")
        )


    def pump_off(self):

        rospy.loginfo(
            "[Mode3] PUMP OFF"
        )

        self.pump_pub.publish(
            String(data="PUMP_OFF")
        )


    # ========================================================
    # 전체 청소 sequence
    # ========================================================

    def run_cleaning_sequence(self):

        try:

            rospy.loginfo(
                "[Mode3] ========================================"
            )

            rospy.loginfo(
                "[Mode3] CLEANING START - stall %s",
                str(self.current_stall_id)
            )

            rospy.loginfo(
                "[Mode3] ========================================"
            )


            # =================================================
            # 1. HOSE1
            #
            # 호스를 분사 위치까지 이동
            # 정상 종료 후 Torque ON 상태 유지
            # =================================================

            hose1_ok = self.run_replay(
                HOSE1_REPLAY,
                "HOSE1"
            )

            if not hose1_ok:

                rospy.logerr(
                    "[Mode3] Hose1 replay failed. "
                    "Cleaning sequence aborted."
                )

                return

            rospy.loginfo(
                "[Mode3] Hose1 completed."
            )


            # =================================================
            # 2. 물 분사
            #
            # PUMP_ON
            #     ↓
            # 2초 대기
            #     ↓
            # PUMP_OFF
            # =================================================

            self.pump_on()

            rospy.loginfo(
                "[Mode3] Spraying water for %.1f sec.",
                WATER_WAIT_TIME
            )

            rospy.sleep(
                WATER_WAIT_TIME
            )

            self.pump_off()

            rospy.loginfo(
                "[Mode3] Water spraying completed."
            )

            if rospy.is_shutdown():
                return


            # =================================================
            # 3. HOSE2
            #
            # 현재 실제 관절 위치 읽기
            #       ↓
            # Hose2 첫 자세까지 Cosine 보간
            #       ↓
            # Hose2 replay
            # =================================================

            hose2_ok = self.run_replay(
                HOSE2_REPLAY,
                "HOSE2"
            )

            if not hose2_ok:

                rospy.logerr(
                    "[Mode3] Hose2 replay failed. "
                    "Cleaning sequence aborted."
                )

                return


            # =================================================
            # 4. Hose -> Brush 안정화
            # =================================================

            rospy.sleep(1.0)

            if rospy.is_shutdown():
                return


            # =================================================
            # 5. BRUSH
            # =================================================

            brush_ok = self.run_replay(
                BRUSH_REPLAY,
                "BRUSH"
            )

            if not brush_ok:

                rospy.logerr(
                    "[Mode3] Brush replay failed. "
                    "Cleaning sequence aborted."
                )

                return


            # =================================================
            # 6. 청소 완료
            # =================================================

            stall_id = (
                self.current_stall_id
                if self.current_stall_id is not None
                else "unknown"
            )

            status = "MODE3_CLEAN_DONE:%s" % stall_id

            self.status_pub.publish(
                String(data=status)
            )

            self.completed_for_current_activation = True

            rospy.loginfo(
                "[Mode3] Cleaning completed."
            )

            rospy.loginfo(
                "[Mode3] Published: %s",
                status
            )


        except Exception as e:

            # 예외가 발생하면 펌프가 ON 상태로
            # 남지 않도록 OFF 명령을 한번 보냄
            try:
                self.pump_off()
            except Exception:
                pass

            rospy.logerr(
                "[Mode3] Cleaning sequence error: %s",
                str(e)
            )


        finally:

            self.cleaning = False


    # ========================================================
    # RUN
    # ========================================================

    def run(self):

        rospy.spin()


if __name__ == "__main__":

    try:

        node = Mode3CleanNode()
        node.run()

    except rospy.ROSInterruptException:

        pass