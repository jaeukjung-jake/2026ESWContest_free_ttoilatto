# -*- coding: utf-8 -*-

import csv
import time
import sys

import scservo_sdk as scs


# ============================================================
# 설정
# ============================================================

PORT = "/dev/ttyACM1"
BAUDRATE = 1000000

CSV_PATH = "/home/gungoose/catkin_ws/src/final_git/toilet_brush_replay_new_raw.csv"

FPS = 30.0
PERIOD = 1.0 / FPS


# ============================================================
# STS3215 Control Table
# LeRobot tables.py 기준
# ============================================================

ADDR_P_COEFFICIENT = 21
LEN_P_COEFFICIENT = 1

ADDR_OPERATING_MODE = 33
LEN_OPERATING_MODE = 1

ADDR_TORQUE_ENABLE = 40
LEN_TORQUE_ENABLE = 1

ADDR_GOAL_POSITION = 42
LEN_GOAL_POSITION = 2


# ============================================================
# 설정값
# ============================================================

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

POSITION_MODE = 0

# LeRobot SOFollower.configure()와 동일하게 사용
P_COEFFICIENT = 16


# ============================================================
# Motor IDs
# ============================================================

MOTORS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


CSV_COLUMNS = {
    "shoulder_pan": "shoulder_pan_raw",
    "shoulder_lift": "shoulder_lift_raw",
    "elbow_flex": "elbow_flex_raw",
    "wrist_flex": "wrist_flex_raw",
    "wrist_roll": "wrist_roll_raw",
    "gripper": "gripper_raw",
}


# ============================================================
# CSV 읽기
# ============================================================

def load_csv(path):

    frames = []

    with open(path, "r", newline="") as f:

        reader = csv.DictReader(f)

        for row in reader:

            frame = {
                "frame": int(row["frame"]),
                "time": float(row["time"]),
            }

            for motor_name, column_name in CSV_COLUMNS.items():
                frame[motor_name] = int(row[column_name])

            frames.append(frame)

    return frames


# ============================================================
# SDK 통신 결과 확인
# ============================================================

def check_comm(packet_handler, comm_result, error, description):

    if comm_result != scs.COMM_SUCCESS:
        print(
            "[ERROR] {} : {}".format(
                description,
                packet_handler.getTxRxResult(comm_result)
            )
        )
        return False

    if error != 0:
        print(
            "[ERROR] {} : {}".format(
                description,
                packet_handler.getRxPacketError(error)
            )
        )
        return False

    return True


# ============================================================
# 1 byte register write
# ============================================================

def write_1byte(port_handler, packet_handler, motor_id, address, value):

    comm_result, error = packet_handler.write1ByteTxRx(
        motor_id,
        address,
        value
    )

    return check_comm(
        packet_handler,
        comm_result,
        error,
        "motor {} write address {}".format(motor_id, address)
    )


# ============================================================
# 모터 설정
# ============================================================

def configure_motors(port_handler, packet_handler):

    print()
    print("Configuring motors...")

    # --------------------------------------------------------
    # 먼저 Torque OFF
    # --------------------------------------------------------

    for name, motor_id in MOTORS.items():

        if not write_1byte(
            port_handler,
            packet_handler,
            motor_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE
        ):
            raise RuntimeError(
                "Failed to disable torque: {}".format(name)
            )

    print("Torque disabled.")

    # --------------------------------------------------------
    # Position Mode
    # --------------------------------------------------------

    for name, motor_id in MOTORS.items():

        if not write_1byte(
            port_handler,
            packet_handler,
            motor_id,
            ADDR_OPERATING_MODE,
            POSITION_MODE
        ):
            raise RuntimeError(
                "Failed to set position mode: {}".format(name)
            )

    print("Position mode configured.")

    # --------------------------------------------------------
    # P coefficient = 16
    # --------------------------------------------------------

    for name, motor_id in MOTORS.items():

        if not write_1byte(
            port_handler,
            packet_handler,
            motor_id,
            ADDR_P_COEFFICIENT,
            P_COEFFICIENT
        ):
            raise RuntimeError(
                "Failed to set P coefficient: {}".format(name)
            )

    print("P coefficient = 16.")

    # --------------------------------------------------------
    # Torque ON
    # --------------------------------------------------------

    for name, motor_id in MOTORS.items():

        if not write_1byte(
            port_handler,
            packet_handler,
            motor_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE
        ):
            raise RuntimeError(
                "Failed to enable torque: {}".format(name)
            )

    print("Torque enabled.")


# ============================================================
# Torque OFF
# ============================================================

def disable_torque(port_handler, packet_handler):

    print()
    print("Disabling torque...")

    for name, motor_id in MOTORS.items():

        try:
            write_1byte(
                port_handler,
                packet_handler,
                motor_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE
            )
        except Exception:
            pass


# ============================================================
# GroupSyncWrite
# ============================================================

def send_positions(group_sync_write, packet_handler, frame):

    group_sync_write.clearParam()

    for motor_name, motor_id in MOTORS.items():

        value = frame[motor_name]

        # Goal_Position = 2 byte
        param_goal_position = [
            packet_handler.scs_lobyte(value),
            packet_handler.scs_hibyte(value),
        ]

        success = group_sync_write.addParam(
            motor_id,
            param_goal_position
        )

        if not success:
            raise RuntimeError(
                "GroupSyncWrite addParam failed: {}".format(
                    motor_name
                )
            )

    comm_result = group_sync_write.txPacket()

    if comm_result != scs.COMM_SUCCESS:
        raise RuntimeError(
            "GroupSyncWrite failed with communication result {}".format(
                comm_result
            )
        )


# ============================================================
# MAIN
# ============================================================

def main():

    print("========================================")
    print(" SO-100 Standalone RAW CSV Replay")
    print("========================================")
    print()
    print("LeRobot framework is NOT used.")
    print()

    # --------------------------------------------------------
    # CSV 로딩
    # --------------------------------------------------------

    frames = load_csv(CSV_PATH)

    if not frames:
        print("CSV contains no frames.")
        sys.exit(1)

    print("CSV:", CSV_PATH)
    print("Frames:", len(frames))
    print("Duration: {:.2f} sec".format(len(frames) / FPS))

    print()
    print("First RAW frame:")

    for name in MOTORS:
        print(
            "  {:15s} {}".format(
                name,
                frames[0][name]
            )
        )

    # --------------------------------------------------------
    # Port / Packet Handler
    # --------------------------------------------------------

    port_handler = scs.PortHandler(PORT)

    # LeRobot Feetech default protocol = 0
    packet_handler = scs.sms_sts(port_handler)

    # --------------------------------------------------------
    # COM Port Open
    # --------------------------------------------------------

    print()
    print("Opening {}...".format(PORT))

    if not port_handler.openPort():
        print("ERROR: Failed to open {}".format(PORT))
        sys.exit(1)

    print("{} opened.".format(PORT))

    # --------------------------------------------------------
    # Baudrate
    # --------------------------------------------------------

    if not port_handler.setBaudRate(BAUDRATE):
        print("ERROR: Failed to set baudrate.")
        port_handler.closePort()
        sys.exit(1)

    print("Baudrate:", BAUDRATE)

    # --------------------------------------------------------
    # GroupSyncWrite 생성
    # --------------------------------------------------------

    group_sync_write = scs.GroupSyncWrite(
        packet_handler,
        ADDR_GOAL_POSITION,
        LEN_GOAL_POSITION
    )

    try:

        # ----------------------------------------------------
        # 모터 설정
        # ----------------------------------------------------

        configure_motors(
            port_handler,
            packet_handler
        )

        # ----------------------------------------------------
        # 안전 카운트다운
        # ----------------------------------------------------

        print()
        print("========================================")
        print(" WARNING")
        print(" Robot will start moving.")
        print(" Keep emergency stop / power ready.")
        print("========================================")
        print()

        for i in range(5, 0, -1):
            print("Starting in {}...".format(i))
            time.sleep(1)

        print()
        print("REPLAY START")
        print()

        replay_start = time.perf_counter()

        # ----------------------------------------------------
        # Replay
        # ----------------------------------------------------

        for index, frame in enumerate(frames):

            # CSV의 timestamp를 기준으로 목표 전송 시각 계산
            target_time = replay_start + frame["time"]

            # 목표 시간이 될 때까지 대기
            while True:

                remaining = target_time - time.perf_counter()

                if remaining <= 0:
                    break

                # 너무 긴 busy-wait 방지
                if remaining > 0.002:
                    time.sleep(remaining - 0.001)

            # 6개 모터 동시 전송
            send_positions(
                group_sync_write,
                packet_handler,
                frame
            )

            # 1초마다 진행상황 출력
            if index % 30 == 0:

                elapsed = time.perf_counter() - replay_start

                print(
                    "frame {:4d}/{:4d} | "
                    "CSV {:6.2f}s | "
                    "real {:6.2f}s".format(
                        index,
                        len(frames),
                        frame["time"],
                        elapsed
                    )
                )

        print()
        print("REPLAY FINISHED")

    except KeyboardInterrupt:

        print()
        print("Replay stopped by user.")

    except Exception as e:

        print()
        print("ERROR:")
        print(e)

    finally:

        # ----------------------------------------------------
        # 종료
        # ----------------------------------------------------

        disable_torque(
            port_handler,
            packet_handler
        )

        port_handler.closePort()

        print("Port closed.")
        print("Done.")


if __name__ == "__main__":
    main()