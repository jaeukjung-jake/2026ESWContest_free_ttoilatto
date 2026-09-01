# -*- coding: utf-8 -*-

import csv
import time
import sys
import math

import scservo_sdk as scs


# ============================================================
# 설정
# ============================================================

PORT = "/dev/ttyACM1"
BAUDRATE = 1000000

CSV_PATH = (
    "/home/gungoose/catkin_ws/src/final_git/"
    "toilet_hose2_replay_new_raw.csv"
)

FPS = 30.0
PERIOD = 1.0 / FPS


# ============================================================
# Present Position / 보간 설정
# ============================================================

ADDR_PRESENT_POSITION = 56
LEN_PRESENT_POSITION = 2

# 현재 자세 -> Hose2 첫 자세
INTERPOLATION_DURATION = 2.0
INTERPOLATION_FPS = 30.0


# ============================================================
# STS3215 Control Table
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
# Jetson scservo_sdk 방식
# ============================================================

def write_1byte(port_handler, packet_handler, motor_id, address, value):

    # Jetson에서는 packet_handler가 이미 port_handler를 가지고 있음
    comm_result, error = packet_handler.write1ByteTxRx(
        motor_id,
        address,
        value
    )

    return check_comm(
        packet_handler,
        comm_result,
        error,
        "motor {} write address {}".format(
            motor_id,
            address
        )
    )


# ============================================================
# 모터 설정
# ============================================================

def configure_motors(port_handler, packet_handler):

    print()
    print("Configuring motors...")

    # --------------------------------------------------------
    # Torque OFF
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
    # P coefficient
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

    print("P coefficient = {}.".format(P_COEFFICIENT))

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
# 현재 모터 위치 읽기
# Jetson sms_sts API 방식
# ============================================================

def read_present_positions(port_handler, packet_handler):

    positions = {}

    print()
    print("Reading current motor positions...")

    for motor_name, motor_id in MOTORS.items():

        # Jetson에서는 port_handler를 인자로 다시 전달하지 않음
        position, comm_result, error = packet_handler.read2ByteTxRx(
            motor_id,
            ADDR_PRESENT_POSITION
        )

        if comm_result != scs.COMM_SUCCESS:

            raise RuntimeError(
                "Failed to read Present_Position of {}: {}".format(
                    motor_name,
                    packet_handler.getTxRxResult(comm_result)
                )
            )

        if error != 0:

            raise RuntimeError(
                "Servo error while reading {}: {}".format(
                    motor_name,
                    packet_handler.getRxPacketError(error)
                )
            )

        positions[motor_name] = int(position)

        print(
            "  {:15s}: {}".format(
                motor_name,
                positions[motor_name]
            )
        )

    return positions


# ============================================================
# GroupSyncWrite
# ============================================================

def send_positions(group_sync_write, packet_handler, frame):

    group_sync_write.clearParam()

    for motor_name, motor_id in MOTORS.items():

        value = int(frame[motor_name])

        # Jetson SDK의 sms_sts helper 사용
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
# Cosine interpolation
# 현재 자세 -> Hose2 첫 프레임
# ============================================================

def cosine_move_to_first_frame(
    group_sync_write,
    packet_handler,
    start_positions,
    target_positions,
    duration=INTERPOLATION_DURATION,
    fps=INTERPOLATION_FPS
):

    steps = max(1, int(duration * fps))
    dt = 1.0 / fps

    print()
    print("========================================")
    print(" Smooth transition to Hose2 start")
    print("========================================")
    print("Duration: {:.2f} sec".format(duration))
    print("FPS: {:.1f}".format(fps))
    print("Steps: {}".format(steps))

    print()
    print("Position difference:")

    for motor_name in MOTORS:

        start = start_positions[motor_name]
        target = target_positions[motor_name]

        print(
            "  {:15s}: {} -> {}  (delta={:+d})".format(
                motor_name,
                start,
                target,
                target - start
            )
        )

    print()
    print("Interpolation start...")

    start_time = time.perf_counter()

    for step in range(1, steps + 1):

        # 0.0 ~ 1.0
        t = float(step) / float(steps)

        # Cosine interpolation
        # 시작과 끝에서 속도가 0에 가까워짐
        ratio = (1.0 - math.cos(math.pi * t)) / 2.0

        frame = {}

        for motor_name in MOTORS:

            start = start_positions[motor_name]
            target = target_positions[motor_name]

            value = start + ratio * (target - start)

            frame[motor_name] = int(round(value))

        send_positions(
            group_sync_write,
            packet_handler,
            frame
        )

        # 일정한 주기로 보간 목표 전송
        target_time = start_time + step * dt
        remaining = target_time - time.perf_counter()

        if remaining > 0:
            time.sleep(remaining)

    # 마지막 프레임을 정확하게 한 번 더 전송
    send_positions(
        group_sync_write,
        packet_handler,
        target_positions
    )

    print("Interpolation complete.")


# ============================================================
# MAIN
# ============================================================

def main():

    print("========================================")
    print(" SO-100 Hose2 RAW CSV Replay - Jetson")
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

    # Jetson SDK 방식
    packet_handler = scs.sms_sts(port_handler)

    # --------------------------------------------------------
    # Port Open
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
    # GroupSyncWrite
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

        # ----------------------------------------------------
        # 현재 실제 자세 읽기
        # ----------------------------------------------------

        current_positions = read_present_positions(
            port_handler,
            packet_handler
        )

        # ----------------------------------------------------
        # Hose2 CSV 첫 자세
        # ----------------------------------------------------

        first_frame = frames[0]

        target_positions = {
            motor_name: int(first_frame[motor_name])
            for motor_name in MOTORS
        }

        # ----------------------------------------------------
        # 현재 자세 -> Hose2 시작 자세 Cosine 보간
        # ----------------------------------------------------

        cosine_move_to_first_frame(
            group_sync_write,
            packet_handler,
            current_positions,
            target_positions
        )

        # ----------------------------------------------------
        # Hose2 Replay 시작
        # ----------------------------------------------------

        print()
        print("HOSE2 REPLAY START")
        print()

        # 보간 완료 시점을 CSV time=0으로 설정
        replay_start = time.perf_counter()

        # ----------------------------------------------------
        # Replay
        # ----------------------------------------------------

        for index, frame in enumerate(frames):

            target_time = replay_start + frame["time"]

            while True:

                remaining = target_time - time.perf_counter()

                if remaining <= 0:
                    break

                if remaining > 0.002:
                    time.sleep(remaining - 0.001)

            send_positions(
                group_sync_write,
                packet_handler,
                frame
            )

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
        print("HOSE2 REPLAY FINISHED")

    except KeyboardInterrupt:

        print()
        print("Replay stopped by user.")

    except Exception as e:

        print()
        print("ERROR:")
        print(e)

    finally:

        # ----------------------------------------------------
        # Hose2 종료
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