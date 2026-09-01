#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import time
import board
import busio
import socket
import json
import sys
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

# I2C 및 센서 설정 (주소 0x4A)
i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c, address=0x4A)

# 모든 가용 리포트 활성화
report_list = [
    adafruit_bno08x.BNO_REPORT_ACCELEROMETER,
    adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION,
    adafruit_bno08x.BNO_REPORT_GRAVITY,
    adafruit_bno08x.BNO_REPORT_GYROSCOPE,
    adafruit_bno08x.BNO_REPORT_MAGNETOMETER,
    adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR,
    adafruit_bno08x.BNO_REPORT_STABILITY_CLASSIFIER
]
for r in report_list:
    bno.enable_feature(r)

# UDP 설정
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 실행 시 화면 전체를 한 번 깨끗하게 비움
sys.stdout.write("\033[H\033[J")
sys.stdout.flush()

try:
    while True:
        # 데이터 수집
        try:
            acc = bno.acceleration
            l_acc = bno.linear_acceleration
            gyro = bno.gyro
            q = bno.quaternion
            stab = bno.stability_classification
        except Exception as e:
            # I2C 통신 오류 등 센서 데이터 읽기 실패 시
            sys.stdout.write(f"\n[TX ERROR] 센서 데이터 읽기 실패: {e}\n")
            time.sleep(0.5) # 0.5초 후 재시도
            continue

        payload = {
            "acc": acc, "l_acc": l_acc, "gyro": gyro, 
            "quat": q, "stab": str(stab)
        }
        
        sock.sendto(json.dumps(payload).encode(), (UDP_IP, UDP_PORT))
        
        # [TX 출력] \033[H로 맨 위로 이동 후 덮어쓰기
        # \033[K는 줄의 나머지 부분을 지워 잔상을 방지함
        output = (
            "\033[H"
            "\033[K[TX 1] Acc : [%6.2f, %6.2f, %6.2f] | L_Acc: [%6.2f, %6.2f, %6.2f]\n"
            "\033[K[TX 2] Gyro: [%6.2f, %6.2f, %6.2f] | Quat : [%5.2f, %5.2f, %5.2f, %5.2f] | S:%s"
        ) % (
            acc[0], acc[1], acc[2], l_acc[0], l_acc[1], l_acc[2],
            gyro[0], gyro[1], gyro[2], q[0], q[1], q[2], q[3], str(stab)
        )
        sys.stdout.write(output)
        sys.stdout.flush()
        
        time.sleep(0.01)
except KeyboardInterrupt:
    sys.stdout.write("\n[TX] 프로세스 종료\n")