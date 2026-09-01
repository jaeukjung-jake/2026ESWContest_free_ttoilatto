# mode2_lid_mqtt (MicroPython, ESP32)
#
# MG996R(360도 연속회전 서보)를 MQTT 명령으로 개폐하는 코드.
# 젯슨 나노(브로커) 가 토픽 "mode2/lid_cmd/<STALL_ID>" 에 아래 페이로드를
# publish 하면 해당 동작을 수행한다.
#   b"ENTER" -> 정방향 OPEN_MS 동안 회전 (뚜껑 열림)
#   b"EXIT"  -> 역방향 CLOSE_MS 동안 회전 (뚜껑 닫힘)
#
# [변기 여러 개] 변기마다 이 코드를 올린 ESP32 보드를 하나씩 둔다. 보드마다
# 아래 STALL_ID만 그 변기 번호로 바꿔서 올리면 됨 (mode2_stall.py가 현재
# 작업 중인 stall_id로 "mode2/lid_cmd/<stall_id>" 토픽에 publish하므로,
# 이 보드는 자기 담당 stall_id 토픽만 구독해서 반응함). MQTT_CLIENT_ID도
# 보드마다 겹치면 브로커가 서로를 끊어버리니 STALL_ID로 자동 구분되게 함.
#
# 사전 준비 (PC에서, ESP32를 USB로 연결한 상태):
#   1) ESP32용 MicroPython 펌웨어 플래시
#        pip install esptool
#        esptool.py --port <PORT> erase_flash
#        esptool.py --port <PORT> write_flash -z 0x1000 esp32-<version>.bin
#      (펌웨어 .bin은 https://micropython.org/download/ESP32_GENERIC/ 에서 받는다)
#   2) mpremote 설치 및 umqtt.simple 라이브러리 설치
#        pip install mpremote
#        mpremote connect <PORT> mip install umqtt.simple
#   3) 아래 STALL_ID를 이 보드가 담당할 변기 번호로 바꾼 뒤, 파일을 보드에
#      main.py 로 업로드 (전원 인가 시 자동 실행됨)
#        mpremote connect <PORT> cp arduino/main.py :main.py
#
# 아래 WIFI_SSID / WIFI_PASSWORD / MQTT_BROKER 를 환경에 맞게 수정할 것.

import time

import network
from machine import PWM, Pin
from umqtt.simple import MQTTClient

# ---------------- WiFi 설정 ----------------
WIFI_SSID = "embed1203"
WIFI_PASSWORD = "embed1203"

# ---------------- 변기 번호 (보드마다 다르게!) ----------------
# 이 보드가 담당하는 변기 번호. 보드마다 이 한 줄만 바꿔서 올리면 됨.
STALL_ID = "1"

# ---------------- MQTT 설정 ----------------
MQTT_BROKER = "192.168.0.103"  # 브로커(젯슨 나노) IP
MQTT_PORT = 1883
# STALL_ID를 그대로 붙여서 보드마다 자동으로 겹치지 않게 함
MQTT_CLIENT_ID = "esp32_mode2_lid_%s" % STALL_ID
TOPIC_LID_CMD = ("mode2/lid_cmd/%s" % STALL_ID).encode()  # payload: b"ENTER" / b"EXIT"

# ---------------- 서보 설정 ----------------
SERVO_PIN = 18
PWM_FREQ_HZ = 50
PERIOD_US = 1000000 // PWM_FREQ_HZ  # 50Hz -> 20000us

STOP_US = 1500
FORWARD_US = 1700  # 뚜껑 여는 방향
REVERSE_US = 1300  # 뚜껑 닫는 방향

OPEN_MS = 8000   # 실측: 최대로 열리는 데 걸리는 시간
CLOSE_MS = 1500  # 실측: 완전히 닫히는 데 걸리는 시간

servo = PWM(Pin(SERVO_PIN), freq=PWM_FREQ_HZ)


def servo_write_us(pulse_us):
    duty = int(pulse_us * 65535 // PERIOD_US)
    servo.duty_u16(duty)


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("[wifi] connecting to", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        while not wlan.isconnected():
            time.sleep_ms(500)
            print(".", end="")
        print("")
    print("[wifi] connected:", wlan.ifconfig())


def enter_lid():
    print("[mode2] ENTER -> opening lid...")
    servo_write_us(FORWARD_US)
    time.sleep_ms(OPEN_MS)
    servo_write_us(STOP_US)
    print("[mode2] lid OPEN done")


def exit_lid():
    print("[mode2] EXIT -> closing lid...")
    servo_write_us(REVERSE_US)
    time.sleep_ms(CLOSE_MS)
    servo_write_us(STOP_US)
    print("[mode2] lid CLOSED done")


def on_message(topic, msg):
    print("[mqtt]", topic, "->", msg)
    if topic != TOPIC_LID_CMD:
        return
    if msg == b"ENTER":
        enter_lid()
    elif msg == b"EXIT":
        exit_lid()
    else:
        print("[mqtt] unknown payload:", msg)


def main():
    servo_write_us(STOP_US)
    connect_wifi()

    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, port=MQTT_PORT)
    client.set_callback(on_message)
    client.connect()
    client.subscribe(TOPIC_LID_CMD)
    print("[mqtt] connected & subscribed to", TOPIC_LID_CMD)

    while True:
        try:
            # 메시지가 올 때까지 대기 (모터 구동 중에는 그 시간만큼 블로킹됨)
            client.wait_msg()
        except OSError as e:
            print("[mqtt] connection lost, reconnecting...", e)
            time.sleep(2)
            try:
                client.connect()
                client.subscribe(TOPIC_LID_CMD)
            except OSError:
                pass


if __name__ == "__main__":
    main()