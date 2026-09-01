🚽 모방학습 기반 자율주행 변기 청소 로봇

2026 임베디드 소프트웨어 경진대회
Team 또이라또 (ttoilatto)

1. 프로젝트 개요
본 프로젝트는 공공화장실의 반복적이고 비위생적인 변기 청소 작업을 자동화하기 위해 개발한 모방학습 기반 자율주행 변기 청소 Mobile Manipulator이다.
LiDAR, IMU, Wheel Encoder를 활용하여 화장실 내부를 이동하고 목표 변기칸에 진입한 후, LiDAR를 이용하여 변기 앞에서 로봇의 위치와 방향을 정렬한다.
변기 내부는 곡면으로 이루어져 있어 단순한 좌표 기반 제어만으로 복잡한 접촉 청소 동작을 구현하기 어렵다. 이를 해결하기 위해 사람이 SO-100 Leader Robot Arm을 직접 조작하여 청소 동작을 시연하고, Camera Image와 Joint State/Action 데이터를 수집하였다. 수집한 데이터를 LeRobot Dataset으로 구성하고 ACT Policy로 학습하여 Follower Robot Arm이 사람의 청소 동작을 재현하도록 구현하였다.
최종적으로 자율주행 → 변기칸 진입 → 위치 및 방향 정렬 → 세정액 분사 → 로봇팔 청소 → 복귀 과정을 하나의 시스템으로 통합하였다.

2. 개발 목적
공공화장실의 변기 청소는 반복적으로 수행해야 하며 오염된 환경에 작업자가 직접 노출되는 문제가 있다.
또한 변기는 복잡한 곡면 구조를 가지기 때문에 단순 반복 동작이나 고정된 좌표만으로 실제 사람이 수행하는 청소 동작을 구현하기 어렵다. 이러한 문제를 해결하기 위해 본 프로젝트에서는 다음 두 기술을 결합하였다.
- 센서 기반 자율주행 Mobile Platform
- 모방학습 기반 6DOF Robot Manipulator
이를 통해 로봇이 스스로 변기까지 이동하고 사람의 청소 동작을 학습하여 실제 청소 작업까지 수행하는 것을 목표로 한다.

3. 주요 기능
3.1. 자율주행 및 변기칸 접근
LDS-01 LiDAR, BNO08x IMU, Wheel Encoder를 이용하여 로봇의 주변 환경과 이동 상태를 인식한다.
ROS1 Melodic 기반으로 센서 데이터를 처리하고 Mobile Platform을 제어하여 목표 변기칸까지 이동하도록 구현하였다.
주요 기능은 다음과 같다.
- LiDAR 기반 주변 환경 및 장애물 인식
- IMU 기반 로봇 방향 정보 측정
- Wheel Encoder 기반 이동량 측정
- 목표 변기칸 방향으로 이동
- 변기칸 내부 진입
- 변기 청소 위치까지 접근

3.2. LiDAR 기반 변기칸 정렬
좁은 변기칸 내부에서 로봇팔이 안정적으로 청소 작업을 수행하려면 Mobile Platform이 일정한 위치와 방향에서 정지해야 한다.
이를 위해 LiDAR 데이터를 이용하여 변기칸 좌·우 벽면과 로봇 사이의 거리 및 방향을 계산하고, 좌우 벽면을 기준으로 로봇의 위치와 방향을 보정하도록 구현하였다.
이를 통해 로봇이 변기 앞의 일정한 위치에 정렬된 상태에서 Manipulation 작업을 시작할 수 있도록 하였다.

3.3. 모방학습 기반 변기 청소
변기 내부의 곡면을 따라 수행되는 복잡한 청소 동작을 구현하기 위해 모방학습을 적용하였다.
사람이 SO-100 Leader Robot Arm을 직접 조작하여 청소 동작을 시연하고 다음 데이터를 수집하였다.

Camera Image
      +
Joint State / Action
      ↓
LeRobot Dataset
      ↓
ACT Policy 학습
      ↓
Follower Robot Arm
      ↓
Cleaning Motion

수집한 데이터를 LeRobot Dataset으로 구성하고 ACT Policy로 학습하여 Follower Robot Arm이 사람의 청소 동작을 재현하도록 구현하였다. 이러한 Leader/Follower 기반 시연 수집과 ACT 적용 방식은 프로젝트 초기 모방학습 설계의 핵심 구조이다.
최종적으로 학습된 데이터를 기반으로 동작 제어 코드를 작성하여 Follower Robot Arm 제어로 연결하였다.

3.4. 세정액 자동 분사
변기 청소 전에 세정액을 자동으로 분사할 수 있도록 워터펌프 제어 시스템을 구현하였다.
ROS Node에서 펌프 동작 명령을 전달하고, 변기 앞 정렬이 완료된 이후 정해진 순서에 따라 세정액을 분사하도록 구성하였다.

변기 접근 및 정렬 완료
        ↓
Water Pump 동작
        ↓
세정액 분사
        ↓
Robot Arm 청소 시작

이를 통해 이동 플랫폼, 세정액 분사 장치, 로봇팔의 청소 동작을 하나의 청소 시나리오로 연계하였다.

4. 전체 동작 과정
전체 시스템은 다음 순서로 동작한다.
START
  │
  ▼
① 화장실 환경 인식
  │
  ▼
② 목표 변기칸 이동
  │
  ▼
③ 변기칸 진입
  │
  ▼
④ LiDAR 기반 위치·방향 정렬
  │
  ▼
⑤ 변기 청소 위치까지 접근
  │
  ▼
⑥ 세정액 분사
  │
  ▼
⑦ Follower Robot Arm 청소
  │
  ▼
⑧ 복귀
  │
  ▼
END
LiDAR를 이용한 환경 인식부터 변기칸 진입, 좌우 정렬, 변기 접근, 로봇팔 청소로 이어지는 동작 시나리오를 기반으로 시스템을 구성하였다.

5. Software Architecture
본 프로젝트의 소프트웨어는 ROS1 Melodic을 기반으로 구성하였다.
전체 청소 과정을 기능별 Mode로 분리하고 mode_manager.py에서 각 Mode의 실행 순서를 관리한다.
                    ┌────────────────────┐
                    │   mode_manager.py  │
                    └─────────┬──────────┘
                              │
             ┌────────────────┼────────────────┐
             ▼                ▼                ▼
       Mode 1             Mode 2           Mode 3
    Navigation         Stall Control       Cleaning
             │                │                │
             └────────────────┼────────────────┘
                              │
                              ▼
                         Sensor Nodes
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
            LiDAR            IMU          Wheel Encoder
Mode 1 — Navigation
mode1_node.py
- 화장실 내부 이동
- IMU 및 Encoder 기반 주행 상태 확인
- 목표 변기칸 접근
Mode 2 — Stall Control
mode2_stall.py
- LiDAR 데이터 기반 변기칸 벽면 인식
- 좌·우 벽면을 이용한 위치 보정
- 로봇 방향 정렬
- 변기 청소 위치 접근
Mode 3 — Cleaning
mode3_clean.py
- 세정액 분사
- 로봇팔 청소 과정 실행
- Brush / Hose 동작 관리
- 청소 종료 후 다음 동작 연결

6. Sensor & Control Nodes
- lidar_node.py
LDS-01에서 수신한 LaserScan 데이터를 처리하여 로봇 주변의 거리 정보를 생성한다.
- imusensor.py
IMU로부터 로봇의 자세 및 방향 정보를 획득한다.
- imu_bridge_node.py
IMU 데이터를 ROS 환경에서 사용할 수 있도록 /imu/data 형태의 ROS Topic으로 연결한다.
- odom.py
Wheel Encoder 및 센서 정보를 이용하여 Mobile Platform의 이동 상태를 관리한다.
- serial_bridge_node.py
Jetson Nano와 Mobile Platform의 하위 제어 장치 사이의 Serial 통신을 담당한다.
- pump_control_node.py
Water Pump를 제어하여 청소 단계에서 세정액을 자동으로 분사한다.

7. Repository Structure
2026ESWContest_free_ttoilatto/
│
├── launch/
│   ├── bringup.launch
│   ├── modes.launch
│   └── sensors.launch
│
├── param/
│   ├── mode1_param.yaml
│   └── mode2_param.yaml
│
├── scripts/
│   ├── modes/
│   │   ├── mode1_node.py
│   │   ├── mode2_stall.py
│   │   └── mode3_clean.py
│   │
│   ├── sensors/
│   │   ├── imu_bridge_node.py
│   │   ├── imusensor.py
│   │   ├── lidar_node.py
│   │   ├── odom.py
│   │   ├── pump_control_node.py
│   │   └── serial_bridge_node.py
│   │
│   ├── mode_manager.py
│   └── run_imusensor.sh
│
├── src/
│   └── toilet_cleaning/
│       ├── __init__.py
│       └── utils.py
│
├── standalone_brush_replay_jetson.py
├── standalone_hose1_replay_jetson.py
├── standalone_hose2_replay_jetson.py
│
├── toilet_brush_replay_new_raw.csv
├── toilet_hose1_replay_new_raw.csv
├── toilet_hose2_replay_new_raw.csv
│
├── CMakeLists.txt
├── package.xml
└── README.md

8. 기대효과
- 반복적이고 비위생적인 변기 청소 작업을 자동화하여 작업자의 위생 부담과 노동 강도를 감소시킬 수 있다.
- 자율주행과 모방학습 기반 로봇팔을 결합하여 이동부터 청소까지 연속적인 무인 작업 수행이 가능하다.
- 향후 다양한 청소 동작을 추가하여 공공화장실 및 다중이용시설의 청소 자동화 시스템으로 확장할 수 있다.
