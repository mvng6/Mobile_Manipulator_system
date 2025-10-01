# TR200 Simple Control Package

TR200 로봇의 라이다 센서를 활용한 자동 구동 및 안전 제어 시스템입니다. 센서 데이터를 기반으로 장애물 감지, 안전 거리 유지, 자동 정지 등의 기능을 제공합니다.

## 📋 개요

이 패키지는 TR200 로봇의 듀얼 라이다 센서를 활용하여 자동으로 앞뒤 구동하면서 장애물을 감지하고 안전 거리에 따라 속도를 조절하거나 정지하는 시스템입니다. ROS 환경에서 실행되며 실시간 센서 데이터 처리와 안전 제어를 제공합니다.

## 🎯 핵심 기능

- **듀얼 라이다 센서 활용**: TR200의 전방/후방 라이다 센서 데이터 통합 처리
- **실시간 장애물 감지**: 360도 전후방 장애물 감지 및 거리 측정
- **지능형 속도 제어**: 거리에 따른 자동 속도 조절 (정상 → 감속 → 정지)
- **안전 우선 제어**: 위험 상황에서 즉시 비상 정지
- **부드러운 제어**: 급격한 가속/감속 방지
- **자동 방향 전환**: 일정 시간마다 자동으로 방향 전환
- **ROS 통합**: ROS 토픽/서비스를 통한 실시간 모니터링 및 제어

## 🏗️ 시스템 구조

### 주요 컴포넌트
- **센서 데이터 처리**: 듀얼 라이다 센서 데이터 수신 및 분석
- **안전 제어 로직**: 거리 기반 속도 조절 및 비상 정지
- **로봇 제어**: SDK를 통한 실제 로봇 구동 제어
- **ROS 인터페이스**: 토픽 발행 및 서비스 제공

### ROS 토픽
- `/scan`: 원본 스캐너 데이터 (sensor_msgs/LaserScan)
- `/processed_scan`: 처리된 센서 데이터 (sensor_msgs/LaserScan)
- `/safe_cmd_vel`: 안전한 속도 명령 (geometry_msgs/Twist)
- `/safety_status`: 안전 상태 (std_msgs/String)
- `/obstacle_distance`: 장애물 거리 (std_msgs/Float32)
- `/robot_status`: 로봇 상태 (std_msgs/String)
- `/emergency_stop`: 비상 정지 상태 (std_msgs/Bool)

### ROS 서비스
- `/set_safety_params`: 안전 파라미터 동적 설정
- `/emergency_stop`: 비상 정지 서비스

## 🚀 빠른 시작

### 1. 환경 설정

Docker 컨테이너에서 작업:
```bash
# 컨테이너 실행
./scripts/run_container.sh

# ROS 마스터 실행
roscore

# 추가 터미널 접속
./scripts/connect_container.sh
```

### 2. 패키지 빌드

```bash
cd /catkin_ws
catkin_make
source devel/setup.bash
```

### 3. 센서 기반 안전 제어 실행

#### 방법 1: ROS 런치 파일로 실행 (권장)
```bash
# 기본 설정으로 실행
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch

# 파라미터 조정하여 실행
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch \
  min_obstacle_distance:=0.3 \
  warning_distance:=0.6 \
  normal_speed:=0.15
```

#### 방법 2: 직접 실행
```bash
# ROS 통합 제어기 실행
rosrun tr200_simple_control sensor_based_safety_controller.py

# 기본 왕복 운동 (참고용)
rosrun tr200_simple_control simple_linear_motion.py
```

### 4. RViz 시각화

```bash
# RViz로 센서 데이터 시각화
rviz -d src/tr200_simple_control/config/tr200_sensor_safety.rviz
```

## 📁 패키지 구조

```
tr200_simple_control/
├── CMakeLists.txt                           # 빌드 설정
├── package.xml                             # 패키지 매니페스트
├── README.md                               # 패키지 문서
├── launch/                                 # ROS 런치 파일
│   └── tr200_sensor_safety_controller.launch   # 센서 안전 제어 런치
├── config/                                 # 설정 파일
│   ├── tr200_sensor_safety_params.yaml        # ROS 통합 안전 파라미터
│   ├── tr200_sensor_safety.rviz              # RViz 시각화 설정
│   ├── area_motion_params.yaml               # 영역 제한 구동 파라미터
│   ├── robot_params.yaml                     # 로봇 기본 파라미터
│   └── test_safe_params.yaml                 # 테스트용 안전 파라미터
├── scripts/                                # Python 제어 스크립트
│   ├── sensor_based_safety_controller.py      # 순수 SDK 제어기
│   └── simple_linear_motion.py                # 기본 왕복 운동
└── srv/                                    # ROS 서비스 정의
    └── SetSafetyParams.srv                    # 안전 파라미터 설정 서비스
```

## ⚙️ 설정

### 안전 제어 파라미터

`config/tr200_sensor_safety_params.yaml`에서 안전 제어 파라미터를 설정할 수 있습니다:

```yaml
# 센서 설정 (단위: 미터)
sensor:
  min_obstacle_distance: 0.5    # 최소 장애물 거리 - 위험 구역
  warning_distance: 0.8         # 경고 거리 - 주의 구역  
  safe_distance: 1.0           # 안전 거리 - 안전 구역
  
  # 센서 활성화
  front_sensor_enabled: true
  rear_sensor_enabled: true

# 제어 설정 (단위: m/s)
control:
  normal_speed: 0.2            # 정상 속도
  slow_speed: 0.1              # 감속 속도
  stop_speed: 0.0              # 정지 속도
  control_frequency: 20.0      # 제어 주파수 (Hz)
```

### 로봇 연결 설정

```yaml
# 로봇 연결 정보
robot:
  ip: "169.254.128.2"  # TR200 로봇 IP
  port: 5480           # TR200 로봇 포트
  identity: "tr200_sensor_controller"
```

## 🔍 센서 데이터 처리

### 듀얼 센서 분석

시스템은 TR200의 듀얼 라이다 센서 데이터를 다음과 같이 처리합니다:

1. **센서 데이터 분리**: 전방 센서(270°~810°)와 후방 센서(810°~270°)로 분리
2. **장애물 감지**: 각 센서별로 최소 거리 및 장애물 개수 계산
3. **통합 분석**: 전후방 센서 데이터를 통합하여 전체 안전 상태 판단

### 안전 구역 설정

- **안전 구역** (1.0m 이상): 정상 속도로 이동
- **주의 구역** (0.8m~1.0m): 감속하여 이동
- **위험 구역** (0.5m 이하): 즉시 정지

## 📊 모니터링 및 디버깅

### ROS 토픽 모니터링

```bash
# 안전 상태 모니터링
rostopic echo /safety_status

# 장애물 거리 모니터링
rostopic echo /obstacle_distance

# 속도 명령 모니터링
rostopic echo /safe_cmd_vel

# 스캐너 데이터 모니터링
rostopic echo /scan

# 로봇 상태 모니터링
rostopic echo /robot_status
```

### ROS 서비스 사용

```bash
# 안전 파라미터 동적 변경
rosservice call /set_safety_params "min_obstacle_distance: 0.3
warning_distance: 0.6
safe_distance: 0.9
normal_speed: 0.15
slow_speed: 0.05"

# 비상 정지 서비스
rosservice call /emergency_stop "data: true"
```

### 예상 출력

```
📡 스캐너 데이터 수신: 1081개 포인트
📡 각도 범위: -3.14° ~ 3.14°
📡 거리 범위: 0.01m ~ 50.00m
============================================================
🔍 센서 분리 완료:
   전방 센서: 540개 포인트 (인덱스 270~810)
   후방 센서: 540개 포인트 (인덱스 810~270)
🔍 듀얼 센서 분석:
   전방 센서: 최소거리 1.200m, 장애물 2개
   후방 센서: 최소거리 2.500m, 장애물 0개
   전체 최소거리: 1.200m, 총 장애물: 2개
```

## 🔧 개발

### 새로운 제어 로직 추가

1. `scripts/` 디렉토리에 새 제어 스크립트 생성
2. `CMakeLists.txt`에 실행 파일 등록
3. 필요시 런치 파일에 노드 추가

### 새로운 설정 파일 추가

1. `config/` 디렉토리에 새 설정 파일 생성
2. 런치 파일에서 파라미터 로드 확인
3. 코드에서 파라미터 읽기 구현

## 🚨 안전 주의사항

### 첫 실행 시
1. **충분한 공간 확보**: 로봇 주변에 장애물이 없는지 확인
2. **낮은 속도로 테스트**: `normal_speed:=0.1`로 설정
3. **비상정지 준비**: 언제든지 시스템을 중단할 수 있도록 준비

### 일반 사용 시
1. **센서 상태 확인**: 라이다 센서가 정상 작동하는지 확인
2. **안전 거리 설정**: 환경에 맞게 안전 거리 조정
3. **정기적 점검**: 센서 데이터가 정상적으로 수신되는지 확인

## 🐛 트러블슈팅

### 센서 데이터 수신 안됨

1. 로봇 연결 상태 확인
2. 라이다 센서 하드웨어 상태 확인
3. 토픽 구독 상태 확인

### 장애물 감지가 부정확함

1. 센서 파라미터 조정
2. 노이즈 필터링 설정 확인
3. 안전 거리 임계값 조정

### 로봇이 움직이지 않음

1. 로봇 연결 상태 확인
2. 비상정지 상태 확인
3. 속도 명령이 정상적으로 전송되는지 확인

## 📞 지원

문제가 발생하면 다음을 확인하세요:

1. 로그 메시지 확인
2. ROS 토픽 상태 확인
3. 센서 데이터 수신 상태 확인
4. 로봇 연결 상태 확인

## 📄 라이선스

MIT License

## 👥 기여자

- User (KATECH 연구원) - 프로젝트 설계 및 구현

---

**버전**: v1.0.0  
**최종 업데이트**: 2025년 10월 01일
