# Mobile Manipulator ROS 통합 제어 시스템

## 📋 프로젝트 개요

TR200 로봇을 위한 ROS 기반 통합 제어 시스템입니다. Docker 기반 ROS Noetic 환경에서 Woosh SDK를 통해 TR200 로봇을 제어하며, 두 가지 주요 제어 방식을 제공합니다:

1. **원격 제어**: 키보드 입력을 통한 수동 로봇 제어
2. **자동 제어**: 라이다 센서 기반 자동 구동 및 안전 제어

## 🎯 핵심 기능

### 공통 기능
- **ROS + SDK 통합**: ROS의 모듈성과 SDK의 직접 제어 장점 결합
- **Docker 기반 환경**: Ubuntu 20.04 + ROS Noetic 환경에서 안정적 실행
- **실시간 모니터링**: ROS 토픽/서비스를 통한 실시간 상태 모니터링
- **안전 우선 제어**: 위험 상황에서 즉시 비상 정지

### 원격 제어 기능 (tr200_ros_control)
- **키보드 제어**: 직관적인 키보드 입력을 통한 로봇 조작
- **모듈화된 노드**: 연결 관리, 원격 제어, 로봇 구동 노드 분리
- **안전 제한**: 속도 제한 및 안전 파라미터 설정

### 자동 제어 기능 (tr200_simple_control)
- **듀얼 라이다 센서**: TR200의 전방/후방 라이다 센서 활용
- **실시간 장애물 감지**: 360도 전후방 장애물 감지 및 거리 측정
- **지능형 속도 제어**: 거리에 따른 자동 속도 조절 (정상 → 감속 → 정지)
- **자동 방향 전환**: 일정 시간마다 자동으로 방향 전환

## 📁 프로젝트 구조

## 📁 프로젝트 구조

```
mobile_manipulator_ws/tr200_ros_docker_project/
├── README.md                                      # 프로젝트 문서
├── ROS_INDUSTRY_DEVELOPMENT_GUIDE.md              # 현업 ROS 개발 가이드
├── scripts/                                       # 실행 스크립트
│   ├── build_docker.sh                               # Docker 이미지 빌드
│   ├── run_container.sh                              # Docker 컨테이너 실행
│   ├── connect_container.sh                          # 추가 터미널 접속
│   ├── setup_ros.sh                                  # ROS 환경 설정
│   └── test_ros_sensor_safety.sh                     # ROS 통합 테스트
├── src/                                           # ROS 패키지 소스
│   ├── tr200_ros_control/                          # 원격 제어 패키지
│   │   ├── README.md                                  # 원격 제어 패키지 문서
│   │   ├── launch/                                    # ROS 런치 파일
│   │   │   ├── tr200_remote_control.launch               # 통합 원격 제어 런치
│   │   │   ├── tr200_base_system.launch                 # 기본 시스템 런치
│   │   │   ├── tr200_keyboard_control.launch            # 키보드 제어 런치
│   │   │   └── tr200_keyboard_control_safe.launch       # 안전 모드 런치
│   │   ├── config/                                    # 설정 파일
│   │   │   ├── robot_params.yaml                         # 로봇 기본 파라미터
│   │   │   ├── safety_params.yaml                       # 안전 제어 파라미터
│   │   │   └── sensor_params.yaml                       # 센서 파라미터
│   │   ├── scripts/                                   # Python 제어 스크립트
│   │   │   ├── robot_connection_node.py                  # 로봇 연결 관리 노드
│   │   │   ├── remote_control_node.py                   # 원격 제어 노드
│   │   │   └── robot_driver_node.py                     # 로봇 구동 제어 노드
│   │   ├── srv/                                      # ROS 서비스 정의
│   │   │   ├── SetSafetyParams.srv                      # 안전 파라미터 설정 서비스
│   │   │   └── GetRobotStatus.srv                       # 로봇 상태 조회 서비스
│   │   └── msg/                                      # ROS 메시지 정의
│   │       ├── SafetyStatus.msg                          # 안전 상태 메시지
│   │       └── RobotStatus.msg                          # 로봇 상태 메시지
│   ├── tr200_simple_control/                        # 자동 제어 패키지
│   │   ├── README.md                                  # 자동 제어 패키지 문서
│   │   ├── launch/                                    # ROS 런치 파일
│   │   │   └── tr200_sensor_safety_controller.launch     # 센서 안전 제어 런치
│   │   ├── config/                                    # 설정 파일
│   │   │   ├── tr200_sensor_safety_params.yaml           # ROS 통합 안전 파라미터
│   │   │   ├── tr200_sensor_safety.rviz                  # RViz 시각화 설정
│   │   │   ├── area_motion_params.yaml                   # 영역 제한 구동 파라미터
│   │   │   ├── robot_params.yaml                         # 로봇 기본 파라미터
│   │   │   └── test_safe_params.yaml                     # 테스트용 안전 파라미터
│   │   ├── scripts/                                   # Python 제어 스크립트
│   │   │   ├── sensor_based_safety_controller.py         # 순수 SDK 제어기
│   │   │   └── simple_linear_motion.py                   # 기본 왕복 운동
│   │   └── srv/                                      # ROS 서비스 정의
│   │       └── SetSafetyParams.srv                      # 안전 파라미터 설정 서비스
│   └── woosh_robot_py/                               # Woosh SDK
│       ├── README.md                                     # SDK 문서
│       ├── examples/                                     # 예제 코드
│       └── woosh/                                        # SDK 핵심 모듈
├── docker/                                        # Docker 환경
│   ├── Dockerfile                                    # Docker 이미지 정의
│   ├── docker-compose.yml                            # 컨테이너 오케스트레이션
│   ├── entrypoint.sh                                 # 컨테이너 시작 스크립트
│   ├── data/                                         # 데이터 디렉토리
│   └── logs/                                         # 로그 디렉토리
```

## 🚀 사용 방법

### 🐳 Docker 환경 설정

#### 1. Docker 이미지 빌드 (처음 한 번만)
```bash
./scripts/build_docker.sh
```

#### 2. Docker 컨테이너 실행 (첫 번째 터미널)
```bash
./scripts/run_container.sh
```

#### 3. 추가 터미널 접속 (두 번째 터미널)
```bash
./scripts/connect_container.sh
```

### 🔧 ROS 환경 설정

#### 1. ROS 환경 설정 (컨테이너 내부에서)
```bash
./scripts/setup_ros.sh
```

#### 2. 워크스페이스 빌드
```bash
tr200_build
# 또는
catkin_make
```

#### 3. 환경 재설정
```bash
source devel/setup.bash
```

#### 4. ROS Master 시작 (수동)
```bash
roscore &
```

### 🤖 TR200 로봇 제어 실행

## 🎮 원격 제어 (tr200_ros_control)

### 키보드 제어 실행

#### 방법 1: 분리된 실행 (권장)
```bash
# 터미널 1에서 기본 시스템 실행
roslaunch tr200_ros_control tr200_base_system.launch

# 터미널 2에서 키보드 제어 실행
roslaunch tr200_ros_control tr200_keyboard_control.launch
```

#### 방법 2: 통합 실행 (간단한 테스트용)
```bash
# 모든 노드를 한 번에 실행
roslaunch tr200_ros_control tr200_remote_control.launch
```

#### 방법 3: 안전 모드 실행 (개발용)
```bash
# 매우 낮은 속도로 실행
roslaunch tr200_ros_control tr200_keyboard_control_safe.launch
```

### 키보드 제어 방법
- **`w`** - 전진, **`s`** - 후진, **`a`** - 좌회전, **`d`** - 우회전
- **`q`** - 제자리 좌회전, **`e`** - 제자리 우회전
- **`스페이스`** - 정지, **`x`** - 비상정지
- **`h`** - 도움말, **`c`** - 연결 상태 확인

## 🤖 자동 제어 (tr200_simple_control)

### 센서 기반 안전 제어 실행

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
# 순수 SDK 제어기 실행
rosrun tr200_simple_control sensor_based_safety_controller.py

# 기본 왕복 운동 (참고용)
rosrun tr200_simple_control simple_linear_motion.py
```

### RViz 시각화
```bash
# RViz로 센서 데이터 시각화
rviz -d src/tr200_simple_control/config/tr200_sensor_safety.rviz
```

## 🔍 ROS 모니터링 및 디버깅

### 공통 모니터링 명령

#### ROS 토픽 모니터링
```bash
# 로봇 상태 모니터링
rostopic echo /robot_status

# 연결 상태 모니터링
rostopic echo /connection_status

# 비상 정지 상태 모니터링
rostopic echo /emergency_stop
```

#### ROS 서비스 사용
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

### 원격 제어 모니터링 (tr200_ros_control)
```bash
# 원격 제어 명령 확인
rostopic echo /remote_cmd_vel

# 속도 명령 확인
rostopic echo /cmd_vel
```

### 자동 제어 모니터링 (tr200_simple_control)
```bash
# 안전 상태 모니터링
rostopic echo /safety_status

# 장애물 거리 모니터링
rostopic echo /obstacle_distance

# 안전한 속도 명령 모니터링
rostopic echo /safe_cmd_vel

# 스캐너 데이터 모니터링
rostopic echo /scan
```

## ⚙️ 설정 파라미터

### 공통 로봇 설정
- **로봇 IP**: 169.254.128.2
- **로봇 포트**: 5480
- **제어 주파수**: 20Hz

### 원격 제어 설정 (tr200_ros_control)
- **기본 선속도**: 0.5 m/s
- **기본 각속도**: 0.5 rad/s
- **명령 타임아웃**: 2.0초
- **최대 선속도**: 1.0 m/s
- **최대 각속도**: 1.0 rad/s

### 자동 제어 설정 (tr200_simple_control)
- **경고 거리 (warning_distance)**: 0.8m (이 거리에서 감속 시작)
- **위험 거리 (min_obstacle_distance)**: 0.5m (이 거리 이하에서 즉시 정지)
- **안전 거리 (safe_distance)**: 1.0m (이 거리 이상에서 정상 속도)
- **정상 속도 (normal_speed)**: 0.2 m/s
- **감속 속도 (slow_speed)**: 0.1 m/s
- **정지 속도 (stop_speed)**: 0.0 m/s

### 설정 파일들

#### 원격 제어 패키지 (tr200_ros_control)
- **`src/tr200_ros_control/config/robot_params.yaml`**: 로봇 기본 파라미터
- **`src/tr200_ros_control/config/safety_params.yaml`**: 안전 제어 파라미터
- **`src/tr200_ros_control/config/sensor_params.yaml`**: 센서 파라미터

#### 자동 제어 패키지 (tr200_simple_control)
- **`src/tr200_simple_control/config/tr200_sensor_safety_params.yaml`**: ROS 통합 안전 파라미터 (기본 설정)
- **`src/tr200_simple_control/config/area_motion_params.yaml`**: 영역 제한 구동 파라미터
- **`src/tr200_simple_control/config/robot_params.yaml`**: 로봇 기본 파라미터
- **`src/tr200_simple_control/config/test_safe_params.yaml`**: 테스트용 안전 파라미터 (더 보수적 설정)

### ROS 토픽 및 서비스

#### 공통 토픽
- **`/robot_status`**: 로봇 상태 (std_msgs/String)
- **`/connection_status`**: 연결 상태 (std_msgs/String)
- **`/emergency_stop`**: 비상 정지 상태 (std_msgs/Bool)

#### 원격 제어 토픽 (tr200_ros_control)
- **`/cmd_vel`**: 로봇 속도 명령 (geometry_msgs/Twist)
- **`/remote_cmd_vel`**: 원격 제어 명령 (geometry_msgs/Twist)

#### 자동 제어 토픽 (tr200_simple_control)
- **`/scan`**: 원본 스캐너 데이터 (sensor_msgs/LaserScan)
- **`/processed_scan`**: 처리된 센서 데이터 (sensor_msgs/LaserScan)
- **`/safe_cmd_vel`**: 안전한 속도 명령 (geometry_msgs/Twist)
- **`/safety_status`**: 안전 상태 (std_msgs/String)
- **`/obstacle_distance`**: 장애물 거리 (std_msgs/Float32)

#### 제공 서비스
- **`/set_safety_params`**: 안전 파라미터 동적 설정 (tr200_simple_control/SetSafetyParams)
- **`/get_robot_status`**: 로봇 상태 조회 (tr200_ros_control/GetRobotStatus)

## 📊 예상 동작

### 원격 제어 시스템 실행 시 (tr200_ros_control)
```
[INFO] [1234567890.123]: Starting robot connection node
[INFO] [1234567890.124]: Starting remote control node
[INFO] [1234567890.125]: Starting robot driver node
[INFO] [1234567890.126]: ROS Master URI: http://localhost:11311
[INFO] [1234567890.127]: Loading parameters from config file
[INFO] [1234567890.128]: Robot IP: 169.254.128.2
[INFO] [1234567890.129]: Robot Port: 5480
[INFO] [1234567890.130]: Connecting to TR200 robot...
[INFO] [1234567890.131]: TR200 robot connected successfully
[INFO] [1234567890.132]: Remote control system initialized
[INFO] [1234567890.133]: Press 'h' for help
```

### 자동 제어 시스템 실행 시 (tr200_simple_control)
```
[INFO] [1234567890.123]: Starting sensor-based safety controller
[INFO] [1234567890.124]: Loading safety parameters
[INFO] [1234567890.125]: Warning distance: 0.8m
[INFO] [1234567890.126]: Danger distance: 0.5m
[INFO] [1234567890.127]: Normal speed: 0.2 m/s
[INFO] [1234567890.128]: Connecting to TR200 robot...
[INFO] [1234567890.129]: TR200 robot connected successfully
[INFO] [1234567890.130]: Subscribing to scanner data...
[INFO] [1234567890.131]: Safety controller initialized
```

### 센서 데이터 처리 (자동 제어)
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

### ROS 토픽 출력 예시
```
08:15:23 | forward   |  0.200 |  1.200m | 🟢 안전
08:15:24 | forward   |  0.200 |  1.100m | 🟢 안전
08:15:25 | forward   |  0.200 |  0.900m | 🟡 주의
⚠️ 주의: 장애물 근접 (거리: 0.900m)
08:15:26 | forward   |  0.100 |  0.800m | 🟡 주의
08:15:27 | forward   |  0.100 |  0.600m | 🟡 주의
08:15:28 | forward   |  0.000 |  0.500m | 🔴 위험
🚨 위험! 장애물 감지 (거리: 0.500m)
🚨 비상 정지! 이유: 장애물 감지 (거리: 0.500m)
✅ 비상 정지 완료
```

### ROS 서비스 호출 예시
```bash
# 안전 파라미터 동적 변경
$ rosservice call /set_safety_params "min_obstacle_distance: 0.3
warning_distance: 0.6
safe_distance: 0.9
normal_speed: 0.15
slow_speed: 0.05"
success: True
message: "파라미터 업데이트 성공"

# 비상 정지 서비스
$ rosservice call /emergency_stop "data: true"
success: True
message: "비상 정지 활성화"

# 로봇 상태 조회
$ rosservice call /get_robot_status "request: true"
status: "connected"
battery_level: 85
position: "x: 1.2, y: 0.8, theta: 0.5"
```

## 🔧 파일 설명

### 원격 제어 패키지 (tr200_ros_control)

#### 핵심 스크립트
- **`robot_connection_node.py`**: 로봇 연결 관리 노드 (SDK 연결 및 상태 모니터링)
- **`remote_control_node.py`**: 원격 제어 노드 (키보드 입력 처리 및 명령 생성)
- **`robot_driver_node.py`**: 로봇 구동 제어 노드 (SDK를 통한 실제 로봇 제어)

#### 런치 파일
- **`tr200_remote_control.launch`**: 통합 원격 제어 런치 (모든 노드 한 번에 실행)
- **`tr200_base_system.launch`**: 기본 시스템 런치 (연결 및 구동 노드만)
- **`tr200_keyboard_control.launch`**: 키보드 제어 런치 (원격 제어 노드만)
- **`tr200_keyboard_control_safe.launch`**: 안전 모드 런치 (매우 낮은 속도)

### 자동 제어 패키지 (tr200_simple_control)

#### 핵심 스크립트
- **`sensor_based_safety_controller.py`**: 순수 SDK 제어기 (듀얼 라이다 센서 기반 안전 제어)
- **`simple_linear_motion.py`**: 기본 왕복 운동 (참고용, 센서 없이 단순 이동)

#### 런치 파일
- **`tr200_sensor_safety_controller.launch`**: 센서 안전 제어 런치 파일

### 공통 설정 파일
- **`robot_params.yaml`**: 로봇 기본 파라미터 (연결 정보, 제어 설정)
- **`safety_params.yaml`**: 안전 제어 파라미터 (거리 임계값, 속도 설정)
- **`sensor_params.yaml`**: 센서 파라미터 (라이다 설정, 데이터 처리)

### 실행 스크립트들
- **`build_docker.sh`**: Docker 이미지 빌드
- **`run_container.sh`**: Docker 컨테이너 실행
- **`connect_container.sh`**: 추가 터미널 접속
- **`setup_ros.sh`**: ROS 환경 설정
- **`test_ros_sensor_safety.sh`**: ROS 통합 테스트

## 🎯 개발 목표 달성

### 공통 목표
✅ **ROS + SDK 통합**: ROS의 모듈성과 SDK의 직접 제어 장점 결합  
✅ **Docker 환경**: 안정적인 개발 및 배포 환경 구축  
✅ **현업 표준**: 모듈화 설계, 파라미터 관리, 런치 파일 구조  

### 원격 제어 목표 (tr200_ros_control)
✅ **키보드 제어**: 직관적인 키보드 입력을 통한 로봇 조작  
✅ **모듈화된 노드**: 연결 관리, 원격 제어, 로봇 구동 노드 분리  
✅ **안전 제한**: 속도 제한 및 안전 파라미터 설정  
✅ **ROS 토픽/서비스**: 실시간 모니터링 및 동적 파라미터 조정  

### 자동 제어 목표 (tr200_simple_control)
✅ **듀얼 라이다 센서**: TR200의 전방/후방 라이다 센서 활용  
✅ **실시간 장애물 감지**: 360도 전후방 장애물 감지 및 거리 측정  
✅ **지능형 속도 제어**: 거리에 따른 자동 속도 조절 (정상 → 감속 → 정지)  
✅ **안전한 구동**: 거리 기반 자동 속도 조절 및 비상 정지  
✅ **자동 방향 전환**: 일정 시간마다 자동으로 방향 전환  

## 🌐 네트워크 설정

### TR200 로봇 연결
- **IP**: 169.254.128.2
- **Port**: 5480
- **네트워크 모드**: host (Docker)

### ROS 통신
- **ROS Master**: http://localhost:11311
- **ROS Hostname**: localhost

## 🐛 문제 해결

### Docker 관련
```bash
# 컨테이너 상태 확인
docker ps | grep tr200_control_container

# 컨테이너 로그 확인
docker logs tr200_control_container

# 컨테이너 강제 재시작
docker stop tr200_control_container && docker rm tr200_control_container
```

### ROS 관련
```bash
# ROS Master 재시작
pkill roscore && roscore &

# 워크스페이스 재빌드
tr200_build
# 또는
catkin_make

# 환경 변수 확인
echo $ROS_PACKAGE_PATH
echo $PYTHONPATH
```

### TR200 연결 문제
```bash
# 네트워크 연결 확인
ping 169.254.128.2

# 포트 확인
telnet 169.254.128.2 5480
```

### 런치 파일 실행 문제
```bash
# 패키지 확인
rospack find tr200_ros_control
rospack find tr200_simple_control

# 런치 파일 확인
ls -la $(rospack find tr200_ros_control)/launch/
ls -la $(rospack find tr200_simple_control)/launch/

# config 파일 확인
ls -la $(rospack find tr200_ros_control)/config/
ls -la $(rospack find tr200_simple_control)/config/
```

## 📝 개발 가이드

### 새로운 노드 추가
1. `src/tr200_ros_control/scripts/` 또는 `src/tr200_simple_control/scripts/`에 Python 파일 추가
2. `CMakeLists.txt`에 실행 파일 등록
3. `package.xml`에 의존성 추가
4. 워크스페이스 재빌드

### 새로운 런치 파일 추가
1. `src/tr200_ros_control/launch/` 또는 `src/tr200_simple_control/launch/` 디렉토리에 `.launch` 파일 추가
2. `CMakeLists.txt`에 런치 파일 설치 추가
3. 파라미터 설정 확인
4. 테스트 실행

### 새로운 설정 파일 추가
1. `src/tr200_ros_control/config/` 또는 `src/tr200_simple_control/config/` 디렉토리에 `.yaml` 파일 추가
2. `CMakeLists.txt`에 config 디렉토리 설치 추가
3. 런치 파일에서 파라미터 로드 확인

## 🔒 보안 고려사항

- Docker 컨테이너는 `privileged` 모드로 실행됩니다
- 네트워크는 `host` 모드를 사용합니다
- TR200 로봇과의 통신은 WebSocket을 통해 이루어집니다

## 📞 지원

문제가 발생하거나 추가 기능이 필요한 경우:
1. 로그 파일 확인 (`docker/logs/` 디렉토리)
2. 설정 파일 검토 (`src/tr200_ros_control/config/`, `src/tr200_simple_control/config/` 디렉토리)
3. Woosh SDK 문서 참조 (`src/woosh_robot_py/README.md`)

### 주요 문제 해결
- **로봇 연결 실패**: TR200 앱에서 로봇 상태 확인 (비상정지 해제, 작업 모드 설정)
- **키보드 입력 문제**: 터미널 포커스 확인 및 노드 실행 상태 확인
- **센서 데이터 없음**: TR200의 라이다 센서 상태 확인
- **ROS 통신 문제**: `roscore` 실행 확인 및 네트워크 설정 검토
- **런치 파일 오류**: 패키지 빌드 및 파일 경로 확인
- **장애물 감지 개선**: 듀얼 센서 융합, 전방/후방 섹터 분석, 강화된 비상 정지 시스템

---

**개발자**: ldj  
**버전**: v1.0.0 (원격 제어 및 자동 제어 통합 시스템)  
**최종 업데이트**: 2025년 10월 01일