# TR200 ROS Control Package

TR200 로봇을 ROS Noetic 환경에서 SDK와 연동하여 키보드 입력을 통한 원격 제어를 제공하는 패키지입니다.

## 📋 개요

이 패키지는 TR200 로봇을 ROS 환경에서 제어하기 위한 통합 솔루션을 제공합니다. SDK를 통한 로봇 연결, 키보드 입력 처리, 안전 제어, 원격 제어 등의 기능을 포함합니다.

## 🏗️ 시스템 구조

### 노드 구성
- **Robot Connection Node**: TR200 로봇과의 SDK 연결 관리
- **Remote Control Node**: 키보드 입력을 통한 원격 제어
- **Robot Driver Node**: 로봇 구동 제어 및 명령 실행

### ROS 토픽
- `/cmd_vel`: 로봇 속도 명령
- `/remote_cmd_vel`: 원격 제어 명령
- `/connection_status`: 로봇 연결 상태
- `/robot_status`: 로봇 상태 정보

### ROS 서비스
- `/set_safety_params`: 안전 파라미터 설정
- `/get_robot_status`: 로봇 상태 조회

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

### 3. 원격 제어 실행

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

### 4. 키보드 제어

터미널을 클릭한 후 다음 키를 사용하여 로봇을 제어할 수 있습니다:

#### 기본 이동 제어
- **`w`** - 전진 (앞으로 이동)
- **`s`** - 후진 (뒤로 이동)
- **`a`** - 좌회전 (왼쪽으로 회전하며 이동)
- **`d`** - 우회전 (오른쪽으로 회전하며 이동)

#### 제자리 회전
- **`q`** - 제자리 좌회전 (현재 위치에서 왼쪽으로 회전)
- **`e`** - 제자리 우회전 (현재 위치에서 오른쪽으로 회전)

#### 제어 명령
- **`스페이스`** - 정지 (즉시 정지)
- **`x`** - 비상정지 (긴급 정지)

#### 시스템 명령
- **`h`** - 도움말 표시
- **`c`** - 연결 상태 확인
- **`r`** - 시스템 리셋

#### 종료
- **`Ctrl+C`** - 시스템 종료

## 📁 패키지 구조

```
tr200_ros_control/
├── CMakeLists.txt                    # 빌드 설정
├── package.xml                      # 패키지 매니페스트
├── README.md                        # 패키지 문서
├── launch/                          # ROS 런치 파일
│   ├── tr200_remote_control.launch      # 통합 원격 제어 런치
│   ├── tr200_base_system.launch         # 기본 시스템 런치
│   ├── tr200_keyboard_control.launch    # 키보드 제어 런치
│   └── tr200_keyboard_control_safe.launch # 안전 모드 런치
├── config/                          # 설정 파일
│   ├── robot_params.yaml               # 로봇 기본 파라미터
│   ├── safety_params.yaml             # 안전 제어 파라미터
│   └── sensor_params.yaml             # 센서 파라미터
├── scripts/                         # Python 제어 스크립트
│   ├── robot_connection_node.py        # 로봇 연결 관리 노드
│   ├── remote_control_node.py         # 원격 제어 노드
│   └── robot_driver_node.py           # 로봇 구동 제어 노드
├── srv/                             # ROS 서비스 정의
│   ├── SetSafetyParams.srv             # 안전 파라미터 설정 서비스
│   └── GetRobotStatus.srv              # 로봇 상태 조회 서비스
└── msg/                             # ROS 메시지 정의
    ├── SafetyStatus.msg                # 안전 상태 메시지
    └── RobotStatus.msg                 # 로봇 상태 메시지
```

## ⚙️ 설정

### 로봇 연결 설정

`config/robot_params.yaml`에서 로봇 연결 정보를 설정할 수 있습니다:

```yaml
robot:
  ip: "169.254.128.2"  # TR200 로봇 IP
  port: 5480           # TR200 로봇 포트
  identity: "tr200_ros_controller"
```

### 원격 제어 설정

런치 파일에서 원격 제어 파라미터를 조정할 수 있습니다:

```bash
roslaunch tr200_ros_control tr200_remote_control.launch \
  linear_speed:=0.3 \
  angular_speed:=0.3 \
  command_timeout:=3.0
```

### 안전 설정

`config/safety_params.yaml`에서 안전 제어 파라미터를 설정할 수 있습니다:

```yaml
sensor:
  min_obstacle_distance: 0.5    # 최소 장애물 거리
  warning_distance: 0.8         # 경고 거리
  safe_distance: 1.0           # 안전 거리

control:
  normal_speed: 0.2            # 정상 속도
  slow_speed: 0.1              # 감속 속도
  stop_speed: 0.0              # 정지 속도
```

## 🔧 개발

### 새로운 노드 추가

1. `scripts/` 디렉토리에 새 노드 파일 생성
2. `CMakeLists.txt`에 스크립트 설치 추가
3. 필요시 런치 파일에 노드 추가

### 메시지/서비스 추가

1. `msg/` 또는 `srv/` 디렉토리에 새 파일 생성
2. `CMakeLists.txt`에 메시지/서비스 추가
3. `package.xml`에 의존성 추가
4. `catkin_make` 실행

## 📊 모니터링

### 토픽 모니터링

```bash
# 연결 상태 확인
rostopic echo /connection_status

# 원격 제어 명령 확인
rostopic echo /remote_cmd_vel

# 로봇 상태 확인
rostopic echo /robot_status
```

### 노드 상태 확인

```bash
# 실행 중인 노드 확인
rosnode list

# 노드 정보 확인
rosnode info /robot_connection_node
rosnode info /remote_control_node
rosnode info /robot_driver_node
```

### 서비스 사용

```bash
# 안전 파라미터 동적 변경
rosservice call /set_safety_params "min_obstacle_distance: 0.3
warning_distance: 0.6
safe_distance: 0.9
normal_speed: 0.15
slow_speed: 0.05"

# 로봇 상태 조회
rosservice call /get_robot_status "request: true"
```

## 🚨 안전 주의사항

### 첫 실행 시
1. **낮은 속도로 테스트**: `linear_speed:=0.1, angular_speed:=0.1`
2. **충분한 공간 확보**: 로봇 주변에 장애물이 없는지 확인
3. **비상정지 준비**: 언제든지 `x` 키를 누를 수 있도록 준비

### 일반 사용 시
1. **로봇 연결 상태 확인**: `c` 키로 연결 상태 확인
2. **부드러운 조작**: 급격한 방향 전환 피하기
3. **정기적 정지**: `스페이스` 키로 주기적 정지

## 🐛 트러블슈팅

### 로봇 연결 실패

1. 로봇 IP 주소 확인
2. 네트워크 연결 상태 확인
3. 로봇 전원 상태 확인
4. 방화벽 설정 확인

### 키보드 입력이 작동하지 않음

1. 터미널을 클릭하여 포커스 확인
2. 키보드 권한 확인
3. 노드 실행 상태 확인

### 센서 데이터 수신 안됨

1. 로봇 연결 상태 확인
2. 센서 하드웨어 상태 확인
3. 토픽 구독 상태 확인

## 📞 지원

문제가 발생하면 다음을 확인하세요:

1. 로그 메시지 확인
2. ROS 토픽 상태 확인
3. 노드 실행 상태 확인
4. 네트워크 연결 상태 확인

## 📄 라이선스

MIT License

## 👥 기여자

- User (KATECH 연구원) - 프로젝트 설계 및 구현

---

**버전**: v1.0.0  
**최종 업데이트**: 2025년 10월 01일