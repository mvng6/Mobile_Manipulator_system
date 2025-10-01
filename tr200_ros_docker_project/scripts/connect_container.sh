#!/bin/bash
# Mobile Manipulator ROS Docker 컨테이너에 추가 터미널로 접속하는 스크립트

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔗 Mobile Manipulator ROS Docker 컨테이너에 추가 접속${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""

# 컨테이너 상태 확인
if ! docker ps | grep -q "mobile_manipulator_control_container"; then
    echo -e "${RED}❌ Mobile Manipulator ROS 컨테이너가 실행되지 않았습니다.${NC}"
    echo -e "${YELLOW}💡 먼저 './scripts/run_container.sh'를 실행하세요.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Mobile Manipulator ROS 컨테이너가 실행 중입니다${NC}"

# 컨테이너에 접속
echo -e "${YELLOW}🔗 컨테이너에 접속합니다...${NC}"
echo -e "${BLUE}💡 ROS 환경이 이미 설정되어 있습니다.${NC}"
echo -e "${BLUE}💡 바로 ROS 명령어를 사용할 수 있습니다!${NC}"
echo ""

# 컨테이너 접속 (ROS 환경 설정 및 /catkin_ws 경로로 이동)
docker exec -it mobile_manipulator_control_container bash -c "
    echo -e '${BLUE}🔧 ROS 환경을 자동 설정합니다...${NC}'
    cd /catkin_ws
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash 2>/dev/null || echo -e '${YELLOW}⚠️  devel/setup.bash가 없습니다. catkin_make를 실행하세요.${NC}'
    export PYTHONPATH=\"/catkin_ws/src/woosh_robot_py:\${PYTHONPATH}\"
    
    echo -e '${GREEN}✅ ROS 환경 설정 완료!${NC}'
    echo -e '${BLUE}🚀 ROS 환경 재설정: ./scripts/setup_ros.sh${NC}'
    echo -e '${BLUE}📁 현재 위치: \$(pwd)${NC}'
    echo ''
    exec bash
"
