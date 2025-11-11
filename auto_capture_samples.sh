#!/bin/bash
# auto_capture_samples.sh
# Hand-Eye Calibration 자동 샘플 수집 스크립트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 미리 정의된 포즈 배열 (x y z)
# 다양한 위치에서 샘플을 수집하여 캘리브레이션 품질 향상
POSES=(
    "0.5 0.0 0.3"
    "0.5 0.1 0.4"
    "0.6 0.0 0.3"
    "0.6 0.1 0.4"
    "0.4 0.1 0.3"
    "0.5 -0.1 0.4"
    "0.6 -0.1 0.3"
    "0.5 0.0 0.5"
    "0.5 0.1 0.2"
    "0.7 0.0 0.3"
    "0.4 0.0 0.4"
    "0.6 0.05 0.35"
)

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}   Hand-Eye Calibration 자동 샘플 수집${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""
echo -e "${GREEN}총 ${#POSES[@]}개 포즈에서 샘플 수집 예정${NC}"
echo ""

# ROS 2 노드 확인
echo -e "${YELLOW}[1/3] ROS 2 노드 상태 확인...${NC}"
if ! ros2 node list | grep -q "mock_robot_publisher"; then
    echo -e "${RED}❌ Mock robot publisher 노드가 실행되지 않았습니다!${NC}"
    echo -e "${YELLOW}먼저 launch 파일을 실행하세요:${NC}"
    echo -e "  ros2 launch hand_eye_calibration hand_eye_prep.launch.py"
    exit 1
fi

if ! ros2 node list | grep -q "calibration_collector"; then
    echo -e "${RED}❌ Calibration collector 노드가 실행되지 않았습니다!${NC}"
    exit 1
fi

echo -e "${GREEN}✅ 모든 노드가 실행 중입니다${NC}"
echo ""

# 샘플 수집 시작
echo -e "${YELLOW}[2/3] 샘플 수집 시작...${NC}"
echo ""

SAMPLE_NUM=1
SUCCESS_COUNT=0
FAIL_COUNT=0

for pose in "${POSES[@]}"; do
    # 공백으로 분리
    read -r x y z <<< "$pose"
    
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${YELLOW}📍 샘플 $SAMPLE_NUM/${#POSES[@]}${NC}"
    echo -e "   위치: x=${GREEN}$x${NC}, y=${GREEN}$y${NC}, z=${GREEN}$z${NC}"
    
    # 파라미터 설정
    echo -n "   파라미터 설정 중... "
    ros2 param set /mock_robot_publisher pose.position.x $x > /dev/null 2>&1
    ros2 param set /mock_robot_publisher pose.position.y $y > /dev/null 2>&1
    ros2 param set /mock_robot_publisher pose.position.z $z > /dev/null 2>&1
    echo -e "${GREEN}완료${NC}"
    
    # 안정화 대기 (로봇이 위치에 도달하고 진동이 멈출 때까지)
    echo -n "   시스템 안정화 대기 (1초)... "
    sleep 1
    echo -e "${GREEN}완료${NC}"
    
    # 샘플 수집
    echo -n "   💾 샘플 캡처 중... "
    if ros2 service call /capture_sample std_srvs/srv/Empty > /dev/null 2>&1; then
        echo -e "${GREEN}✅ 성공${NC}"
        ((SUCCESS_COUNT++))
    else
        echo -e "${RED}❌ 실패${NC}"
        ((FAIL_COUNT++))
    fi
    
    ((SAMPLE_NUM++))
    sleep 0.5
    echo ""
done

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${GREEN}✅ 샘플 수집 완료!${NC}"
echo -e "   성공: ${GREEN}$SUCCESS_COUNT${NC} / 실패: ${RED}$FAIL_COUNT${NC}"
echo ""

# 캘리브레이션 실행
if [ $SUCCESS_COUNT -ge 5 ]; then
    echo -e "${YELLOW}[3/3] 캘리브레이션 실행 중...${NC}"
    echo ""
    
    if ros2 service call /run_calibration std_srvs/srv/Empty; then
        echo ""
        echo -e "${BLUE}================================================${NC}"
        echo -e "${GREEN}🎉 캘리브레이션 완료!${NC}"
        echo -e "${BLUE}================================================${NC}"
        echo ""
        echo -e "${YELLOW}결과 파일 확인:${NC}"
        echo -e "  - calibration_samples.jsonl"
        echo -e "  - calibration_result_*.yaml"
        echo ""
        
        # 생성된 파일 표시
        if [ -f "calibration_samples.jsonl" ]; then
            SAMPLE_COUNT=$(wc -l < calibration_samples.jsonl)
            echo -e "${GREEN}📊 총 $SAMPLE_COUNT 개 샘플이 저장되었습니다${NC}"
        fi
        
        RESULT_FILE=$(ls -t calibration_result_*.yaml 2>/dev/null | head -1)
        if [ -n "$RESULT_FILE" ]; then
            echo -e "${GREEN}📄 최신 결과: $RESULT_FILE${NC}"
        fi
    else
        echo -e "${RED}❌ 캘리브레이션 실행 실패${NC}"
        exit 1
    fi
else
    echo -e "${RED}❌ 샘플이 부족합니다 (최소 5개 필요, 현재: $SUCCESS_COUNT)${NC}"
    echo -e "${YELLOW}더 많은 샘플을 수집한 후 수동으로 캘리브레이션을 실행하세요:${NC}"
    echo -e "  ros2 service call /run_calibration std_srvs/srv/Empty"
    exit 1
fi

echo ""
echo -e "${BLUE}================================================${NC}"
