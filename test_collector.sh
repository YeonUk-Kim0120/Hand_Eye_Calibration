#!/bin/bash
# Collector 점검 스크립트

cd /home/kist/Desktop/hand_eye_ws
source install/setup.bash

echo "============================================="
echo "Collector 점검 스크립트"
echo "============================================="
echo ""

echo "1. 현재 실행 중인 노드 확인..."
ros2 node list 2>/dev/null || echo "노드 없음"
echo ""

echo "2. 필요한 토픽 확인..."
echo "  - /base_to_end_effector:"
ros2 topic info /base_to_end_effector 2>/dev/null || echo "    토픽 없음"
echo "  - /camera_to_checkerboard:"
ros2 topic info /camera_to_checkerboard 2>/dev/null || echo "    토픽 없음"
echo ""

echo "3. Collector가 기대하는 동작:"
echo "  - 두 토픽을 ApproximateTimeSynchronizer로 동기화 (slop=0.1s)"
echo "  - /capture_sample 서비스로 현재 포즈 쌍 저장"
echo "  - /run_calibration 서비스로 캘리브레이션 실행"
echo ""

echo "============================================="
echo "사용자가 직접 실행할 명령어:"
echo "============================================="
echo ""
echo "터미널 1 (Detector):"
echo "  cd /home/kist/Desktop/hand_eye_ws"
echo "  source install/setup.bash"
echo "  ros2 run hand_eye_calibration detector --ros-args --params-file src/hand_eye_calibration/config/checkerboard_params.yaml"
echo ""
echo "터미널 2 (Collector):"
echo "  cd /home/kist/Desktop/hand_eye_ws"
echo "  source install/setup.bash"
echo "  ros2 run hand_eye_calibration collector"
echo ""
echo "터미널 3 (샘플 캡처):"
echo "  cd /home/kist/Desktop/hand_eye_ws"
echo "  source install/setup.bash"
echo "  ros2 service call /capture_sample std_srvs/srv/Empty"
echo ""
echo "터미널 4 (캘리브레이션 실행, 5개 이상 샘플 수집 후):"
echo "  ros2 service call /run_calibration std_srvs/srv/Empty"
echo ""
