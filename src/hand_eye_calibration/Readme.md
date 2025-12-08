# Hand-Eye Calibration Package (Eye-to-Hand)

이 패키지는 ZED 2i 카메라와 UR5e 로봇팔을 이용한 Eye-to-Hand 캘리브레이션을 수행하기 위한 ROS 2 패키지입니다.

**Eye-to-Hand 구성**: 카메라는 고정되어 있고, 체커보드가 로봇 엔드이펙터에 부착되어 있습니다. 로봇을 다양한 위치로 이동시키면서 카메라가 체커보드를 관찰하여 로봇 베이스와 카메라 간의 변환 관계를 계산합니다.

## 📝 주요 기능

* **체커보드 감지 (`checkerboard_detector_node`)**: ZED 2i 카메라의 이미지에서 체커보드를 감지하여 `T_cam_board` 변환을 `/camera_to_checkerboard` 토픽으로 발행합니다.
* **TF to Pose 변환 (`tf_to_pose_publisher`)**: UR5e 로봇의 TF 정보(`/tf` 토픽)를 구독하여 `T_base_ee` 변환을 `/base_to_end_effector` 토픽으로 발행합니다.
* **캘리브레이션 수집기 (`calibration_collector_node`)**: 위 두 토픽을 동기화하여 구독하고, 서비스 호출을 통해 샘플 쌍을 수집한 뒤, `cv2.calibrateRobotWorldHandEye` 함수를 이용해 최종 `T_base_cam` 변환 행렬을 계산합니다.
* **Mock Robot (테스트용)**: 실제 로봇 없이 테스트하기 위한 가상 로봇 노드가 포함되어 있습니다.

## 📦 사전 요구사항

### 1. ZED SDK 설치 (필수)
```bash
# ZED SDK 다운로드 및 설치
# https://www.stereolabs.com/developers/release
# Ubuntu 22.04용 최신 버전 다운로드 후 설치
```

### 2. ZED ROS 2 Wrapper 설치 (필수)
```bash
# 별도 workspace에 설치
cd ~/zed_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ~/zed_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### 3. Python 패키지 설치 (필수)
```bash
# NumPy (특정 버전 필요)
pip install numpy==1.26.4

# OpenCV
pip install opencv-python

# YAML 파서
pip install pyyaml

# TF Transformations (특별 설치)
pip install git+https://github.com/DLu/tf_transformations.git
```

### 4. ROS 2 의존성
```bash
# 자동 설치 (이 저장소를 clone한 후)
cd ~/Hand_Eye_Calibration
sudo rosdep install --from-paths src --ignore-src -r -y
```

## 🚀 빌드

```bash
cd ~/Desktop/Hand_Eye_Calibration
colcon build --symlink-install
source install/setup.bash
```

## 📋 실행 방법 (실제 UR5e 로봇 사용)

### 준비사항
- UR5e 로봇이 실행 중이고 TF를 발행하고 있어야 함
- ZED 2i 카메라가 연결되어 있고, zed-ros2-wrapper가 실행 중이어야 함
- 체커보드가 로봇 엔드이펙터에 부착되어 있어야 함

### 실행 단계

#### **Terminal 1: ZED 카메라**
```bash
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

#### **Terminal 2: Checkerboard Detector**
```bash
source install/setup.bash
ros2 run hand_eye_calibration detector
```

**파라미터 조정 (선택사항):**
```bash
ros2 run hand_eye_calibration detector \
  --ros-args \
  -p checkerboard_rows:=6 \
  -p checkerboard_cols:=9 \
  -p square_size:=0.018
```

#### **Terminal 3: TF to Pose Publisher**
```bash
source install/setup.bash
ros2 run hand_eye_calibration tf_to_pose \
  --ros-args \
  -p parent_frame:=ur5e_base_link \
  -p child_frame:=ur5e_tool0
```

**참고:** Frame 이름은 로봇 설정에 따라 다를 수 있습니다. 다음 명령어로 확인:
```bash
ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>
```

#### **Terminal 4: Calibration Collector**
```bash
source install/setup.bash
ros2 run hand_eye_calibration collector
```

### 샘플 수집 및 캘리브레이션

#### **1. 샘플 수집 (10-20개 권장)**

로봇을 첫 번째 위치로 이동 → 정지 → 다음 명령 실행:

```bash
# Terminal 5
ros2 service call /capture_sample std_srvs/srv/Empty
```

**샘플 수집 가이드:**
- **다양한 위치**: X, Y, Z 축으로 골고루 이동
- **다양한 각도**: 로봇을 회전시켜 체커보드 방향 변경
- **완전 정지**: 각 위치에서 1-2초 대기 후 캡처
- **체커보드 가시성**: 모든 코너가 카메라에 보여야 함
- **최소 10개**: 더 많을수록 정확도 향상

위 과정을 10-20번 반복합니다.

#### **2. 캘리브레이션 실행**

충분한 샘플 수집 후:

```bash
ros2 service call /run_calibration std_srvs/srv/Empty
```

#### **3. 결과 확인**

- Terminal 4 (Collector)에 결과가 출력됩니다
- `calibration_result_YYYYMMDD_HHMMSS.yaml` 파일이 생성됩니다
- `calibration_samples.npz` 파일에 샘플 데이터가 저장됩니다

**결과 예시:**
```
=== Eye-to-Hand Calibration Results ===
T_base_cam (Robot Base → Camera):
  Translation [m]: [0.550, 0.200, 0.600]
  Orientation (xyzw): [0.000, 0.001, 0.707, 0.707]
  
Consistency Check:
  Mean residual: 2.5 mm ✅
  Max residual: 4.8 mm ✅
```

## 🧪 테스트 모드 (Mock Robot)

실제 로봇 없이 테스트하려면:

```bash
# Mock Robot 실행 (Terminal 3 대신)
ros2 run hand_eye_calibration mock_robot
```

**참고:** Mock robot은 테스트 목적.

## 📊 결과 파일

### `calibration_result_*.yaml`
```yaml
calibration_result:
  T_base_cam:
    translation:
      x: 0.550123
      y: 0.200456
      z: 0.600789
    rotation_quaternion:
      x: 0.000123
      y: 0.001234
      z: 0.707107
      w: 0.707107
```

### `calibration_samples.npz`
- NumPy 압축 파일
- `T_base_ee`: 로봇 포즈 배열
- `T_cam_board`: 카메라-체커보드 포즈 배열

## 🔧 문제 해결

### 체커보드가 감지되지 않을 때
```bash
# Detector 로그 확인
# "No checkerboard detected" 메시지가 계속 나오면:
# - 조명 확인
# - 체커보드 크기 확인 (config 파일 수정)
# - 카메라-체커보드 거리 조정
```

### TF를 찾을 수 없을 때
```bash
# Frame 이름 확인
ros2 run tf2_ros tf2_echo <parent> <child>

# 또는 전체 TF 트리 확인
ros2 run tf2_tools view_frames
evince frames.pdf
```

### 캘리브레이션 실패 시
```
[ERROR] Iterations do not converge
```
**원인**: 샘플 다양성 부족  
**해결**: 기존 샘플 삭제 후 더 다양한 위치/각도로 재수집
```bash
rm calibration_samples.npz
```

## 📖 추가 문서

- `QUICK_START.md`: 빠른 시작 가이드
- `wsl2_setup.md`: WSL2 환경 설정 (해당되는 경우)