# Hand-Eye Calibration Package

이 패키지는 ZED 2i 카메라와 UR5e 로봇팔을 이용한 Hand-Eye 캘리브레이션을 수행하기 위한 ROS 2 패키지입니다.
**Eye-on-Base**와 **Eye-in-Hand** 두 가지 모드를 모두 지원합니다.

- **Eye-on-Base**: 카메라 고정, 체커보드가 로봇 EE에 부착 → `T_base_cam` (base → camera) 계산
- **Eye-in-Hand**: 카메라가 로봇 EE에 부착, 체커보드 고정 → `T_ee_cam` (EE → camera) 계산

## 📝 주요 기능

* **체커보드 감지 (`checkerboard_detector_node`)**: ZED 2i 카메라의 이미지에서 체커보드를 감지하여 `T_cam_board` 변환을 `/camera_to_checkerboard` 토픽으로 발행합니다.
* **TF to Pose 변환 (`tf_to_pose_publisher`)**: UR5e 로봇의 TF 정보(`/tf` 토픽)를 구독하여 `T_base_ee` 변환을 `/base_to_end_effector` 토픽으로 발행합니다.
* **캘리브레이션 수집기 (`calibration_collector_node`)**: 위 두 토픽을 동기화하여 구독하고, 서비스 호출을 통해 샘플 쌍을 수집한 뒤, `cv2.calibrateHandEye` 함수를 이용해 캘리브레이션을 계산합니다. `calibration_type` 파라미터로 `eye_on_base` / `eye_in_hand` 모드를 선택할 수 있습니다.
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
pip install transforms3d
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

#### **Terminal 1: ZED Wrapper 실행**
```bash
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

#### **Terminal 2: Checkerboard Detector**
```bash
source install/setup.bash
ros2 run hand_eye_calibration detector
```

**파라미터 조정 명령어 (선택사항):**
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

**참고:** Frame 이름은 로봇 설정에 따라 다를 수 있습니다.(일반적으론 base_link, tool0) 다음 명령어로 확인:
```bash
ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>
```

#### **Terminal 4: Calibration Collector**
```bash
source install/setup.bash

# Eye-on-Base 모드 (카메라 고정, 보드가 EE에 부착)
ros2 run hand_eye_calibration collector --ros-args -p calibration_type:=eye_on_base

# Eye-in-Hand 모드 (카메라가 EE에 부착, 보드 고정)
ros2 run hand_eye_calibration collector --ros-args -p calibration_type:=eye_in_hand
```

### 샘플 수집 및 캘리브레이션
#### 단위는 m(미터) 
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

**결과 예시 (Eye-on-Base):**
```
=== Eye-on-Base Calibration Results ===
🎯 T_base_cam (Robot Base → Camera) - PRIMARY RESULT
   Translation [m]: [0.550, 0.200, 0.600]
   Orientation (xyzw): [0.000, 0.001, 0.707, 0.707]
📊 Consistency Check:
   Mean residual: 2.5 mm ✅
```

**결과 예시 (Eye-in-Hand):**
```
=== Eye-in-Hand Calibration Results ===
🎯 T_ee_cam (End-Effector → Camera) - PRIMARY RESULT
   Translation [m]: [0.120, 0.210, 0.137]
   Orientation (xyzw): [0.000, 0.000, 0.000, 1.000]
📊 Consistency Check:
   Mean residual: 1.2 mm ✅
```

## 🧪 테스트 모드 (Mock Robot)

실제 로봇 없이 테스트하려면:

#### **Terminal 1: ZED Wrapper 실행**
```bash
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

#### **Terminal 2: Checkerboard Detector**
```bash
source install/setup.bash
ros2 run hand_eye_calibration detector
```

#### **Terminal 3: Mock Robot (가상 로봇 TF 발행)**
```bash
source install/setup.bash
ros2 run hand_eye_calibration mock_robot
```

#### **Terminal 4: TF to Pose Publisher**
```bash
source install/setup.bash
ros2 run hand_eye_calibration tf_to_pose \
  --ros-args \
  -p parent_frame:=ur5e_base_link \
  -p child_frame:=ur5e_tool0
```

#### **Terminal 5: Calibration Collector**
```bash
source install/setup.bash
ros2 run hand_eye_calibration collector --ros-args -p calibration_type:=eye_on_base
```

#### **샘플 수집 방법:**
```bash
# Terminal 6에서 포즈 파라미터 변경 후 캡처를 반복합니다

# 1) Mock Robot 포즈 변경
ros2 param set /mock_robot_publisher_node pose.position.x 0.6
ros2 param set /mock_robot_publisher_node pose.position.y 0.1
ros2 param set /mock_robot_publisher_node pose.position.z 0.4

# 2) 샘플 캡처
ros2 service call /capture_sample std_srvs/srv/Empty

# 3) 포즈를 다시 변경하고 캡처를 반복 (최소 10회)
```

## 검증 모드
#### 사전: ZED + UR driver + (선택) detector 모두 켜진 상태에서                         
source install/setup.bash                                                              
                                                                                         
#### 가장 최신 calibration_result_*.yaml 자동 선택                                        
ros2 run hand_eye_calibration verify                                                   

#### 특정 YAML 지정하려면                                                               
ros2 run hand_eye_calibration verify --ros-args -p calibration_file:=/home/kist/Desktop/Hand_Eye_Calibration/calibration_result_20251210_112241.yaml

**참고:** Mock robot은 실제 로봇처럼 TF를 발행하므로 tf_to_pose와 함께 사용해야 합니다.
