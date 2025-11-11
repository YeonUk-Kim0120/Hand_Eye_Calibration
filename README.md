# Hand-Eye Calibration for ROS 2

ROS 2 Humble 기반 Hand-on-Base (Eye-to-Hand) 캘리브레이션 시스템

## 📋 시스템 요구사항

- **OS**: Ubuntu 22.04
- **ROS**: ROS 2 Humble
- **Python**: 3.10+
- **카메라**: ZED 2i (또는 다른 ROS 2 호환 카메라)

## 🔧 의존성

### Python 패키지
```bash
pip install numpy==1.26.4
pip install tf-transformations
pip install opencv-python
pip install pyyaml
```

### ROS 2 패키지
```bash
sudo apt install ros-humble-message-filters
```

## 📦 설치

### 1. 저장소 클론
```bash
cd ~
git clone <your-repo-url> hand_eye_ws
cd hand_eye_ws
```

### 2. 의존성 설치
```bash
# Python 패키지
pip install numpy==1.26.4 tf-transformations opencv-python pyyaml

# ROS 2 패키지
sudo apt install ros-humble-message-filters
```

### 3. 빌드
```bash
cd ~/hand_eye_ws
colcon build
source install/setup.bash
```

## 🚀 사용법

### 1. ZED 카메라 실행
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

### 2. 체커보드 감지 노드 실행
```bash
ros2 run hand_eye_calibration checkerboard_detector_node
```

### 3. 로봇 포즈 퍼블리셔 (실제 로봇 또는 Mock)
```bash
# Mock (테스트용)
ros2 run hand_eye_calibration mock_robot_publisher_node

# 실제 로봇의 경우 로봇 드라이버가 /base_to_end_effector 토픽을 publish해야 함
```

### 4. 캘리브레이션 수집 노드 실행
```bash
ros2 run hand_eye_calibration calibration_collector_node
```

### 5. 샘플 수집
로봇을 여러 포즈로 이동시키고 각 포즈에서:
```bash
ros2 service call /capture_sample std_srvs/srv/Empty
```

### 6. 캘리브레이션 실행
10-20개 샘플 수집 후:
```bash
ros2 service call /run_calibration std_srvs/srv/Empty
```

## 📁 프로젝트 구조

```
hand_eye_ws/
├── src/
│   └── hand_eye_calibration/
│       ├── hand_eye_calibration/
│       │   ├── checkerboard_detector_node.py      # 체커보드 감지
│       │   ├── calibration_collector_node.py      # 캘리브레이션 수집
│       │   └── mock_robot_publisher_node.py       # 테스트용 로봇 퍼블리셔
│       ├── launch/
│       │   └── hand_eye_prep.launch.py
│       ├── config/
│       │   └── checkerboard_params.yaml           # 체커보드 설정
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── README.md
```

## ⚙️ 설정

### 체커보드 파라미터 (`config/checkerboard_params.yaml`)
```yaml
checkerboard:
  rows: 6          # 내부 코너 행 수
  cols: 9          # 내부 코너 열 수
  square_size: 0.021  # 사각형 크기 (m)
```

## 🔬 캘리브레이션 방식

- **타입**: Hand-on-Base (Eye-to-Hand)
- **카메라**: 베이스에 고정
- **체커보드**: 로봇 엔드 이펙터에 부착
- **알고리즘**: `cv2.calibrateRobotWorldHandEye` (Shah 방법)

### 입력
- `T_base_ee`: 베이스 → 엔드 이펙터 변환
- `T_cam_board`: 카메라 → 체커보드 변환

### 출력
- `T_base_cam`: 베이스 → 카메라 변환 (고정)
- `T_ee_board`: 엔드 이펙터 → 체커보드 변환

## 📊 결과 파일

캘리브레이션 완료 시:
- `calibration_result_YYYYMMDD_HHMMSS.yaml`: 최종 결과
- `calibration_samples.npz`: 수집된 샘플 데이터

## 🐛 문제 해결

### NumPy 2.x 호환성 문제
```bash
pip uninstall numpy
pip install "numpy<2.0"
```

### ZED 카메라 실행 안됨
```bash
# 올바른 모델명 사용
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

## 📝 노트

- 최소 5개, 권장 10-20개 샘플 필요
- 샘플은 다양한 로봇 포즈에서 수집 (회전 포함)
- 체커보드는 카메라 시야에 완전히 들어와야 함
- 캘리브레이션 샘플은 자동으로 `calibration_samples.npz`에 저장됨

## 📄 라이센스

MIT License

## 👤 작성자

KIST Hand-Eye Calibration Project
