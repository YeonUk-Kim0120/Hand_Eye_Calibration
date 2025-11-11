# Hand-Eye Calibration 사용 가이드

## 🎯 개요

이 패키지는 **Hand-on-Base (Eye-to-Hand)** 방식의 캘리브레이션을 수행합니다.
- **카메라**: 고정 위치 (베이스에 설치)
- **체커보드**: 로봇 엔드 이펙터에 부착
- **목표**: 베이스 좌표계에서 카메라까지의 변환 행렬 `T_base_cam` 계산

---

## 📁 프로젝트 구조 설명

### 1. **Python 노드 파일** (`hand_eye_calibration/`)

#### `checkerboard_detector_node.py`
- **역할**: ZED 카메라 이미지에서 체커보드 검출
- **입력**: 
  - `/zed/zed_node/rgb/image_rect_color` (이미지)
  - `/zed/zed_node/rgb/camera_info` (카메라 내부 파라미터)
- **출력**: 
  - `/camera_to_checkerboard` (PoseStamped) - 카메라에서 체커보드까지의 변환
- **기능**:
  - OpenCV로 체커보드 코너 검출
  - `solvePnP`로 포즈 추정
  - 실시간 시각화 (체커보드 코너, 좌표축, 거리 정보)

#### `mock_robot_publisher_node.py`
- **역할**: 실제 로봇 대신 가상 로봇 팔 시뮬레이션
- **출력**: 
  - `/base_to_end_effector` (PoseStamped) - 베이스에서 엔드 이펙터까지의 변환
- **파라미터**:
  - `pose.position.[x,y,z]` - 위치 (미터)
  - `pose.orientation.[x,y,z,w]` - 방향 (쿼터니언)
- **특징**: ROS2 파라미터로 포즈 변경 가능

#### `calibration_collector_node.py`
- **역할**: 샘플 수집 및 캘리브레이션 실행
- **입력**: 
  - `/base_to_end_effector`
  - `/camera_to_checkerboard`
- **서비스**:
  - `/capture_sample` - 현재 포즈 쌍을 샘플로 저장
  - `/run_calibration` - 수집된 샘플로 캘리브레이션 실행
- **기능**:
  - 샘플 자동 저장/불러오기 (`calibration_samples.npz`)
  - OpenCV `calibrateHandEye()` 사용
  - 결과를 YAML 파일로 저장 (`calibration_result_YYYYMMDD_HHMMSS.yaml`)

### 2. **설정 파일** (`config/`)

#### `checkerboard_params.yaml`
```yaml
checkerboard_rows: 6       # 체커보드 내부 코너 세로 개수
checkerboard_cols: 9       # 체커보드 내부 코너 가로 개수
square_size: 0.025         # 한 칸 크기 (미터)
```

#### `poses.yaml`
- 자동화 스크립트용 로봇 포즈 목록
- 각 포즈는 `position: [x,y,z]`, `orientation: [qx,qy,qz,qw]` 형식

### 3. **Launch 파일** (`launch/`)

#### `hand_eye_prep.launch.py`
- ZED 카메라, 체커보드 검출기, Mock 로봇, Collector를 한 번에 실행
- ZED Wrapper는 `/home/kist/ros2_ws`에 설치된 것을 사용

### 4. **자동화 스크립트** (`scripts/`)

#### `auto_capture.py`
- **키보드 제어 방식**의 자동 샘플 수집 스크립트
- `poses.yaml` 파일에서 포즈 목록을 읽어 순차 실행
- 각 포즈마다 사용자가 Enter 키로 확인 후 진행

### 5. **빌드 설정 파일**

#### `setup.py`
- Python 패키지 설정
- **entry_points**: 실행 가능한 노드 등록
  - `detector` → `checkerboard_detector_node:main`
  - `mock_robot` → `mock_robot_publisher_node:main`
  - `collector` → `calibration_collector_node:main`
- **data_files**: launch, config, scripts 파일 설치 위치 지정

#### `setup.cfg`
- 패키지 메타데이터 및 빌드 옵션
- 코드 스타일 검사(flake8), 테스트(pytest) 설정

#### `package.xml`
- ROS 2 패키지 메타데이터
- **의존성 선언**:
  - `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_srvs`
  - `cv_bridge`, `message_filters`
  - `zed_ros2_wrapper` (exec_depend)

#### `__init__.py`
- Python 패키지 인식용 빈 파일
- 향후 패키지 레벨 초기화 코드 추가 가능

---

## 🚀 사용 방법

### **방법 1: 수동 샘플 수집 (기본)**

#### 1단계: 시스템 실행
```bash
cd ~/Desktop/hand_eye_ws
source install/setup.bash
ros2 launch hand_eye_calibration hand_eye_prep.launch.py
```

실행되는 노드:
- ZED 카메라 노드
- 체커보드 검출기 (시각화 윈도우 표시)
- Mock 로봇 퍼블리셔
- 캘리브레이션 수집기

#### 2단계: 샘플 수집 (10~15개 권장)

**새 터미널에서:**
```bash
source ~/Desktop/hand_eye_ws/install/setup.bash

# 샘플 1 캡처
ros2 service call /capture_sample std_srvs/srv/Empty
```

**로봇 포즈 변경 (파라미터 수정):**
```bash
ros2 param set /mock_robot_publisher pose.position.x 0.6
ros2 param set /mock_robot_publisher pose.position.y 0.1
ros2 param set /mock_robot_publisher pose.position.z 0.5

# 샘플 2 캡처
ros2 service call /capture_sample std_srvs/srv/Empty
```

이 과정을 다양한 포즈에서 반복 (10~15회)

#### 3단계: 캘리브레이션 실행
```bash
ros2 service call /run_calibration std_srvs/srv/Empty
```

결과는 launch를 실행한 터미널에 출력되며, `calibration_result_YYYYMMDD_HHMMSS.yaml` 파일로 저장됩니다.

---

### **방법 2: 자동 샘플 수집 (권장) ⭐**

#### 1단계: 시스템 실행 (동일)
```bash
cd ~/Desktop/hand_eye_ws
source install/setup.bash
ros2 launch hand_eye_calibration hand_eye_prep.launch.py
```

#### 2단계: 자동화 스크립트 실행

**새 터미널에서:**
```bash
source ~/Desktop/hand_eye_ws/install/setup.bash
python3 ~/Desktop/hand_eye_ws/src/hand_eye_calibration/scripts/auto_capture.py
```

또는 설치된 경로에서:
```bash
cd ~/Desktop/hand_eye_ws/install/hand_eye_calibration/share/hand_eye_calibration/scripts
python3 auto_capture.py
```

#### 3단계: 키보드로 제어

스크립트가 각 포즈를 순서대로 제시하며, 다음 명령을 입력:

- **Enter** - 다음 단계 진행 (포즈 설정 → 샘플 캡처)
- **s** - 현재 포즈 건너뛰기
- **q** - 종료
- **c** - 지금 바로 캘리브레이션 실행

**워크플로우:**
1. 스크립트가 포즈 정보 표시
2. Enter로 포즈 설정 확인
3. 시각화 윈도우에서 체커보드 검출 확인
4. Enter로 샘플 캡처
5. 다음 포즈로 자동 이동
6. 충분한 샘플 수집 후 자동으로 캘리브레이션 제안

---

## 📊 샘플 저장 및 불러오기

### 자동 저장
- 각 샘플 캡처 시 `calibration_samples.npz` 파일에 자동 저장됨
- 노드를 종료해도 샘플이 보존됨

### 샘플 불러오기
- `collector` 노드 시작 시 기존 `calibration_samples.npz` 자동 로드
- 중단했다가 재개하여 샘플을 추가 수집 가능

### 샘플 초기화 (새로 시작)
```bash
# 기존 샘플 파일 삭제
rm calibration_samples.npz
```

---

## 🎨 시각화 기능

### 체커보드 검출 윈도우 (OpenCV)

자동으로 표시되는 정보:
- ✅ **체커보드 코너**: 빨간색 점과 선
- ✅ **좌표축**:
  - 빨간색 선: X축
  - 초록색 선: Y축
  - 파란색 선: Z축
- ✅ **거리 정보**: 카메라에서 체커보드까지 거리 (미터)
- ✅ **포즈 정보**: X, Y, Z 위치 값

### 시각화 끄기
```bash
# launch 파일 수정 또는 파라미터 변경
ros2 param set /checkerboard_detector show_visualization False
```

---

## 🔧 문제 해결

### 1. 체커보드가 검출되지 않음
- ✅ `checkerboard_params.yaml`에서 `rows`와 `cols` 값 확인
- ✅ 조명 개선 (그림자 제거, 균일한 조명)
- ✅ 체커보드가 평평하고 구겨지지 않았는지 확인

### 2. ZED 카메라 토픽이 안 보임
```bash
ros2 topic list | grep zed
```
토픽이 없으면:
- ZED SDK 설치 확인
- ZED 카메라 USB 연결 확인
- `/home/kist/ros2_ws`의 setup.bash 소싱 확인

### 3. 샘플 수집이 안 됨
- ✅ 두 토픽이 모두 발행되는지 확인:
  ```bash
  ros2 topic echo /camera_to_checkerboard
  ros2 topic echo /base_to_end_effector
  ```
- ✅ Collector 노드 로그 확인 (동기화 메시지)

### 4. 캘리브레이션 결과가 이상함
- ✅ 최소 10~15개 샘플 수집
- ✅ 다양한 위치와 방향에서 샘플 수집
- ✅ 체커보드가 이미지 전체 영역에 골고루 분포하도록 배치

---

## 📝 중요 개념 정리

### Hand-on-Base vs Eye-in-Hand

| 구분 | Hand-on-Base (이 프로젝트) | Eye-in-Hand |
|------|---------------------------|-------------|
| 카메라 위치 | 고정 (베이스) | 로봇 손목 |
| 체커보드 위치 | 엔드 이펙터 | 고정 |
| 계산 목표 | `T_base_cam` | `T_ee_cam` |
| 장점 | 넓은 작업 영역 | 정밀 작업 |

### 변환 관계식
```
T_base_ee * T_ee_board = T_base_cam * T_cam_board
       A    *     Y     =     X      *      B

A (알고 있음): 로봇 FK (base → end-effector)
B (측정): 카메라 비전 (camera → checkerboard)
X (목표): 캘리브레이션 결과 (base → camera)
Y (부산물): end-effector → checkerboard
```

### 필요한 샘플 개수
- **최소**: 5개 (이론적 최소)
- **권장**: 10~15개 (실용적)
- **최적**: 20개 이상 (고정밀)

---

## 📦 의존성 정리

### ROS 2 패키지
```bash
sudo apt-get install \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf-transformations
```

### Python 패키지
```bash
pip3 install opencv-python opencv-contrib-python pyyaml numpy
```

### ZED ROS 2 Wrapper
- 이미 `/home/kist/ros2_ws`에 설치됨
- 새로 설치 불필요

---

## 💡 팁과 Best Practices

### 1. 체커보드 크기 측정
- 정확한 `square_size` 값이 중요합니다
- 자로 여러 칸을 측정하여 평균값 사용

### 2. 샘플 수집 전략
- ✅ 작업 영역 전체를 커버하도록 포즈 선택
- ✅ 체커보드가 이미지의 다양한 위치에 나타나도록
- ✅ 회전 변화도 포함 (z축 회전 변화)

### 3. 포즈 YAML 파일 수정
`config/poses.yaml`을 편집하여:
- 포즈 추가/삭제
- 위치/방향 값 조정
- 실제 로봇의 작업 영역에 맞게 설정

### 4. 결과 검증
캘리브레이션 후:
- 결과 행렬의 translation 값이 실제 거리와 유사한지 확인
- 여러 번 실행하여 재현성 확인
- RMS 에러 값 확인 (로그에 출력)

---

## 🎓 다음 단계

### 실제 로봇 연동
1. `mock_robot_publisher_node` 대신 실제 로봇 FK publisher 구현
2. 안전 검사 추가 (충돌 회피, 속도 제한)
3. MoveIt2 또는 로봇 제어기와 통합

### 정밀도 향상
1. 더 큰 체커보드 사용
2. 더 많은 샘플 수집
3. ArUco 마커 사용 고려
4. 카메라 재캘리브레이션

---

## 📞 문의 및 기여

- **Maintainer**: Yeonuk Kim
- **Email**: kimyeonuk0120@gmail.com
- **License**: Apache-2.0

---

**마지막 업데이트**: 2025년 11월 4일
