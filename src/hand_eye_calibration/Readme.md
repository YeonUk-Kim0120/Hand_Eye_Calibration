# Hand-Eye Calibration Package (Eye-to-Hand)

이 패키지는 ZED 2i 카메라와 로봇팔을 이용한 Hand-on-Base (Eye-to-Hand) 캘리브레이션을 수행하기 위한 ROS 2 패키지입니다.

현재 로봇팔 없이 카메라 측 컴퓨터에서 로직을 개발하고 테스트하기 위해 '가상 로봇팔 노드'를 포함하고 있습니다.

## 📝 주요 기능

* **체커보드 감지 (`checkerboard_detector_node`)**: ZED 2i 카메라의 이미지에서 체커보드를 감지하여 `$T_{cam \to board}$` 변환 행렬을 `/camera_to_checkerboard` 토픽으로 발행합니다.
* **가상 로봇팔 (`mock_robot_publisher_node`)**: 실제 로봇팔의 `$T_{base \to ee}$` 변환 행렬을 `/base_to_end_effector` 토픽으로 발행하여 시뮬레이션합니다.
* **캘리브레이션 수집기 (`calibration_collector_node`)**: 위 두 토픽을 동기화하여 구독하고, 서비스 호출을 통해 샘플 쌍을 수집한 뒤, `cv2.calibrateHandEye` 함수를 이용해 최종 `$T_{base \to cam}$` 변환 행렬을 계산합니다.

## 📦 의존성

* `rclpy`
* `zed-ros2-wrapper`
* `sensor_msgs`
* `geometry_msgs`
* `std_srvs`
* `cv_bridge`
* `message_filters`
* `numpy`
* `opencv-python`
* `tf-transformations`

## 🚀 빌드 및 실행

1.  **빌드**:
    ```bash
    cd ~/hand_eye_ws
    colcon build --packages-select hand_eye_calibration
    source install/setup.bash
    ```

2.  **실행 (테스트 환경)**:
    * 모든 노드 (ZED, Detector, Mock Robot, Collector)를 한 번에 실행합니다.
    ```bash
    ros2 launch hand_eye_calibration hand_eye_prep.launch.py
    ```

## 📋 사용법 (테스트 워크플로우)

1.  위의 `launch` 명령어로 모든 노드를 실행합니다.
2.  새 터미널을 열고 `/capture_sample` 서비스를 호출하여 첫 번째 샘플을 저장합니다.
    ```bash
    ros2 service call /capture_sample std_srvs/srv/Empty
    ```
3.  `hand_eye_prep.launch.py` 파일 (또는 CLI)에서 `mock_robot_node`의 파라미터(가상 포즈)를 수정합니다.
4.  `launch` 파일을 다시 실행합니다. (Ctrl+C 후 다시 실행)
5.  다시 `/capture_sample` 서비스를 호출하여 두 번째 샘플을 저장합니다.
6.  3~5번 과정을 10~15회 이상 반복하여 서로 다른 위치의 샘플을 수집합니다.
7.  모든 샘플이 수집되면, `/run_calibration` 서비스를 호출하여 캘리브레이션을 실행합니다.
    ```bash
    ros2 service call /run_calibration std_srvs/srv/Empty
    ```
8.  `collector` 노드가 실행된 터미널(1번 터미널)에 최종 계산된 `$T_{base \to cam}$` 변환 행렬이 출력됩니다.