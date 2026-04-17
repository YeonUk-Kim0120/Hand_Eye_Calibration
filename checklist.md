🔴 CRITICAL: 기존 .npz 샘플이 자동 로드됨 (가장 놓치기 쉬운 함정)

  현재 워크스페이스에 calibration_samples_eye_on_base.npz가 이미 있고,
  collector의 load_on_start=True가 기본값입니다. 현장에서 collector를
  띄우면:
  - 과거 테스트 샘플을 자동으로 읽어들임
  - 새 샘플과 섞여서 캘리브레이션이 엉뚱하게 나옴

  대책 (셋 중 하나):
  # ① 기존 샘플 이동
  mv calibration_samples.npz calibration_samples_old.npz

  # ② 실행 시 로드 끄기
  ros2 run hand_eye_calibration collector --ros-args \
    -p calibration_type:=eye_on_base \
    -p load_on_start:=false

  # ③ 다른 파일명 쓰기
  ros2 run hand_eye_calibration collector --ros-args \
    -p calibration_type:=eye_on_base \
    -p samples_file:=onsite_$(date +%Y%m%d).npz

  주의: 기본 samples_file이 calibration_samples.npz(일반명)이므로, 앞서
  eye_in_hand 모드를 돌렸다면 그 파일이 이미 생겨 있을 가능성도
  있습니다.
#############################################################################
  🔴 CRITICAL: ZED 토픽명이 하드코딩됨
#############################################################################
  checkerboard_detector_node.py:68-69:
  image_sub = message_filters.Subscriber(self, Image,
  '/zed/zed_node/rgb/image_rect_color')
  info_sub = message_filters.Subscriber(self, CameraInfo,
  '/zed/zed_node/rgb/camera_info')
  파라미터로 바꿀 수 없습니다. 현장 ZED wrapper 네임스페이스가 다르면
  detector가 영원히 대기합니다. 대책:

  # 현장 토픽 확인
  ros2 topic list | grep zed

  # 다르면 remap
  ros2 run hand_eye_calibration detector --ros-args \
    -r /zed/zed_node/rgb/image_rect_color:=<실제_이미지_토픽> \
    -r /zed/zed_node/rgb/camera_info:=<실제_info_토픽>

  반드시 image_rect_color (rectified) 를 써야 합니다. image_color
  (raw)를 쓰면 ZED wrapper가 D 벡터를 0으로 주더라도 실제 영상엔 왜곡이
  남아있어서 solvePnP 결과가 망가집니다.

  🔴 CRITICAL: 체커보드 파라미터 현장 것과 일치 확인

  기본값: checkerboard_rows=6, checkerboard_cols=9, square_size=0.018(m)

  - **rows/cols는 "내부 코너 수"**입니다 (칸 수가 아님). 예: 10x7
  사각형이 그려진 보드 → 내부 코너는 9x6
  - 제조사 스펙이 아닌 실측 사각형 크기를 mm 단위로 재서 미터로 변환
  - square_size가 1mm만 틀려도 translation이 5% 이상 뒤틀림

  ros2 run hand_eye_calibration detector --ros-args \
    -p checkerboard_rows:=<rows> \
    -p checkerboard_cols:=<cols> \
    -p square_size:=<meters>

  🟠 HIGH: TF 프레임명 확인 필수

  기본값 ur5e_base_link / ur5e_tool0은 UR ROS2 드라이버 설정마다
  다릅니다. base_link/tool0이거나 ur5e_base/ur5e_flange일 수도.

  # 현장에서 실제 프레임명 확인
  ros2 run tf2_tools view_frames
  # 또는
  ros2 topic echo /tf_static --once
  필요시 -p parent_frame:=<실제이름> -p child_frame:=<실제이름> 으로
  override.

  🟠 HIGH: package.xml에 tf2_ros 의존성 누락

  tf2_ros를 import하는데 package.xml엔 <depend>tf2_ros</depend>가
  없습니다. 현장 컴퓨터에 ros-<distro>-tf2-ros가 설치되어 있지 않은
  clean 환경이면 rosdep install 시 자동 설치가 안 돼서 런타임 에러. ROS2
   기본 환경이라면 대부분 이미 있지만, 확인하고 없으면 sudo apt install
  ros-<distro>-tf2-ros 하세요.

  🟡 MEDIUM: workspace_root 하드코딩

  calibration_collector_node.py:19가 ~/Desktop/Hand_Eye_Calibration으로
  고정. 현장 컴퓨터에서 경로가 다르면 결과 YAML이 엉뚱한 곳(실행
  디렉토리)에 저장됩니다. 기능은 죽지 않지만 파일을 못 찾을 수 있음.
  현장 컴퓨터에서도 동일 경로에 두거나, 찾을 위치를 미리 기억해두세요.

  🟡 MEDIUM: max_reprojection_error=1.0px가 현장 조명에서 빡빡할 수 있음

  감지되는 프레임 비율이 낮으면 sample 수집에 시간이 걸립니다. 만약
  현장에서 "No checkerboard detected" 혹은 "HIGH ERROR: X.XXXpx
  (rejected)"가 자주 뜨면:
  -p max_reprojection_error:=2.0
  정도로 완화하세요. 단, 너무 느슨하면 결과 품질이 떨어지니 2.0 이상은
  비권장.

  🟡 MEDIUM: capture_sample 직후 다음 캡처까지 지연

  캡처 후 last_pose=None으로 초기화 → 다음 synced 포즈까지 대기.
  detector가 2Hz라 최대 ~0.5초 대기 필요. 빠르게 연타하면 "No
  synchronized poses received yet" 경고로 실패합니다. 자세 이동 후 1-2초
   정지 → 캡처 원칙을 지키면 문제 없음.

  🟢 LOW: 코드 수정 후 빌드 1회 필요

  방금 Bug 3 수정했으므로:
  cd ~/Desktop/Hand_Eye_Calibration
  colcon build --symlink-install
  source install/setup.bash
  (symlink install이면 이후 Python 수정은 재빌드 불요)

  ---
  현장 투입 전 로컬 사전 점검 권장

  집에서 할 수 있는 검증:

  # Terminal 1-5로 mock 파이프라인 완주 (README 『테스트 모드』 참고)
  # 1. ZED wrapper (실제 카메라 + 실물 체커보드로 detector만 검증)
  # 2. detector: /camera_to_checkerboard에 포즈가 나오는지
  ros2 topic echo /camera_to_checkerboard --once

  # 3. mock_robot + tf_to_pose로 /base_to_end_effector 나오는지
  ros2 topic echo /base_to_end_effector --once

  # 4. collector에 두 토픽이 동기화되는지 (캡처 성공 메시지 확인)
  # 5. run_calibration으로 Mean residual [mm] 출력 확인

  이걸 로컬에서 성공시키면 현장에선 토픽명/프레임명/체커보드 스펙
  3가지만 교체하면 됩니다.

  ---
  요약 체크리스트 (현장 첫 실행 전)

  1. 기존 .npz 파일 이동 또는 load_on_start:=false
  2. ros2 topic list로 ZED 토픽 실제 이름 확인 → 필요시 remap 인자 준비
  3. ros2 run tf2_tools view_frames로 base/tool 프레임명 확인
  4. 현장 체커보드 내부 코너 수와 square_size 실측 → 파라미터 값 준비
  5. ros2 topic echo /zed/.../camera_info --once로 D 벡터가 0인지
  (rectified인지) 확인
  6. 로컬에서 빌드 성공 (colcon build --symlink-install)
  7. README의 실행 순서대로 터미널 5개 준비해 커맨드를 미리 메모장에
  붙여넣어 두기