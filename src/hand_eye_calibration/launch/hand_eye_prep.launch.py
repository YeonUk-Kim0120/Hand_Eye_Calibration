import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. ZED Wrapper 실행
    # zed_wrapper 패키지의 zed2i.launch.py를 포함시킵니다.
    # (zed_wrapper에 zed2i.launch.py가 없다면 zed.launch.py 또는 zedm.launch.py 등으로 수정해야 할 수 있습니다.)
    zed_wrapper_launch_dir = os.path.join(get_package_share_directory('zed_wrapper'), 'launch')
    
    # ZED Wrapper 실행 (시뮬레이션이 아니므로 sim:=False)
    # ZED Wrapper의 기본 launch 파일을 사용합니다.
    # 이미 다른 워크스페이스(/home/kist/ros2_ws)에 설치된 zed_wrapper를 사용합니다.
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_launch_dir, 'zed_camera.launch.py')
        ),

        # ZED Wrapper에 필요한 파라미터를 전달할 수 있습니다.
        launch_arguments={'camera_model': 'zed2i'}.items()
    )

    # 2. 체커보드 파라미터 파일 경로
    config_file_dir = os.path.join(get_package_share_directory('hand_eye_calibration'), 'config')
    checkerboard_params_file = os.path.join(config_file_dir, 'checkerboard_params.yaml')

    # 3. 노드 1: Checkerboard Detector 실행
    detector_node = Node(
        package='hand_eye_calibration',
        executable='detector',
        name='checkerboard_detector',
        parameters=[checkerboard_params_file] # 설정 파일 로드
    )

    # 4. 노드 2: Mock Robot Publisher 실행
    # 캘리브레이션 테스트를 위해 포즈를 파라미터로 받도록 설정합니다.
    # Readme에서 설명한 대로, 샘플 수집 시 이 값들을 변경해야 합니다.
    mock_robot_node = Node(
        package='hand_eye_calibration',
        executable='mock_robot',
        name='mock_robot_publisher',
        parameters=[{
            'pose.position.x': 0.5,
            'pose.position.y': 0.1,
            'pose.position.z': 0.4,
            'pose.orientation.x': 0.0,
            'pose.orientation.y': 0.0,
            'pose.orientation.z': 0.0,
            'pose.orientation.w': 1.0,
        }]
    )

    # 5. 노드 3: Calibration Collector 실행
    collector_node = Node(
        package='hand_eye_calibration',
        executable='collector',
        name='calibration_collector',
        output='screen'  # 터미널에 로그(결과)를 바로 출력
    )

    # 실행할 노드 리스트 반환
    return LaunchDescription([
        zed_launch,
        detector_node,
        mock_robot_node,
        collector_node
    ])