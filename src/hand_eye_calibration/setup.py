from setuptools import setup
import os
from glob import glob

package_name = 'hand_eye_calibration'

setup(
    name=package_name,
    version='0.0.1',
    # 'hand_eye_calibration' 폴더 안의 모든 파이썬 모듈을 패키지에 포함
    packages=[package_name],
    
    # 패키지 설치 시 함께 설치될 데이터 파일들 (launch, config 등)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 디렉토리 안의 .launch.py 파일들을 포함
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        # config 디렉토리 안의 .yaml 파일들을 포함
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
        # scripts 디렉토리 안의 Python 스크립트들을 포함
        (os.path.join('share', package_name, 'scripts'), 
            glob(os.path.join('scripts', '*.py'))),
    ],
    
    # 파이썬 패키지 의존성 (ROS 패키지 외)
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    
    # package.xml과 동일하게 작성
    maintainer='Yeonuk Kim',
    maintainer_email='kimyeonuk0120@gmail.com',
    description='Eye-to-Hand calibration package with ZED 2i and mock robot',
    license='Apache-2.0',
    
    # 테스트 도구
    tests_require=['pytest'],
    
    # 'console_scripts'는 ROS 2 노드를 실행 파일로 등록하는 핵심 부분
    entry_points={
        'console_scripts': [
            # '실행파일_이름 = 패키지명.모듈명:main함수'
            'detector = hand_eye_calibration.checkerboard_detector_node:main',
            'mock_robot = hand_eye_calibration.mock_robot_publisher_node:main',
            'collector = hand_eye_calibration.calibration_collector_node:main',
            'tf_to_pose = hand_eye_calibration.tf_to_pose_publisher:main',
        ],
    },
)