#!/usr/bin/env python3
"""
자동 캘리브레이션 샘플 수집 스크립트 (키보드 제어)

YAML 파일에서 로봇 포즈 목록을 읽어, 각 포즈마다:
1. 사용자에게 포즈 정보를 표시
2. 사용자가 Enter 키를 누르면 포즈 설정
3. 체커보드 위치 확인 후 다시 Enter로 샘플 캡처
4. 'q' 입력 시 중단, 's' 입력 시 건너뛰기

사용법:
    python3 auto_capture.py [poses.yaml 경로]
    
또는:
    ros2 run hand_eye_calibration auto_capture
    
키보드 명령:
    Enter - 다음 단계 진행 (포즈 설정 → 샘플 캡처)
    s - 현재 포즈 건너뛰기
    q - 종료
    c - 지금까지 수집한 샘플로 캘리브레이션 실행
"""

import sys
import time
import subprocess
import yaml
from pathlib import Path


def run_command(cmd):
    """subprocess로 명령 실행"""
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"⚠️  Command failed: {' '.join(cmd)}")
        print(f"   Error: {result.stderr}")
    return result.returncode == 0


def set_robot_pose(position, orientation):
    """ROS2 파라미터로 mock 로봇의 포즈 설정"""
    x, y, z = position
    qx, qy, qz, qw = orientation
    
    params = {
        'pose.position.x': x,
        'pose.position.y': y,
        'pose.position.z': z,
        'pose.orientation.x': qx,
        'pose.orientation.y': qy,
        'pose.orientation.z': qz,
        'pose.orientation.w': qw,
    }
    
    for param_name, value in params.items():
        cmd = ['ros2', 'param', 'set', '/mock_robot_publisher', param_name, str(value)]
        if not run_command(cmd):
            return False
    
    return True


def capture_sample():
    """캘리브레이션 샘플 캡처 서비스 호출"""
    cmd = ['ros2', 'service', 'call', '/capture_sample', 'std_srvs/srv/Empty']
    return run_command(cmd)


def run_calibration():
    """캘리브레이션 실행 서비스 호출"""
    cmd = ['ros2', 'service', 'call', '/run_calibration', 'std_srvs/srv/Empty']
    return run_command(cmd)


def get_user_input(prompt_text):
    """사용자 입력 받기 (Enter, s, q, c)"""
    while True:
        user_input = input(prompt_text).strip().lower()
        if user_input == '' or user_input in ['s', 'q', 'c']:
            return user_input
        print("⚠️  잘못된 입력입니다. Enter, 's', 'q', 'c' 중 하나를 입력하세요.")


def main():
    # YAML 파일 경로 확인
    if len(sys.argv) > 1:
        yaml_path = Path(sys.argv[1])
    else:
        # 기본 경로: config/poses.yaml
        yaml_path = Path(__file__).parent.parent / 'config' / 'poses.yaml'
    
    if not yaml_path.exists():
        print(f"❌ YAML 파일을 찾을 수 없습니다: {yaml_path}")
        print(f"\n사용법: {sys.argv[0]} [poses.yaml 경로]")
        return 1
    
    # YAML 파일 읽기
    print(f"📂 포즈 파일 로딩: {yaml_path}")
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    poses = config.get('poses', [])
    
    if not poses:
        print("❌ YAML 파일에 포즈가 없습니다.")
        return 1
    
    print(f"✅ {len(poses)}개의 포즈를 찾았습니다.\n")
    
    # 사용자 확인
    print("=" * 70)
    print("🤖 자동 캘리브레이션 샘플 수집 (키보드 제어)")
    print("=" * 70)
    print("\n⚠️  주의사항:")
    print("  1. hand_eye_prep.launch.py가 실행 중이어야 합니다.")
    print("  2. ZED 카메라가 체커보드를 볼 수 있어야 합니다.")
    print("  3. 각 포즈에서 Enter로 진행, 's'로 건너뛰기, 'q'로 종료합니다.\n")
    
    print("📌 키보드 명령:")
    print("   Enter - 다음 단계 진행 (포즈 설정 → 샘플 캡처)")
    print("   s     - 현재 포즈 건너뛰기")
    print("   q     - 종료")
    print("   c     - 지금 캘리브레이션 실행\n")
    
    input("시작하려면 Enter를 누르세요... (Ctrl+C로 취소)")
    print()
    
    # 각 포즈마다 샘플 수집
    successful_captures = 0
    
    for i, pose_data in enumerate(poses, 1):
        position = pose_data['position']
        orientation = pose_data['orientation']
        
        print("\n" + "=" * 70)
        print(f"📍 포즈 {i}/{len(poses)}")
        print("=" * 70)
        print(f"위치 (m): x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
        print(f"방향 (quaternion): x={orientation[0]:.3f}, y={orientation[1]:.3f}, "
              f"z={orientation[2]:.3f}, w={orientation[3]:.3f}")
        
        # 포즈 설정 확인
        user_input = get_user_input("\n➡️  이 포즈를 설정하시겠습니까? [Enter=예 / s=건너뛰기 / q=종료 / c=캘리브레이션]: ")
        
        if user_input == 'q':
            print("\n❌ 사용자에 의해 종료되었습니다.")
            break
        elif user_input == 'c':
            print("\n🔧 캘리브레이션을 실행합니다...")
            if successful_captures >= 5:
                if run_calibration():
                    print("✅ 캘리브레이션 완료!")
                    print("   결과는 calibration_collector 노드의 터미널에서 확인하세요.")
                else:
                    print("❌ 캘리브레이션 실행 실패")
            else:
                print(f"⚠️  샘플이 부족합니다. 최소 5개 필요 (현재 {successful_captures}개)")
            break
        elif user_input == 's':
            print("⏭️  포즈를 건너뜁니다.\n")
            continue
        
        # 로봇 포즈 설정
        print("\n⚙️  로봇 포즈 설정 중...")
        if not set_robot_pose(position, orientation):
            print("❌ 포즈 설정 실패. 다음 포즈로 이동합니다.\n")
            continue
        
        print("✅ 포즈 설정 완료!")
        print("\n💡 체커보드 검출 윈도우에서 포즈를 확인하세요.")
        print("   - 녹색 축이 표시되면 체커보드가 검출된 것입니다.")
        print("   - 체커보드가 잘 보이는지 확인 후 샘플을 캡처하세요.")
        
        # 샘플 캡처 확인
        user_input = get_user_input("\n📸 샘플을 캡처하시겠습니까? [Enter=예 / s=건너뛰기 / q=종료 / c=캘리브레이션]: ")
        
        if user_input == 'q':
            print("\n❌ 사용자에 의해 종료되었습니다.")
            break
        elif user_input == 'c':
            print("\n🔧 캘리브레이션을 실행합니다...")
            if successful_captures >= 5:
                if run_calibration():
                    print("✅ 캘리브레이션 완료!")
                    print("   결과는 calibration_collector 노드의 터미널에서 확인하세요.")
                else:
                    print("❌ 캘리브레이션 실행 실패")
            else:
                print(f"⚠️  샘플이 부족합니다. 최소 5개 필요 (현재 {successful_captures}개)")
            break
        elif user_input == 's':
            print("⏭️  샘플 캡처를 건너뜁니다.\n")
            continue
        
        # 샘플 캡처
        print("\n📸 샘플 캡처 중...")
        if capture_sample():
            successful_captures += 1
            print(f"✅ 샘플 {successful_captures} 저장 완료!")
        else:
            print("❌ 샘플 캡처 실패")
    
    # 결과 요약
    print("\n" + "=" * 70)
    print(f"📊 샘플 수집 완료: {successful_captures}/{len(poses)}")
    print("=" * 70)
    
    if successful_captures >= 5:
        print(f"\n✅ 충분한 샘플이 수집되었습니다! (총 {successful_captures}개)")
        user_input = get_user_input("\n🔧 캘리브레이션을 실행하시겠습니까? [Enter=예 / 기타=아니오]: ")
        
        if user_input == '':
            print("\n🔧 캘리브레이션 실행 중...")
            if run_calibration():
                print("✅ 캘리브레이션 완료!")
                print("   결과는 calibration_collector 노드의 터미널에서 확인하세요.")
                print("   calibration_result_YYYYMMDD_HHMMSS.yaml 파일이 생성되었습니다.")
            else:
                print("❌ 캘리브레이션 실행 실패")
    else:
        print(f"\n⚠️  샘플이 부족합니다. 최소 5개 필요 (현재 {successful_captures}개)")
        print("   스크립트를 다시 실행하여 추가 샘플을 수집할 수 있습니다.")
        print("   (기존 샘플은 자동 저장되어 있습니다)")
    
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n❌ Ctrl+C로 중단되었습니다.")
        sys.exit(1)
