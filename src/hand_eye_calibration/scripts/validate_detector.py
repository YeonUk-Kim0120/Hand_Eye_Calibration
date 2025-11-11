#!/usr/bin/env python3
"""
Detector 정확성 검증 도구

실시간으로 detector의 품질을 모니터링하고 검증합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from collections import deque
import time

class DetectorValidator(Node):
    def __init__(self):
        super().__init__('detector_validator')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/camera_to_checkerboard',
            self.pose_callback,
            10
        )
        
        # 통계 수집
        self.positions = deque(maxlen=100)  # 최근 100개 위치
        self.distances = deque(maxlen=100)
        self.quaternions = deque(maxlen=100)
        self.timestamps = deque(maxlen=100)
        
        # 타이머: 5초마다 통계 출력
        self.timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info("Detector Validator started. Monitoring /camera_to_checkerboard...")
    
    def pose_callback(self, msg):
        # 위치 추출
        position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        # 쿼터니언 추출
        quaternion = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        # 거리 계산
        distance = np.linalg.norm(position)
        
        # 데이터 저장
        self.positions.append(position)
        self.distances.append(distance)
        self.quaternions.append(quaternion)
        self.timestamps.append(time.time())
        
        # 실시간 검증
        self._validate_pose(position, quaternion, distance)
    
    def _validate_pose(self, position, quaternion, distance):
        """포즈 유효성 실시간 검증"""
        
        # 1. 쿼터니언 정규화 체크
        quat_norm = np.linalg.norm(quaternion)
        if abs(quat_norm - 1.0) > 0.01:
            self.get_logger().warn(f"⚠️  Quaternion not normalized: {quat_norm:.6f}")
        
        # 2. Z축 양수 체크 (카메라 앞에 있어야 함)
        if position[2] < 0:
            self.get_logger().error("❌ Checkerboard behind camera! Z < 0")
        elif position[2] < 0.1:
            self.get_logger().warn(f"⚠️  Checkerboard very close: Z={position[2]:.3f}m")
        
        # 3. 거리 범위 체크
        if distance < 0.2:
            self.get_logger().warn(f"⚠️  Too close: {distance:.3f}m")
        elif distance > 5.0:
            self.get_logger().warn(f"⚠️  Too far: {distance:.3f}m")
        
        # 4. 위치 범위 체크
        if abs(position[0]) > 3.0 or abs(position[1]) > 3.0:
            self.get_logger().warn(f"⚠️  Unusual position: X={position[0]:.3f}, Y={position[1]:.3f}")
    
    def print_statistics(self):
        """통계 출력"""
        if len(self.positions) < 2:
            return
        
        positions_array = np.array(self.positions)
        distances_array = np.array(self.distances)
        
        # 위치 통계
        mean_pos = np.mean(positions_array, axis=0)
        std_pos = np.std(positions_array, axis=0)
        
        # 거리 통계
        mean_dist = np.mean(distances_array)
        std_dist = np.std(distances_array)
        
        # 프레임 레이트 계산
        if len(self.timestamps) >= 2:
            time_diff = self.timestamps[-1] - self.timestamps[0]
            fps = len(self.timestamps) / time_diff if time_diff > 0 else 0
        else:
            fps = 0
        
        # 출력
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📊 Detection Statistics (last {len(self.positions)} samples)")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"Position Mean: X={mean_pos[0]:.4f}, Y={mean_pos[1]:.4f}, Z={mean_pos[2]:.4f}")
        self.get_logger().info(f"Position StdDev: X={std_pos[0]:.4f}, Y={std_pos[1]:.4f}, Z={std_pos[2]:.4f}")
        self.get_logger().info(f"Distance: {mean_dist:.4f}m ± {std_dist:.4f}m")
        self.get_logger().info(f"Detection Rate: {fps:.2f} Hz")
        self.get_logger().info("-" * 60)
        
        # 안정성 평가
        if std_pos[0] < 0.001 and std_pos[1] < 0.001 and std_pos[2] < 0.001:
            self.get_logger().info("✅ Excellent stability (σ < 1mm)")
        elif std_pos[0] < 0.005 and std_pos[1] < 0.005 and std_pos[2] < 0.005:
            self.get_logger().info("✅ Good stability (σ < 5mm)")
        elif std_pos[0] < 0.01 and std_pos[1] < 0.01 and std_pos[2] < 0.01:
            self.get_logger().info("⚠️  Fair stability (σ < 10mm)")
        else:
            self.get_logger().warn("❌ Poor stability (σ > 10mm) - Check setup!")
        
        self.get_logger().info("=" * 60 + "\n")

def main(args=None):
    rclpy.init(args=args)
    validator = DetectorValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
