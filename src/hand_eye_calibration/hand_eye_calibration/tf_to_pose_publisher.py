#!/usr/bin/env python3
"""
TF를 PoseStamped로 변환하여 발행하는 노드
UR5e의 base_link → tool0 변환을 /base_to_end_effector 토픽으로 발행
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TFToPosePublisher(Node):
    def __init__(self):
        super().__init__('tf_to_pose_publisher')
        
        # 파라미터 선언
        self.declare_parameter('parent_frame', 'ur5e_base_link')
        self.declare_parameter('child_frame', 'ur5e_tool0')
        self.declare_parameter('publish_topic', '/base_to_end_effector')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        publish_topic = self.get_parameter('publish_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # TF2 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher 생성
        self.pose_pub = self.create_publisher(PoseStamped, publish_topic, 10)
        
        # 타이머 생성
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'TF to Pose Publisher started')
        self.get_logger().info(f'  Parent frame: {self.parent_frame}')
        self.get_logger().info(f'  Child frame: {self.child_frame}')
        self.get_logger().info(f'  Publishing to: {publish_topic}')
        self.get_logger().info(f'  Rate: {publish_rate} Hz')
    
    def timer_callback(self):
        try:
            # TF 조회
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time()
            )
            
            # PoseStamped 메시지 생성
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.parent_frame
            
            # Transform → Pose 변환
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            
            pose_msg.pose.orientation.x = transform.transform.rotation.x
            pose_msg.pose.orientation.y = transform.transform.rotation.y
            pose_msg.pose.orientation.z = transform.transform.rotation.z
            pose_msg.pose.orientation.w = transform.transform.rotation.w
            
            # 발행
            self.pose_pub.publish(pose_msg)
            
        except TransformException as ex:
            # TF를 찾을 수 없으면 경고 (너무 자주 출력하지 않도록)
            self.get_logger().warn(
                f'Could not transform {self.parent_frame} to {self.child_frame}: {ex}',
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = TFToPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
