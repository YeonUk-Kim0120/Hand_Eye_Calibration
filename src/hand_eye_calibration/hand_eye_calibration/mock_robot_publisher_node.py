import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import TransformBroadcaster

class MockRobotPublisherNode(Node):
    def __init__(self):
        super().__init__('mock_robot_publisher_node')
        
        # 캘리브레이션 샘플 테스트를 위해 포즈 값을 파라미터로 선언합니다.
        # 터미널이나 launch 파일에서 이 값들을 변경하며 샘플을 수집합니다.
        pd_pos = ParameterDescriptor(description="Mock robot end-effector position")
        pd_quat = ParameterDescriptor(description="Mock robot end-effector orientation (quaternion)")
        pd_frame = ParameterDescriptor(description="TF frame names")

        self.declare_parameter('pose.position.x', 0.5, pd_pos)
        self.declare_parameter('pose.position.y', 0.0, pd_pos)
        self.declare_parameter('pose.position.z', 0.3, pd_pos)
        
        self.declare_parameter('pose.orientation.x', 0.0, pd_quat)
        self.declare_parameter('pose.orientation.y', 0.0, pd_quat)
        self.declare_parameter('pose.orientation.z', 0.0, pd_quat)
        self.declare_parameter('pose.orientation.w', 1.0, pd_quat) # 기본값: 회전 없음
        
        self.declare_parameter('parent_frame', 'ur5e_base_link', pd_frame)
        self.declare_parameter('child_frame', 'ur5e_tool0', pd_frame)

        # TF broadcaster 생성 (실제 로봇처럼 TF를 발행)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 10Hz (0.1초) 주기로 TF 발행
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        
        self.get_logger().info("Mock robot TF broadcaster started.")
        self.get_logger().info(f"Broadcasting TF: {self.parent_frame} → {self.child_frame}")
        self.get_logger().warn("Change parameters 'pose.position.[x,y,z]' and 'pose.orientation.[x,y,z,w]' to get different samples.")
        self.get_logger().info("Use with tf_to_pose node to publish /base_to_end_effector topic.")

    def timer_callback(self):
        # TransformStamped 메시지 생성
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        
        # 파라미터에서 현재 포즈 값 읽어오기
        t.transform.translation.x = self.get_parameter('pose.position.x').value
        t.transform.translation.y = self.get_parameter('pose.position.y').value
        t.transform.translation.z = self.get_parameter('pose.position.z').value
        
        t.transform.rotation.x = self.get_parameter('pose.orientation.x').value
        t.transform.rotation.y = self.get_parameter('pose.orientation.y').value
        t.transform.rotation.z = self.get_parameter('pose.orientation.z').value
        t.transform.rotation.w = self.get_parameter('pose.orientation.w').value

        # TF 발행
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MockRobotPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()