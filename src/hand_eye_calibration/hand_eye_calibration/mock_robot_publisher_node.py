import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor

class MockRobotPublisherNode(Node):
    def __init__(self):
        super().__init__('mock_robot_publisher_node')
        
        # 캘리브레이션 샘플 테스트를 위해 포즈 값을 파라미터로 선언합니다.
        # 터미널이나 launch 파일에서 이 값들을 변경하며 샘플을 수집합니다.
        pd_pos = ParameterDescriptor(description="Mock robot end-effector position")
        pd_quat = ParameterDescriptor(description="Mock robot end-effector orientation (quaternion)")

        self.declare_parameter('pose.position.x', 0.5, pd_pos)
        self.declare_parameter('pose.position.y', 0.0, pd_pos)
        self.declare_parameter('pose.position.z', 0.3, pd_pos)
        
        self.declare_parameter('pose.orientation.x', 0.0, pd_quat)
        self.declare_parameter('pose.orientation.y', 0.0, pd_quat)
        self.declare_parameter('pose.orientation.z', 0.0, pd_quat)
        self.declare_parameter('pose.orientation.w', 1.0, pd_quat) # 기본값: 회전 없음

        self.publisher_ = self.create_publisher(PoseStamped, '/base_to_end_effector', 10)
        
        # 10Hz (0.1초) 주기로 포즈 발행
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = "base_link" # 로봇의 베이스 프레임
        
        self.get_logger().info("Mock robot publisher started.")
        self.get_logger().info("Publishing /base_to_end_effector topic.")
        self.get_logger().warn("Change parameters 'pose.position.[x,y,z]' and 'pose.orientation.[x,y,z,w]' to get different samples.")

    def timer_callback(self):
        # 현재 시간을 메시지 헤더에 저장
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 파라미터에서 현재 포즈 값 읽어오기
        self.pose_msg.pose.position.x = self.get_parameter('pose.position.x').value
        self.pose_msg.pose.position.y = self.get_parameter('pose.position.y').value
        self.pose_msg.pose.position.z = self.get_parameter('pose.position.z').value
        
        self.pose_msg.pose.orientation.x = self.get_parameter('pose.orientation.x').value
        self.pose_msg.pose.orientation.y = self.get_parameter('pose.orientation.y').value
        self.pose_msg.pose.orientation.z = self.get_parameter('pose.orientation.z').value
        self.pose_msg.pose.orientation.w = self.get_parameter('pose.orientation.w').value

        self.publisher_.publish(self.pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockRobotPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()