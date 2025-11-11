import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from tf_transformations import quaternion_from_matrix

class CheckerboardDetectorNode(Node):
    def __init__(self):
        super().__init__('checkerboard_detector_node')
        
        # 파라미터 선언 (체커보드 내부 코너 수 및 사각형 크기 m단위)
        # 나중에 launch 파일이나 config/checkerboard_params.yaml에서 수정 가능
        self.declare_parameter('checkerboard_rows', 6)
        self.declare_parameter('checkerboard_cols', 9)
        self.declare_parameter('square_size', 0.021)  # 2.5cm
        self.declare_parameter('show_visualization', True)  # 시각화 활성화 여부

        self.rows = self.get_parameter('checkerboard_rows').value
        self.cols = self.get_parameter('checkerboard_cols').value
        self.square_size = self.get_parameter('square_size').value
        self.show_viz = self.get_parameter('show_visualization').value
        
        self.get_logger().info(f"Checkerboard settings: {self.rows}x{self.cols}, size: {self.square_size}m")
        if self.show_viz:
            self.get_logger().info("Visualization enabled (OpenCV window will appear)")
        else:
            self.get_logger().info("Visualization disabled")

        # 3D 물체 포인트 준비
        self.objp = np.zeros((self.rows * self.cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square_size

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = "zed_left_camera_optical_frame" # 기본값

        # PoseStamped 발행자
        self.pose_publisher = self.create_publisher(PoseStamped, '/camera_to_checkerboard', 10)

        # 이미지와 카메라 정보를 동기화하여 구독
        # ZED Wrapper의 토픽 이름에 맞게 수정
        image_sub = message_filters.Subscriber(self, Image, '/zed/zed_node/rgb/image_rect_color')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/zed/zed_node/rgb/camera_info')

        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)
        
        self.get_logger().info("Checkerboard detector node started. Waiting for images...")

    def image_callback(self, img_msg, info_msg):
        # 카메라 정보가 없으면 저장
        if self.camera_matrix is None:
            self.camera_matrix = np.array(info_msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(info_msg.d)
            self.camera_frame_id = info_msg.header.frame_id
            self.get_logger().info(f"Camera intrinsics received. Frame ID: {self.camera_frame_id}")

        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 체커보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows), None)

        if ret:
            # 코너 위치 미세 조정
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_subpix = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # solvePnP를 사용하여 포즈 추정 (T_cam->board)
            ret, rvec, tvec = cv2.solvePnP(self.objp, corners_subpix, self.camera_matrix, self.dist_coeffs)

            if ret:
                # PoseStamped 메시지 생성 및 발행
                pose_msg = PoseStamped()
                pose_msg.header.stamp = img_msg.header.stamp
                pose_msg.header.frame_id = self.camera_frame_id # ZED 카메라의 optical frame
                
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[1][0]
                pose_msg.pose.position.z = tvec[2][0]

                # 회전 벡터(rvec)를 쿼터니언으로 변환
                rmat, _ = cv2.Rodrigues(rvec)
                
                # 4x4 변환 행렬 생성 (tf_transformations용)
                transform_matrix = np.eye(4)
                transform_matrix[:3, :3] = rmat
                q = quaternion_from_matrix(transform_matrix) # T_cam->board
                
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]

                self.pose_publisher.publish(pose_msg)

                # 시각화: 체커보드 코너 그리기
                if self.show_viz:
                    vis_image = cv_image.copy()
                    cv2.drawChessboardCorners(vis_image, (self.cols, self.rows), corners_subpix, ret)
                    
                    # 좌표계 축 그리기 (X=빨강, Y=초록, Z=파랑)
                    axis_length = self.square_size * 3  # 3칸 길이
                    axis_3d = np.float32([[0, 0, 0],
                                         [axis_length, 0, 0],
                                         [0, axis_length, 0],
                                         [0, 0, axis_length]])
                    axis_2d, _ = cv2.projectPoints(axis_3d, rvec, tvec, 
                                                   self.camera_matrix, self.dist_coeffs)
                    
                    origin = tuple(axis_2d[0].ravel().astype(int))
                    x_axis = tuple(axis_2d[1].ravel().astype(int))
                    y_axis = tuple(axis_2d[2].ravel().astype(int))
                    z_axis = tuple(axis_2d[3].ravel().astype(int))
                    
                    cv2.line(vis_image, origin, x_axis, (0, 0, 255), 3)  # X축: 빨강
                    cv2.line(vis_image, origin, y_axis, (0, 255, 0), 3)  # Y축: 초록
                    cv2.line(vis_image, origin, z_axis, (255, 0, 0), 3)  # Z축: 파랑
                    
                    # 거리 정보 표시
                    distance = np.linalg.norm(tvec)
                    cv2.putText(vis_image, f"Distance: {distance:.3f}m", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(vis_image, f"Pose: X={tvec[0][0]:.3f} Y={tvec[1][0]:.3f} Z={tvec[2][0]:.3f}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    cv2.imshow("Checkerboard Detection", vis_image)
                    cv2.waitKey(1)

        else:
            # 체커보드 미검출
            if self.show_viz:
                vis_image = cv_image.copy()
                cv2.putText(vis_image, "No checkerboard detected", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("Checkerboard Detection", vis_image)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CheckerboardDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()