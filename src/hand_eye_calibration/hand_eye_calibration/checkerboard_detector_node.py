#!/usr/bin/env python3

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
        
        # 파라미터 선언
        self.declare_parameter('checkerboard_rows', 6)
        self.declare_parameter('checkerboard_cols', 9)
        self.declare_parameter('square_size', 0.018)  # 1.8cm
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('max_reprojection_error', 1.0)  # 픽셀 단위
        self.declare_parameter('pnp_method', 'IPPE')  # ITERATIVE, IPPE, SQPNP

        self.rows = self.get_parameter('checkerboard_rows').value
        self.cols = self.get_parameter('checkerboard_cols').value
        self.square_size = self.get_parameter('square_size').value
        self.show_viz = self.get_parameter('show_visualization').value
        self.max_reproj_error = self.get_parameter('max_reprojection_error').value
        
        # PnP 방법 선택
        pnp_method_str = self.get_parameter('pnp_method').value
        self.pnp_flags = self._get_pnp_flags(pnp_method_str)
        
        self.get_logger().info(f"Checkerboard settings: {self.rows}x{self.cols}, size: {self.square_size}m")
        self.get_logger().info(f"PnP method: {pnp_method_str}")
        self.get_logger().info(f"Max reprojection error: {self.max_reproj_error} pixels")
        if self.show_viz:
            self.get_logger().info("Visualization enabled")
        
        # 통계 추적
        self.detection_count = 0
        self.rejection_count = 0
        self.total_error = 0.0

        # 3D 물체 포인트 준비
        self.objp = np.zeros((self.rows * self.cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square_size

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = "zed_left_camera_optical_frame"

        # PoseStamped 발행자
        self.pose_publisher = self.create_publisher(PoseStamped, '/camera_to_checkerboard', 10)

        # 이미지와 카메라 정보를 동기화하여 구독
        image_sub = message_filters.Subscriber(self, Image, '/zed/zed_node/rgb/image_rect_color')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/zed/zed_node/rgb/camera_info')

        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)
        
        self.get_logger().info("Checkerboard detector node started. Waiting for images...")
    
    def _get_pnp_flags(self, method_str):
        """PnP 솔버 방법 선택"""
        methods = {
            'ITERATIVE': cv2.SOLVEPNP_ITERATIVE,
            'IPPE': cv2.SOLVEPNP_IPPE,
            'SQPNP': cv2.SOLVEPNP_SQPNP,
            'EPNP': cv2.SOLVEPNP_EPNP,
        }
        return methods.get(method_str.upper(), cv2.SOLVEPNP_ITERATIVE)

    def image_callback(self, img_msg, info_msg):
        # 카메라 정보 저장 (최초 1회)
        if self.camera_matrix is None:
            self.camera_matrix = np.array(info_msg.k).reshape(3, 3)
            raw_dist = np.array(info_msg.d)
            
            # ZED rectified 이미지는 왜곡이 제거되어 있음
            if len(raw_dist) == 0 or np.allclose(raw_dist, 0):
                self.dist_coeffs = np.zeros(5)
                self.get_logger().info("Using rectified images (no distortion)")
            else:
                self.dist_coeffs = raw_dist
                self.get_logger().info(f"Distortion coefficients: {self.dist_coeffs}")
            
            self.camera_frame_id = info_msg.header.frame_id
            self.get_logger().info(f"Camera intrinsics received:")
            self.get_logger().info(f"  Frame ID: {self.camera_frame_id}")
            self.get_logger().info(f"  fx={self.camera_matrix[0,0]:.2f}, fy={self.camera_matrix[1,1]:.2f}")
            self.get_logger().info(f"  cx={self.camera_matrix[0,2]:.2f}, cy={self.camera_matrix[1,2]:.2f}")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 체커보드 코너 찾기 (개선된 플래그 사용)
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + 
                 cv2.CALIB_CB_NORMALIZE_IMAGE + 
                 cv2.CALIB_CB_FAST_CHECK)
        ret, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows), flags)

        if ret:
            # 서브픽셀 정밀도로 코너 위치 미세 조정
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_subpix = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # solvePnP로 포즈 추정
            ret_pnp, rvec, tvec = cv2.solvePnP(
                self.objp, corners_subpix, 
                self.camera_matrix, self.dist_coeffs,
                flags=self.pnp_flags
            )

            if ret_pnp:
                # 재투영 오차 계산
                imgpoints_reproj, _ = cv2.projectPoints(
                    self.objp, rvec, tvec, 
                    self.camera_matrix, self.dist_coeffs
                )
                reprojection_error = cv2.norm(corners_subpix, imgpoints_reproj, cv2.NORM_L2) / len(imgpoints_reproj)
                
                # 품질 필터링
                if reprojection_error > self.max_reproj_error:
                    self.rejection_count += 1
                    if self.show_viz:
                        vis_image = cv_image.copy()
                        cv2.putText(vis_image, f"HIGH ERROR: {reprojection_error:.3f}px (rejected)", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        cv2.drawChessboardCorners(vis_image, (self.cols, self.rows), corners_subpix, ret)
                        cv2.imshow("Checkerboard Detection", vis_image)
                        cv2.waitKey(1)
                    return
                
                # 통계 업데이트
                self.detection_count += 1
                self.total_error += reprojection_error
                avg_error = self.total_error / self.detection_count
                
                # PoseStamped 메시지 생성 및 발행
                pose_msg = PoseStamped()
                pose_msg.header.stamp = img_msg.header.stamp
                pose_msg.header.frame_id = self.camera_frame_id
                
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[1][0]
                pose_msg.pose.position.z = tvec[2][0]

                # Rodrigues → 회전 행렬 → 쿼터니언
                rmat, _ = cv2.Rodrigues(rvec)
                transform_matrix = np.eye(4)
                transform_matrix[:3, :3] = rmat
                q = quaternion_from_matrix(transform_matrix)
                
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]

                self.pose_publisher.publish(pose_msg)

                # 시각화
                if self.show_viz:
                    vis_image = cv_image.copy()
                    cv2.drawChessboardCorners(vis_image, (self.cols, self.rows), corners_subpix, ret)
                    
                    # 좌표축 그리기
                    axis_length = self.square_size * 3
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
                    
                    cv2.line(vis_image, origin, x_axis, (0, 0, 255), 3)  # X: 빨강
                    cv2.line(vis_image, origin, y_axis, (0, 255, 0), 3)  # Y: 초록
                    cv2.line(vis_image, origin, z_axis, (255, 0, 0), 3)  # Z: 파랑
                    
                    # 정보 표시
                    distance = np.linalg.norm(tvec)
                    cv2.putText(vis_image, f"Distance: {distance:.3f}m", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(vis_image, f"Pose: X={tvec[0][0]:.3f} Y={tvec[1][0]:.3f} Z={tvec[2][0]:.3f}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(vis_image, f"Reproj Error: {reprojection_error:.3f}px (avg: {avg_error:.3f})", 
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(vis_image, f"Detections: {self.detection_count} (rejected: {self.rejection_count})", 
                               (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 통계 출력
        if node.detection_count > 0:
            avg_error = node.total_error / node.detection_count
            node.get_logger().info(f"=== Detection Statistics ===")
            node.get_logger().info(f"Total detections: {node.detection_count}")
            node.get_logger().info(f"Rejected (high error): {node.rejection_count}")
            node.get_logger().info(f"Average reprojection error: {avg_error:.4f} pixels")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
