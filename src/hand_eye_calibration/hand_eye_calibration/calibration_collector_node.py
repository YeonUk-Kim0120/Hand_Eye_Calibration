import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import message_filters
from std_srvs.srv import Empty
import cv2
import numpy as np
import tf_transformations
import json
import yaml
from datetime import datetime
import os

class CalibrationCollectorNode(Node):
    def __init__(self):
        super().__init__('calibration_collector_node')

        # 샘플 저장 파일 경로 파라미터
        self.declare_parameter('samples_file', 'calibration_samples.npz')
        self.declare_parameter('auto_save', True)  # 각 캡처마다 자동 저장
        self.declare_parameter('load_on_start', True)  # 시작 시 기존 샘플 불러오기
        
        self.samples_file = self.get_parameter('samples_file').value
        self.auto_save = self.get_parameter('auto_save').value
        self.load_on_start = self.get_parameter('load_on_start').value

        # 데이터 저장을 위한 리스트
        self.T_base_ee_list = []  # 로봇 (A)
        self.T_cam_board_list = [] # 카메라 (B)

        # 마지막으로 동기화된 메시지를 저장할 변수
        self.last_base_ee_pose = None
        self.last_cam_board_pose = None
        self.lock = False # 샘플 수집 중 변경을 막기 위한 락

        # 토픽 구독
        base_ee_sub = message_filters.Subscriber(self, PoseStamped, '/base_to_end_effector')
        cam_board_sub = message_filters.Subscriber(self, PoseStamped, '/camera_to_checkerboard')

        # 두 토픽 동기화
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [base_ee_sub, cam_board_sub], 
            queue_size=10, 
            slop=0.1 # 100ms 이내의 타임스탬프 차이 허용
        )
        self.ts.registerCallback(self.sync_callback)

        # 샘플 수집을 위한 서비스
        self.capture_srv = self.create_service(Empty, '/capture_sample', self.capture_sample_callback)
        
        # 캘리브레이션 실행을 위한 서비스
        self.calibrate_srv = self.create_service(Empty, '/run_calibration', self.run_calibration_callback)

        # 기존 샘플 불러오기
        if self.load_on_start:
            self._load_samples()

        self.get_logger().info("Calibration collector node started.")
        self.get_logger().info(f"Samples file: {self.samples_file}")
        self.get_logger().info(f"Auto-save: {self.auto_save}, Load on start: {self.load_on_start}")
        self.get_logger().info(f"Currently loaded samples: {len(self.T_base_ee_list)}")
        self.get_logger().info("Waiting for synchronized topics...")
        self.get_logger().info("Call '/capture_sample' service to collect data.")
        self.get_logger().info("Call '/run_calibration' service to perform calibration.")

    def sync_callback(self, base_ee_msg, cam_board_msg):
        # 락이 걸려있지 않을 때만 최신 포즈 저장
        if not self.lock:
            self.last_base_ee_pose = base_ee_msg
            self.last_cam_board_pose = cam_board_msg
            # self.get_logger().info("Received synchronized poses.", throttle_duration_sec=1.0)

    def _pose_to_matrix(self, pose):
        """ geometry_msgs/Pose를 4x4 numpy 변환 행렬(T)로 변환 """
        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        t = [pose.position.x, pose.position.y, pose.position.z]
        
        rotation_matrix = tf_transformations.quaternion_matrix(q)
        translation_matrix = tf_transformations.translation_matrix(t)
        
        # T = T_trans * T_rot
        return np.dot(translation_matrix, rotation_matrix)

    def capture_sample_callback(self, request, response):
        self.lock = True # 데이터 복사 중 덮어쓰기 방지
        
        if self.last_base_ee_pose is None or self.last_cam_board_pose is None:
            self.get_logger().warn("No synchronized poses received yet. Cannot capture sample.")
            return response

        # 현재 저장된 마지막 포즈를 4x4 행렬로 변환하여 리스트에 추가
        T_base_ee = self._pose_to_matrix(self.last_base_ee_pose.pose)
        T_cam_board = self._pose_to_matrix(self.last_cam_board_pose.pose)
        
        self.T_base_ee_list.append(T_base_ee)
        self.T_cam_board_list.append(T_cam_board)

        self.get_logger().info(f"Sample {len(self.T_base_ee_list)} captured.")
        
        # 자동 저장 (NPZ 파일)
        if self.auto_save:
            self._save_samples()
        
        # 다음 샘플을 위해 마지막 포즈 초기화 (중복 수집 방지)
        self.last_base_ee_pose = None
        self.last_cam_board_pose = None
        self.lock = False
        
        return response

    def run_calibration_callback(self, request, response):
        min_samples = 5 # 캘리브레이션을 위한 최소 샘플 수
        if len(self.T_base_ee_list) < min_samples:
            self.get_logger().error(f"Need at least {min_samples} samples to run calibration. Currently have {len(self.T_base_ee_list)}.")
            return response

        self.get_logger().info(f"Running Hand-Eye calibration with {len(self.T_base_ee_list)} samples...")

        # cv2.calibrateRobotWorldHandEye에 맞게 R과 t 분리
        R_base_ee_list = []
        t_base_ee_list = []
        R_cam_board_list = []
        t_cam_board_list = []

        for T_base_ee, T_cam_board in zip(self.T_base_ee_list, self.T_cam_board_list):
            R_base_ee_list.append(T_base_ee[:3, :3])
            t_base_ee_list.append(T_base_ee[:3, 3].reshape(3, 1)) # (3,1) 형태로
            
            R_cam_board_list.append(T_cam_board[:3, :3])
            t_cam_board_list.append(T_cam_board[:3, 3].reshape(3, 1)) # (3,1) 형태로

        # Hand-on-Base (Eye-to-Hand) 캘리브레이션
        # calibrateRobotWorldHandEye 사용
        # 
        # OpenCV 함수:
        #   입력: R_world2cam (world→camera), R_base2gripper (base→gripper)
        #   출력: R_base2world (base→world), R_gripper2cam (gripper→camera)
        #
        # 우리 매핑:
        #   world = checkerboard
        #   gripper = end_effector
        #
        # 따라서:
        #   입력: T_board_cam (board→cam) = inv(T_cam_board), T_base_ee
        #   출력: T_base_board, T_ee_cam
        #
        # 우리가 원하는 것:
        #   T_base_cam = T_base_board * inv(T_board_cam) = T_base_board * T_cam_board
        #   T_ee_board = inv(T_ee_cam) * inv(T_board_cam) = inv(T_ee_cam) * T_cam_board
        try:
            # 입력 변환: T_cam_board를 T_board_cam (board→camera)로 역변환
            R_board_cam_list = []
            t_board_cam_list = []
            
            for T_cam_board in self.T_cam_board_list:
                T_board_cam = np.linalg.inv(T_cam_board)
                R_board_cam_list.append(T_board_cam[:3, :3])
                t_board_cam_list.append(T_board_cam[:3, 3].reshape(3, 1))
            
            # calibrateRobotWorldHandEye 호출
            R_base_board, t_base_board, R_ee_cam, t_ee_cam = cv2.calibrateRobotWorldHandEye(
                R_world2cam=R_board_cam_list,      # T_board_cam (checkerboard → camera)
                t_world2cam=t_board_cam_list,
                R_base2gripper=R_base_ee_list,     # T_base_ee (base → end-effector)
                t_base2gripper=t_base_ee_list,
                method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH  # Shah 방법 사용
            )
            
            # 출력:
            # R_base_board, t_base_board: T_base_board (base → checkerboard)
            # R_ee_cam, t_ee_cam: T_ee_cam (end-effector → camera)
            
            # 우리가 원하는 것 계산:
            # 1. T_base_cam = T_base_board * T_board_cam = T_base_board * inv(T_cam_board)
            T_base_board = np.eye(4)
            T_base_board[:3, :3] = R_base_board
            T_base_board[:3, 3] = t_base_board.flatten()
            
            # 첫 번째 샘플의 T_cam_board 사용 (모든 샘플에서 일관성 확인 가능)
            T_board_cam = np.linalg.inv(self.T_cam_board_list[0])
            T_base_cam = np.dot(T_base_board, T_board_cam)
            
            R_base_cam = T_base_cam[:3, :3]
            t_base_cam = T_base_cam[:3, 3].reshape(3, 1)
            
            # 2. T_ee_board = inv(T_ee_cam) * T_cam_board
            T_ee_cam = np.eye(4)
            T_ee_cam[:3, :3] = R_ee_cam
            T_ee_cam[:3, 3] = t_ee_cam.flatten()
            
            T_cam_ee = np.linalg.inv(T_ee_cam)
            T_ee_board = np.dot(T_cam_ee, self.T_cam_board_list[0])
            
            R_ee_board = T_ee_board[:3, :3]
            t_ee_board = T_ee_board[:3, 3].reshape(3, 1)

            # 결과 출력
            self.get_logger().info("=== Calibration Results ===")
            self.get_logger().info("")
            self.get_logger().info("--- T_base_cam (base → camera) ---")
            self.get_logger().info(f"Transformation Matrix:\n{T_base_cam}")

            # (참고) 쿼터니언과 XYZ로도 분리하여 출력
            q_base_cam = tf_transformations.quaternion_from_matrix(T_base_cam)
            self.get_logger().info(f"Translation (xyz): [{t_base_cam[0][0]:.6f}, {t_base_cam[1][0]:.6f}, {t_base_cam[2][0]:.6f}]")
            self.get_logger().info(f"Orientation (xyzw): [{q_base_cam[0]:.6f}, {q_base_cam[1]:.6f}, {q_base_cam[2]:.6f}, {q_base_cam[3]:.6f}]")
            self.get_logger().info("")
            
            # 추가 정보: T_ee_board도 출력
            q_ee_board = tf_transformations.quaternion_from_matrix(T_ee_board)
            self.get_logger().info("--- T_ee_board (end-effector → checkerboard) ---")
            self.get_logger().info(f"Translation (xyz): [{t_ee_board[0][0]:.6f}, {t_ee_board[1][0]:.6f}, {t_ee_board[2][0]:.6f}]")
            self.get_logger().info(f"Orientation (xyzw): [{q_ee_board[0]:.6f}, {q_ee_board[1]:.6f}, {q_ee_board[2]:.6f}, {q_ee_board[3]:.6f}]")
            self.get_logger().info("")
            # 결과를 YAML 파일로 저장
            result_data = {
                'calibration_result': {
                    'timestamp': datetime.now().isoformat(),
                    'num_samples': len(self.T_base_ee_list),
                    'method': 'CALIB_ROBOT_WORLD_HAND_EYE_SHAH',
                    'calibration_type': 'Hand-on-Base (Eye-to-Hand)',
                    'T_base_cam': {
                        'translation': {
                            'x': float(t_base_cam[0][0]),
                            'y': float(t_base_cam[1][0]),
                            'z': float(t_base_cam[2][0])
                        },
                        'rotation_quaternion': {
                            'x': float(q_base_cam[0]),
                            'y': float(q_base_cam[1]),
                            'z': float(q_base_cam[2]),
                            'w': float(q_base_cam[3])
                        },
                        'rotation_matrix': R_base_cam.tolist(),
                        'homogeneous_matrix': T_base_cam.tolist()
                    },
                    'T_ee_board': {
                        'translation': {
                            'x': float(t_ee_board[0][0]),
                            'y': float(t_ee_board[1][0]),
                            'z': float(t_ee_board[2][0])
                        },
                        'rotation_quaternion': {
                            'x': float(q_ee_board[0]),
                            'y': float(q_ee_board[1]),
                            'z': float(q_ee_board[2]),
                            'w': float(q_ee_board[3])
                        },
                        'rotation_matrix': R_ee_board.tolist(),
                        'homogeneous_matrix': T_ee_board.tolist()
                    },
                    'intermediate_results': {
                        'T_base_board': {
                            'translation': {
                                'x': float(t_base_board[0][0]),
                                'y': float(t_base_board[1][0]),
                                'z': float(t_base_board[2][0])
                            },
                            'rotation_matrix': R_base_board.tolist()
                        },
                        'T_ee_cam': {
                            'translation': {
                                'x': float(t_ee_cam[0][0]),
                                'y': float(t_ee_cam[1][0]),
                                'z': float(t_ee_cam[2][0])
                            },
                            'rotation_matrix': R_ee_cam.tolist()
                        }
                    }
                }
            }
            
            result_file = f'calibration_result_{datetime.now().strftime("%Y%m%d_%H%M%S")}.yaml'
            try:
                with open(result_file, 'w') as f:
                    yaml.dump(result_data, f, default_flow_style=False, sort_keys=False)
                self.get_logger().info(f"✅ Calibration result saved to {result_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to save result to file: {e}")
            
        except cv2.error as e:
            self.get_logger().error(f"OpenCV calibration failed: {e}")

        return response

    def _save_samples(self):
        """현재 수집된 샘플을 NPZ 파일로 저장"""
        if len(self.T_base_ee_list) == 0:
            self.get_logger().warn("No samples to save.")
            return False
        
        try:
            # NumPy 배열로 변환하여 저장
            T_base_ee_array = np.array(self.T_base_ee_list)
            T_cam_board_array = np.array(self.T_cam_board_list)
            
            np.savez(
                self.samples_file,
                T_base_ee=T_base_ee_array,
                T_cam_board=T_cam_board_array,
                num_samples=len(self.T_base_ee_list),
                timestamp=datetime.now().isoformat()
            )
            
            self.get_logger().info(f"💾 Saved {len(self.T_base_ee_list)} samples to {self.samples_file}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to save samples: {e}")
            return False
    
    def _load_samples(self):
        """저장된 샘플을 NPZ 파일에서 불러오기"""
        if not os.path.exists(self.samples_file):
            self.get_logger().info(f"No existing samples file found: {self.samples_file}")
            return False
        
        try:
            data = np.load(self.samples_file, allow_pickle=True)
            
            self.T_base_ee_list = list(data['T_base_ee'])
            self.T_cam_board_list = list(data['T_cam_board'])
            
            num_samples = len(self.T_base_ee_list)
            timestamp = data.get('timestamp', 'unknown')
            
            self.get_logger().info(f"📂 Loaded {num_samples} samples from {self.samples_file}")
            self.get_logger().info(f"   Timestamp: {timestamp}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load samples: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationCollectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()