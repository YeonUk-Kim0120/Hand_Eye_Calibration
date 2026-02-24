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

        # 작업 디렉토리를 프로젝트 루트로 설정
        self.workspace_root = os.path.expanduser('~/Desktop/Hand_Eye_Calibration')
        if os.path.exists(self.workspace_root):
            os.chdir(self.workspace_root)
            self.get_logger().info(f"Working directory: {self.workspace_root}")
        else:
            self.get_logger().warn(f"Workspace not found: {self.workspace_root}, using current directory")

        # 캘리브레이션 타입 파라미터
        self.declare_parameter('calibration_type', 'eye_on_base')  # 'eye_on_base' or 'eye_in_hand'
        self.calibration_type = self.get_parameter('calibration_type').value
        if self.calibration_type not in ('eye_on_base', 'eye_in_hand'):
            self.get_logger().error(f"Invalid calibration_type: '{self.calibration_type}'. Must be 'eye_on_base' or 'eye_in_hand'.")
            raise ValueError(f"Invalid calibration_type: '{self.calibration_type}'")

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

        self.get_logger().info("Calibration collector node started (easy_handeye2 style).")
        self.get_logger().info(f"Calibration type: {self.calibration_type}")
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
            self.lock = False
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
        min_samples = 4 # 캘리브레이션을 위한 최소 샘플 수
        if len(self.T_base_ee_list) < min_samples:
            self.get_logger().error(f"Need at least {min_samples} samples to run calibration. Currently have {len(self.T_base_ee_list)}.")
            return response

        is_eye_on_base = (self.calibration_type == 'eye_on_base')
        mode_str = 'Eye-on-Base' if is_eye_on_base else 'Eye-in-Hand'

        self.get_logger().info(f"Running {mode_str} calibration with {len(self.T_base_ee_list)} samples...")

        # cv2.calibrateHandEye는 AX = XB 를 풀어줌 (원래 Eye-in-Hand용)
        #
        # Eye-in-Hand:
        #   A = T_base_ee (base → ee),  B = T_cam_board (cam → board)
        #   → X = T_ee_cam (ee → camera)
        #
        # Eye-on-Base (트릭):
        #   A = inv(T_base_ee) = T_ee_base,  B = T_cam_board
        #   → X = T_base_cam (base → camera)
        
        try:
            # 입력 데이터 준비
            R_robot_list = []
            t_robot_list = []
            R_cam_board_list = []
            t_cam_board_list = []
            
            for T_base_ee, T_cam_board in zip(self.T_base_ee_list, self.T_cam_board_list):
                if is_eye_on_base:
                    # Eye-on-Base: 역방향 입력 (ee → base)
                    T_robot = np.linalg.inv(T_base_ee)
                else:
                    # Eye-in-Hand: 그대로 입력 (base → ee)
                    T_robot = T_base_ee
                
                R_robot_list.append(T_robot[:3, :3])
                t_robot_list.append(T_robot[:3, 3].reshape(3, 1))
                
                R_cam_board_list.append(T_cam_board[:3, :3])
                t_cam_board_list.append(T_cam_board[:3, 3].reshape(3, 1))
            
            # cv2.calibrateHandEye 호출
            R_result, t_result = cv2.calibrateHandEye(
                R_gripper2base=R_robot_list,
                t_gripper2base=t_robot_list,
                R_target2cam=R_cam_board_list,
                t_target2cam=t_cam_board_list,
                method=cv2.CALIB_HAND_EYE_TSAI
            )
            
            # 결과 4x4 행렬 조합
            T_result = np.eye(4)
            T_result[:3, :3] = R_result
            T_result[:3, 3] = t_result.flatten()
            
            # 검증용 부수 변환 계산
            # Eye-on-Base: T_cam_board = inv(T_base_cam) * T_base_ee * T_ee_board
            #   → T_ee_board = inv(T_base_ee) * T_base_cam * T_cam_board
            # Eye-in-Hand: T_cam_board = inv(T_ee_cam) * inv(T_base_ee) * T_base_board
            #   → T_base_board = T_base_ee * T_ee_cam * T_cam_board
            T_secondary_list = []
            for T_base_ee, T_cam_board in zip(self.T_base_ee_list, self.T_cam_board_list):
                if is_eye_on_base:
                    # T_ee_board = inv(T_base_ee) * T_base_cam * T_cam_board
                    T_sec = np.linalg.inv(T_base_ee) @ T_result @ T_cam_board
                else:
                    # T_base_board = T_base_ee * T_ee_cam * T_cam_board
                    T_sec = T_base_ee @ T_result @ T_cam_board
                T_secondary_list.append(T_sec)
            
            T_secondary_avg = np.mean(T_secondary_list, axis=0)
            R_secondary = T_secondary_avg[:3, :3]
            t_secondary = T_secondary_avg[:3, 3].reshape(3, 1)
            
            # Residual 검증
            residuals = []
            for T_base_ee, T_cam_board in zip(self.T_base_ee_list, self.T_cam_board_list):
                if is_eye_on_base:
                    # 우변: inv(T_base_cam) * T_base_ee * T_ee_board_avg
                    T_cam_board_computed = np.linalg.inv(T_result) @ T_base_ee @ T_secondary_avg
                else:
                    # 우변: inv(T_ee_cam) * inv(T_base_ee) * T_base_board_avg
                    T_cam_board_computed = np.linalg.inv(T_result) @ np.linalg.inv(T_base_ee) @ T_secondary_avg
                
                residual = np.linalg.norm(T_cam_board[:3, 3] - T_cam_board_computed[:3, 3])
                residuals.append(residual)

            # 쿼터니언 계산
            q_result = tf_transformations.quaternion_from_matrix(T_result)
            q_secondary = tf_transformations.quaternion_from_matrix(T_secondary_avg)

            # 모드별 라벨 설정
            if is_eye_on_base:
                result_name = 'T_base_cam'
                result_desc = 'Robot Base → Camera (camera is fixed)'
                secondary_name = 'T_ee_board'
                secondary_desc = 'End-Effector → Checkerboard (board on EE)'
                eq_str = 'T_cam_board = inv(T_base_cam) * T_base_ee * T_ee_board'
                yaml_type = 'Eye-on-Base'
                yaml_desc = 'Camera fixed, board on EE. calibrateHandEye with inv(T_base_ee).'
            else:
                result_name = 'T_ee_cam'
                result_desc = 'End-Effector → Camera (camera on EE)'
                secondary_name = 'T_base_board'
                secondary_desc = 'Robot Base → Checkerboard (board is fixed)'
                eq_str = 'T_cam_board = inv(T_ee_cam) * inv(T_base_ee) * T_base_board'
                yaml_type = 'Eye-in-Hand'
                yaml_desc = 'Camera on EE, board fixed. calibrateHandEye with T_base_ee directly.'

            # 결과 출력
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"=== {mode_str} Calibration Results ===")
            self.get_logger().info("=" * 60)
            self.get_logger().info("")
            self.get_logger().info(f"🎯 {result_name} ({result_desc}) - PRIMARY RESULT")
            self.get_logger().info(f"   Translation (xyz) [m]: [{t_result[0][0]:.6f}, {t_result[1][0]:.6f}, {t_result[2][0]:.6f}]")
            self.get_logger().info(f"   Orientation (xyzw): [{q_result[0]:.6f}, {q_result[1]:.6f}, {q_result[2]:.6f}, {q_result[3]:.6f}]")
            self.get_logger().info("")
            self.get_logger().info(f"   Homogeneous Matrix:\n{T_result}")
            self.get_logger().info("")
            
            self.get_logger().info(f"📋 {secondary_name} ({secondary_desc}) - VERIFICATION")
            self.get_logger().info(f"   This should be constant across all samples.")
            self.get_logger().info(f"   Translation (xyz) [m]: [{t_secondary[0][0]:.6f}, {t_secondary[1][0]:.6f}, {t_secondary[2][0]:.6f}]")
            self.get_logger().info(f"   Orientation (xyzw): [{q_secondary[0]:.6f}, {q_secondary[1]:.6f}, {q_secondary[2]:.6f}, {q_secondary[3]:.6f}]")
            self.get_logger().info("")
            
            # 일관성 검증: residual 통계
            if len(residuals) > 0:
                residuals_array = np.array(residuals)
                mean_residual = np.mean(residuals_array)
                max_residual = np.max(residuals_array)
                
                self.get_logger().info("📊 Consistency Check (Equation Residuals):")
                self.get_logger().info(f"   {eq_str}")
                self.get_logger().info(f"   Mean residual [mm]: {mean_residual*1000:.3f}")
                self.get_logger().info(f"   Max residual [mm]: {max_residual*1000:.3f}")
                
                if max_residual * 1000 < 5.0:
                    self.get_logger().info(f"   ✅ Excellent consistency!")
                elif max_residual * 1000 < 20.0:
                    self.get_logger().warn(f"   ⚠️  Moderate consistency")
                else:
                    self.get_logger().warn(f"   ❌ Poor consistency - Check your data!")
                self.get_logger().info("")
            
            self.get_logger().info("=" * 60)

            # 결과를 YAML 파일로 저장
            result_data = {
                'calibration_result': {
                    'timestamp': datetime.now().isoformat(),
                    'num_samples': len(self.T_base_ee_list),
                    'method': 'CALIB_HAND_EYE_TSAI',
                    'calibration_type': yaml_type,
                    'description': yaml_desc,
                    result_name: {
                        'description': result_desc,
                        'translation': {
                            'x': float(t_result[0][0]),
                            'y': float(t_result[1][0]),
                            'z': float(t_result[2][0])
                        },
                        'rotation_quaternion': {
                            'x': float(q_result[0]),
                            'y': float(q_result[1]),
                            'z': float(q_result[2]),
                            'w': float(q_result[3])
                        },
                        'rotation_matrix': R_result.tolist(),
                        'homogeneous_matrix': T_result.tolist()
                    },
                    secondary_name: {
                        'description': secondary_desc,
                        'translation': {
                            'x': float(t_secondary[0][0]),
                            'y': float(t_secondary[1][0]),
                            'z': float(t_secondary[2][0])
                        },
                        'rotation_quaternion': {
                            'x': float(q_secondary[0]),
                            'y': float(q_secondary[1]),
                            'z': float(q_secondary[2]),
                            'w': float(q_secondary[3])
                        },
                        'rotation_matrix': R_secondary.tolist(),
                        'homogeneous_matrix': T_secondary_avg.tolist(),
                        'note': 'Should be consistent across all samples. Computed as average.'
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
