#!/usr/bin/env python3
"""
Hand-Eye Calibration verification node.

카메라 화면 위 한 점을 클릭하면 (u, v) → depth 조회 → p_cam → p_base 변환을
거쳐 UR5e EE를 그 점 위로 이동시킨다. URScript movel을 직접 송신.

Keyboard:
    Click  : 점 선택 (변환 결과 화면에 표시)
    Enter  : 선택된 점으로 EE 이동 시작 (movel 송신)
    Space  : 즉시 정지 (stopl 송신)
    r      : 점 선택 리셋
    q      : 종료
"""

import glob
import os
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf_transformations
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


WINDOW_NAME = "Hand-Eye Verification"


class VerificationNode(Node):
    def __init__(self):
        super().__init__('hand_eye_verification_node')

        # 파라미터 선언
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('move_speed', 0.05)         # m/s
        self.declare_parameter('move_accel', 0.1)          # m/s^2
        self.declare_parameter('stop_decel', 2.0)          # m/s^2
        self.declare_parameter('reach_min', 0.20)          # m, base 원점에서 거리
        self.declare_parameter('reach_max', 3.85)          # m
        self.declare_parameter('z_min', 0.0)               # m, base frame 최소 z
        self.declare_parameter('rx', 3.14159)              # axis-angle (top-down)
        self.declare_parameter('ry', 0.0)
        self.declare_parameter('rz', 0.0)
        self.declare_parameter('z_offset', 0.05)           # m, target 위로 hover
        self.declare_parameter('base_frame', 'ur5e_base_link')
        self.declare_parameter('ee_frame', 'ur5e_tool0')

        self.move_speed = self.get_parameter('move_speed').value
        self.move_accel = self.get_parameter('move_accel').value
        self.stop_decel = self.get_parameter('stop_decel').value
        self.reach_min = self.get_parameter('reach_min').value
        self.reach_max = self.get_parameter('reach_max').value
        self.z_min = self.get_parameter('z_min').value
        self.rx = self.get_parameter('rx').value
        self.ry = self.get_parameter('ry').value
        self.rz = self.get_parameter('rz').value
        self.z_offset = self.get_parameter('z_offset').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value

        # 캘리브레이션 결과 로드 (Eye-on-Base 또는 Eye-in-Hand 자동 감지)
        cal_file = self.get_parameter('calibration_file').value
        self.calib_type, self.T_calib = self._load_calibration(cal_file)
        self.get_logger().info(f"Calibration type: {self.calib_type}")
        self.get_logger().info(f"Calibration matrix:\n{self.T_calib}")

        # Eye-in-Hand인 경우 TF listener 필요 (현재 T_base_ee 조회)
        if self.calib_type == 'eye_in_hand':
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info(
                f"Will look up TF {self.base_frame} → {self.ee_frame} "
                "to compute live T_base_cam.")
        else:
            self.tf_buffer = None
            self.tf_listener = None

        # 상태 변수
        self.bridge = CvBridge()
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.K: Optional[np.ndarray] = None
        self.fx = self.fy = self.cx = self.cy = None
        self.click_uv: Optional[Tuple[int, int]] = None
        self.click_p_base: Optional[np.ndarray] = None     # 유효한 클릭 좌표
        self.last_tcp_pose: Optional[PoseStamped] = None
        self.move_in_progress = False

        # 구독
        self.create_subscription(
            Image, '/zed/zed_node/rgb/color/rect/image', self.on_rgb, 10)
        self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.on_depth, 10)
        self.create_subscription(
            CameraInfo, '/zed/zed_node/rgb/color/rect/camera_info', self.on_camera_info, 10)
        self.create_subscription(
            PoseStamped, '/tcp_pose_broadcaster/pose', self.on_tcp_pose, 10)

        # URScript publisher
        self.script_pub = self.create_publisher(
            String, '/urscript_interface/script_command', 10)

        # OpenCV 윈도우 + 마우스 콜백
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, self.on_mouse)

        # GUI/키 입력 30 Hz 타이머
        self.gui_timer = self.create_timer(1.0 / 30.0, self.gui_tick)

        self.get_logger().info("Verification node started.")
        self.get_logger().info(
            "Click=select  Enter=move  Space=STOP  r=reset  q=quit")

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------
    def _load_calibration(self, path: str) -> Tuple[str, np.ndarray]:
        """YAML에서 calibration matrix를 로드하고 type을 반환.

        Returns:
            (calib_type, T) where
              calib_type = 'eye_on_base' → T = T_base_cam
              calib_type = 'eye_in_hand' → T = T_ee_cam
        """
        if not path:
            workspace = os.path.expanduser('~/Desktop/Hand_Eye_Calibration')
            candidates = sorted(glob.glob(
                os.path.join(workspace, 'calibration_result_*.yaml')))
            if not candidates:
                raise FileNotFoundError(
                    f"No calibration_result_*.yaml in {workspace}. "
                    "Pass -p calibration_file:=<path>")
            path = candidates[-1]
            self.get_logger().info(f"Auto-selected latest calibration: {path}")
        with open(path) as f:
            data = yaml.safe_load(f)

        result = data.get('calibration_result', {})
        type_str = result.get('calibration_type', '')

        if 'T_base_cam' in result:
            T = np.array(result['T_base_cam']['homogeneous_matrix'])
            calib_type = 'eye_on_base'
        elif 'T_ee_cam' in result:
            T = np.array(result['T_ee_cam']['homogeneous_matrix'])
            calib_type = 'eye_in_hand'
        else:
            raise KeyError(
                f"YAML {path} has neither T_base_cam nor T_ee_cam under "
                "calibration_result.")

        if T.shape != (4, 4):
            raise ValueError(f"Matrix shape {T.shape}, expected (4, 4)")
        self.get_logger().info(f"YAML calibration_type field: {type_str}")
        return calib_type, T

    # ------------------------------------------------------------------
    # ROS 콜백
    # ------------------------------------------------------------------
    def on_camera_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.fx, self.fy = self.K[0, 0], self.K[1, 1]
            self.cx, self.cy = self.K[0, 2], self.K[1, 2]
            self.get_logger().info(
                f"Camera intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, "
                f"cx={self.cx:.2f}, cy={self.cy:.2f}")

    def on_rgb(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")

    def on_depth(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            elif msg.encoding == '16UC1':
                # mm → m
                d = self.bridge.imgmsg_to_cv2(msg, '16UC1')
                self.latest_depth = d.astype(np.float32) / 1000.0
            else:
                self.get_logger().warn(
                    f"Unknown depth encoding: {msg.encoding}",
                    throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def on_tcp_pose(self, msg: PoseStamped):
        self.last_tcp_pose = msg

    def on_mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._handle_click(x, y)

    # ------------------------------------------------------------------
    # 클릭 → 좌표 변환
    # ------------------------------------------------------------------
    def _get_T_base_cam(self) -> Optional[np.ndarray]:
        """현재 T_base_cam을 계산. eye_on_base는 고정값, eye_in_hand는 TF 사용."""
        if self.calib_type == 'eye_on_base':
            return self.T_calib
        # eye_in_hand: T_base_cam(t) = T_base_ee(t) @ T_ee_cam
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(
                f"TF {self.base_frame}→{self.ee_frame} unavailable: {e}",
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        T_base_ee = tf_transformations.quaternion_matrix(
            [q.x, q.y, q.z, q.w])
        T_base_ee[:3, 3] = [t.x, t.y, t.z]
        return T_base_ee @ self.T_calib

    def _handle_click(self, u: int, v: int):
        if self.latest_depth is None or self.K is None:
            self.get_logger().warn("Depth or intrinsics not yet received.")
            return

        depth = self.latest_depth
        h, w = depth.shape[:2]
        if not (1 <= u < w - 1 and 1 <= v < h - 1):
            self.get_logger().warn(f"Click ({u},{v}) too close to border.")
            return

        # 3x3 패치에서 유효한 depth만 모아 median 사용 (NaN 노이즈 회피)
        patch = depth[v - 1:v + 2, u - 1:u + 2]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size < 5:
            self.get_logger().warn(
                f"Insufficient valid depth at ({u},{v}). "
                f"Got {valid.size}/9 valid pixels.")
            self.click_uv = (u, v)
            self.click_p_base = None
            return
        Z = float(np.median(valid))

        # Pinhole back-projection
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        p_cam = np.array([X, Y, Z, 1.0])

        # 현재 T_base_cam 결정 (모드별)
        T_base_cam = self._get_T_base_cam()
        if T_base_cam is None:
            self.get_logger().warn(
                "Cannot compute T_base_cam (TF not available yet).")
            self.click_uv = (u, v)
            self.click_p_base = None
            return

        # Base frame 변환
        p_base = T_base_cam @ p_cam
        target_xyz = p_base[:3]

        # Reachability 체크
        reach = float(np.linalg.norm(target_xyz))
        if reach < self.reach_min or reach > self.reach_max:
            self.get_logger().warn(
                f"Target out of reach: {reach:.3f}m "
                f"(allowed: {self.reach_min}-{self.reach_max})")
            self.click_uv = (u, v)
            self.click_p_base = None
            return
        if target_xyz[2] < self.z_min:
            self.get_logger().warn(
                f"Target below z_min: z={target_xyz[2]:.3f} < {self.z_min}")
            self.click_uv = (u, v)
            self.click_p_base = None
            return

        self.click_uv = (u, v)
        self.click_p_base = target_xyz
        self.get_logger().info(
            f"Click ({u},{v}) Z={Z:.3f}m → "
            f"p_cam=[{X:+.3f},{Y:+.3f},{Z:+.3f}] → "
            f"p_base=[{target_xyz[0]:+.3f},{target_xyz[1]:+.3f},"
            f"{target_xyz[2]:+.3f}] (reach {reach:.3f}m). "
            f"Press Enter to move (will hover {self.z_offset*1000:.0f}mm above).")

    # ------------------------------------------------------------------
    # URScript 송신
    # ------------------------------------------------------------------
    def _send_movel(self):
        if self.click_p_base is None:
            self.get_logger().warn("No valid click point to send.")
            return
        x, y, z = self.click_p_base
        z_target = z + self.z_offset
        cmd = (
            f"movel(p[{x:.4f},{y:.4f},{z_target:.4f},"
            f"{self.rx:.4f},{self.ry:.4f},{self.rz:.4f}], "
            f"a={self.move_accel}, v={self.move_speed})\n"
        )
        self.script_pub.publish(String(data=cmd))
        self.move_in_progress = True
        self.get_logger().info(f"Sent: {cmd.strip()}")

    def _send_stop(self):
        cmd = f"stopl({self.stop_decel})\n"
        self.script_pub.publish(String(data=cmd))
        self.move_in_progress = False
        self.get_logger().warn(f"STOP sent: {cmd.strip()}")

    # ------------------------------------------------------------------
    # GUI 렌더 + 키 입력
    # ------------------------------------------------------------------
    def gui_tick(self):
        if self.latest_rgb is None:
            return
        img = self.latest_rgb.copy()

        # 클릭 마커
        if self.click_uv is not None:
            u, v = self.click_uv
            color = (0, 0, 255) if self.click_p_base is not None else (0, 165, 255)
            cv2.circle(img, (u, v), 8, color, 2)
            cv2.drawMarker(img, (u, v), color, cv2.MARKER_CROSS, 16, 1)
            if self.click_p_base is not None:
                x, y, z = self.click_p_base
                cv2.putText(
                    img, f"base: x={x:.3f} y={y:.3f} z={z:.3f}",
                    (u + 12, v),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 상태 바 (상단)
        mode_label = ('EoB' if self.calib_type == 'eye_on_base' else 'EiH')
        status = (
            f"[{mode_label}]  "
            f"K {'OK' if self.K is not None else '...'}  "
            f"D {'OK' if self.latest_depth is not None else '...'}  "
            f"TCP {'OK' if self.last_tcp_pose is not None else '...'}  "
            f"Move:{'ACTIVE' if self.move_in_progress else 'idle'}"
        )
        cv2.putText(img, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(
            img, "Click=select  Enter=move  Space=STOP  r=reset  q=quit",
            (10, img.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow(WINDOW_NAME, img)
        key = cv2.waitKey(1) & 0xFF
        if key == 13 or key == 10:        # Enter
            self._send_movel()
        elif key == 32:                    # Space
            self._send_stop()
        elif key == ord('r'):
            self.click_uv = None
            self.click_p_base = None
            self.get_logger().info("Reset.")
        elif key == ord('q'):
            self.get_logger().info("Quit requested.")
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = VerificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
