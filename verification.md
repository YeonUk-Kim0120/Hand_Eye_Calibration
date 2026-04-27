# Hand-Eye Calibration Verification

캘리브레이션 결과(`T_base_cam`)가 정확한지 **물리적으로 검증**하는 절차.
사용자가 카메라 화면 위에서 한 점(예: 토마토)을 클릭하면, 그 점의
3D 좌표를 robot base frame으로 변환해 UR5e end-effector를 그 위치까지
이동시킨다. 손끝과 클릭 지점의 일치 여부로 캘리브레이션 품질을 육안 검증.

---

## 0. 전제 조건

- `T_base_cam` 결과 YAML 파일이 존재 (예: `calibration_result_YYYYMMDD_HHMMSS.yaml`)
- ZED ROS 2 wrapper 실행 중 (`/zed/zed_node/rgb/color/rect/image`, `/zed/zed_node/depth/depth_registered`, `/zed/zed_node/rgb/color/rect/camera_info` 발행)
- UR5e ROS 2 driver 실행 중 (`ros2 launch ur5e_control full_robot_system.launch.py`)
  - `/urscript_interface/script_command` (std_msgs/String) 살아있는지 `ros2 topic list | grep urscript` 로 확인
  - `/tcp_pose_broadcaster/pose` 살아있는지도 확인 (현재 EE 위치 디버그용)
- 로봇 주변 작업 영역 정리 (충돌 가능 물체 제거)
- **충돌 회피 자동 검증 없음** — 사용자 책임으로 안전한 점만 클릭

---

## 1. 사용자 인터페이스

### 1-1. 입력
- ZED RGB 화면 OpenCV window 표시
- 마우스 좌클릭 → (u, v) 픽셀 좌표 획득
- 클릭 후 화면에 빨간 점과 변환 결과 (base frame XYZ) 오버레이

### 1-2. 키보드 명령
| 키 | 동작 |
|---|---|
| 좌클릭 | 검증 점 선택 → 화면에 좌표 표시 |
| `Enter` | 선택된 점으로 EE 이동 시작 (확인) |
| `Space` | **즉시 정지** (URScript `stopl(2.0)` 전송) |
| `r` | 점 선택 리셋 |
| `q` | 노드 종료 |

### 1-3. 안전 정책
- 클릭만 하면 이동 안 함 — **반드시 `Enter` 키 입력**해야 명령 송신
- URScript 속도/가속도 매우 낮게: `v=0.05` (5 cm/s), `a=0.1` (0.1 m/s²)
- 이동 중 `Space` 키 입력 시 즉시 `stopl(2.0)` 송신 → 2 m/s² 감속 정지
- 클릭한 점이 reachability 범위 밖이면 송신 거부:
  - `0.2 m < distance from base < 0.85 m`
  - `z_base > 0.0` (base 평면 위쪽만)
- 이동 시작 전 콘솔에 최종 좌표 출력 → 사용자 확인

---

## 2. 좌표 변환 파이프라인

```
[1] 마우스 클릭 (u, v) [pixel]
        ↓
[2] depth_img[v, u] → Z [m, float32]   (NaN/0 거부)
        ↓
[3] Pinhole back-projection:
       X = (u - cx) * Z / fx
       Y = (v - cy) * Z / fy
       p_cam = [X, Y, Z, 1]            (zed_left_camera_optical_frame, m)
        ↓
[4] p_base = T_base_cam @ p_cam        (ur5e_base_link, m)
        ↓
[5] Top-down 자세 부여:
       rx, ry, rz = π, 0, 0            (gripper z축 ↓ 향함, axis-angle)
        ↓
[6] URScript movel(p[x, y, z, rx, ry, rz], a=0.1, v=0.05)
        ↓
[7] /urscript_interface/script_command publish
        ↓
[8] EE가 클릭한 점 위로 이동 → 육안 검증
```

### 2-1. 카메라 intrinsic 읽기

`/zed/zed_node/rgb/color/rect/camera_info` 한 번 구독 → `info_msg.k` (9개 float):
```python
K = np.array(info_msg.k).reshape(3, 3)
fx, fy = K[0, 0], K[1, 1]
cx, cy = K[0, 2], K[1, 2]
```
detector 노드와 동일한 intrinsic 사용 → 일관성 보장.

### 2-2. T_base_cam 로딩

```python
import yaml
with open('calibration_result_YYYYMMDD_HHMMSS.yaml') as f:
    data = yaml.safe_load(f)
T_base_cam = np.array(
    data['calibration_result']['T_base_cam']['homogeneous_matrix']
)
```

### 2-3. Top-down 자세 (axis-angle)

URScript는 회전을 **axis-angle 벡터** `(rx, ry, rz)`로 받음. 크기 = 회전 각도(rad), 방향 = 회전축.

Top-down (gripper z축이 base z축 반대 방향, 즉 아래로):
- 회전축: x축 (또는 y축, 둘 다 가능)
- 회전 각도: π
- 표현: `(rx, ry, rz) = (3.14159, 0, 0)`

> 주의: UR controller의 base frame과 우리가 캘리브레이션한 `ur5e_base_link`가 동일 frame인지 확인 필요. 일반적으로 둘은 같지만, `/tcp_pose_broadcaster/pose` 토픽으로 한 번 sanity check 권장.

---

## 3. 구현 메모

### 3-1. 새 노드 위치
`src/hand_eye_calibration/hand_eye_calibration/verification_node.py`

### 3-2. setup.py에 entry point 추가
```python
'verify = hand_eye_calibration.verification_node:main',
```

### 3-3. 의존 토픽
| 토픽 | 메시지 타입 | 용도 |
|---|---|---|
| `/zed/zed_node/rgb/color/rect/image` | sensor_msgs/Image | 화면 표시, 클릭 받기 |
| `/zed/zed_node/depth/depth_registered` | sensor_msgs/Image (32FC1) | (u,v)에서 Z 읽기 |
| `/zed/zed_node/rgb/color/rect/camera_info` | sensor_msgs/CameraInfo | intrinsic K |
| `/tcp_pose_broadcaster/pose` | geometry_msgs/PoseStamped | 현재 EE 위치 (디버그) |
| `/urscript_interface/script_command` (publish) | std_msgs/String | 명령 송신 |

### 3-4. URScript 명령 포맷
```python
def make_movel(x, y, z, rx, ry, rz, a=0.1, v=0.05):
    return f"movel(p[{x:.4f},{y:.4f},{z:.4f},{rx:.4f},{ry:.4f},{rz:.4f}], a={a}, v={v})"

def make_stop(a=2.0):
    return f"stopl({a})"
```

URScript는 끝에 개행이 필요할 수 있음 (`+ "\n"`). 첫 시도 후 안 먹히면 추가.

### 3-5. Depth 픽셀 인덱싱 주의
- numpy: `depth_img[v, u]` (row, col 순서)
- 클릭 좌표가 이미지 경계 밖이면 거부
- `Z`가 0 또는 NaN이면 "유효하지 않은 depth" 메시지 출력 후 거부
- ZED depth는 종종 노이즈로 인해 한 픽셀이 NaN이지만 주변은 정상 → **3×3 medianfilter** 후 중앙값 사용 권장

```python
patch = depth_img[v-1:v+2, u-1:u+2]
valid = patch[np.isfinite(patch) & (patch > 0)]
if len(valid) < 5:
    return  # 거부
Z = float(np.median(valid))
```

---

## 4. 검증 절차 (현장)

### 4-1. 사전 점검
- [ ] 캘리브레이션 결과 YAML 존재 및 residual `< 5 mm`
- [ ] ZED 토픽 정상 수신 확인 (`ros2 topic hz /zed/zed_node/rgb/color/rect/image`)
- [ ] UR driver 정상 (`ros2 topic echo /tcp_pose_broadcaster/pose --once`)
- [ ] **Teach pendant E-stop 즉시 누를 수 있는 위치에 사람 대기**

### 4-2. 첫 시도 (가까운 안전한 점)
- [ ] 책상 위 카메라에서 **30~50 cm 정도** 떨어진, 주변에 장애물 없는 표면을 클릭
- [ ] 콘솔에 표시된 base 좌표를 보고 사람 머릿속으로 sanity check
  - x: 보통 +0.3 ~ +0.7 m
  - y: 보통 -0.3 ~ +0.3 m
  - z: 책상 높이에 맞는 값 (예: 0.0 ~ 0.2 m)
- [ ] `Enter` → EE가 점 위로 도착하는지 관찰
- [ ] **손끝과 클릭 지점 거리** 측정 (자/줄자) → 5 mm 이내면 우수

### 4-3. 토마토 클릭
- [ ] 토마토를 카메라 시야 내에 배치
- [ ] 토마토 중심 클릭
- [ ] EE가 토마토 정점 위쪽으로 이동하는지 관찰
- [ ] 좌/우/앞/뒤 다른 위치 토마토 3~5개 더 시도

### 4-4. 합격 기준
- 평균 오차 `< 5 mm` → **Excellent**, 즉 pick-and-place 가능
- 평균 오차 `5~20 mm` → **Moderate**, 작은 물체 grasp는 어려울 수 있음
- 평균 오차 `> 20 mm` → **Fail**, 캘리브레이션 재실시 필요

---

## 5. 흔한 실패 원인 진단

| 증상 | 원인 가능성 | 점검 |
|---|---|---|
| 항상 일정 방향으로 5~10 cm 어긋남 | T_base_cam translation 일부 component 부정확 | residual 재확인, 샘플 다양성 부족 의심 |
| 회전적으로 어긋남 (점에 따라 다른 방향) | T_base_cam rotation 부정확 | sample 자세 다양성 확인, `_average_transforms` 결과 점검 |
| 카메라 멀어질수록 오차 커짐 | rotation 오차 (각도가 거리에 곱해짐) | 회전 부분 재캘리브레이션 |
| EE가 엉뚱한 곳 (1m 이상 어긋남) | frame 혼동 (optical vs ROS) | `T_base_cam`이 optical frame 기준인지 확인 |
| Depth가 항상 NaN/0 | depth_registered 미발행 또는 RGB 정렬 안 됨 | ZED launch 파라미터 확인 |
| `urscript_interface` 명령 무반응 | controller 활성화 안 됨 또는 robot mode 문제 | teach pendant에서 remote control mode 확인 |

---

## 6. 파일 산출물

- `verification_node.py` — 검증 도구 노드
- `verification_log_YYYYMMDD_HHMMSS.csv` (선택) — 클릭 좌표 / 변환 결과 / EE 도달 좌표 / 오차 기록
