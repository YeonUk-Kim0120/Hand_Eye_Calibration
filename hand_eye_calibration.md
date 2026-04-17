# Hand-Eye Calibration 개념 정리

ZED 2i 카메라와 UR5e 로봇팔을 이용한 Hand-Eye Calibration의 원리, 수식, 그리고 결과를 실제 pick-and-place에 어떻게 쓰는지까지 논리적으로 정리한 문서.

---

## 1. 기초: 변환 행렬 표기법

### `T_A_B`는 무엇인가

`T_A_B`는 4x4 homogeneous 변환 행렬이며, 두 가지 관점으로 해석할 수 있다:

- **프레임 관점**: A 프레임에서 본 B 프레임의 pose. "A에서 B가 어디 있고, 어떻게 회전되어 있는지"
- **점 변환 관점**: B 좌표계의 점을 A 좌표계로 바꾸는 연산자. 즉 `p_A = T_A_B @ p_B`

예:
- `T_base_cam`: base에서 본 camera의 위치와 방향. translation이 `(1.0, 0.0, 0.5)`라면 base 원점에서 X축으로 1m, Z축으로 0.5m 떨어진 곳에 카메라 원점이 있음
- `T_cam_object`: camera에서 본 object의 위치와 방향

### 체인 규칙

여러 변환을 연결해 새로운 변환을 만들 수 있다. **중간 프레임이 상쇄**되도록 곱하면 된다:

```
T_A_C = T_A_B @ T_B_C
```

예:
```
T_base_object = T_base_cam @ T_cam_object
```
→ base에서 본 object의 pose. 이게 pick-and-place의 궁극적 목표.

### 역변환

`T_A_B`의 역은 `T_B_A`이다:
```python
T_B_A = np.linalg.inv(T_A_B)
```

---

## 2. 시스템에 존재하는 변환들

### 항상 알 수 있는 것 (실시간 측정)

| 변환 | 의미 | 출처 |
|------|------|------|
| `T_base_ee` | base → end-effector | 로봇 드라이버가 발행하는 TF (Forward Kinematics) |
| `T_cam_object` | camera → object | 비전 파이프라인 (체커보드 검출, FoundationPose, ArUco 등) |

### 구하고 싶은 것 (calibration의 출력)

| 모드 | 미지수 | 의미 |
|------|--------|------|
| Eye-on-Base | `T_base_cam` | 카메라가 base에 고정 |
| Eye-in-Hand | `T_ee_cam` | 카메라가 end-effector에 부착 |

카메라는 기계적으로 장착된 후 위치를 직접 자로 재서 정확히 알기 어렵다. 특히 회전(몇 도 기울어졌는지)은 측정이 매우 어렵다. 그래서 **수학적으로 계산**하는 절차가 Hand-Eye Calibration이다.

---

## 3. Hand-Eye Calibration이 푸는 문제

### 기본 아이디어

로봇을 여러 자세로 움직이면서:
- 각 자세에서 로봇 FK로 `T_base_ee_i`를 알 수 있음
- 각 자세에서 카메라로 체커보드 pose `T_cam_board_i`를 측정

체커보드는 어딘가에 **고정**되어 있기 때문에, 같은 기하 루프가 모든 샘플에서 성립해야 한다. 이 조건으로부터 미지의 카메라 위치를 수학적으로 유도.

### Eye-in-Hand 방정식

카메라가 EE에 붙어있고 체커보드는 작업대에 고정되어 있다면:
```
T_base_board = T_base_ee_i @ T_ee_cam @ T_cam_board_i   (모든 i에서 동일)
```
`T_base_board`는 고정(체커보드가 작업대에 고정이라), `T_ee_cam`도 고정(카메라가 EE에 단단히 장착)이므로 이 방정식의 미지수는 둘. 서로 다른 자세 두 개를 써서 방정식을 세우면 `AX = XB` 형태로 정리되어 `T_ee_cam`을 풀 수 있다. 이게 OpenCV `cv2.calibrateHandEye`가 해결하는 문제다.

### Eye-on-Base 방정식

카메라가 base에 고정되어 있고 체커보드가 EE에 부착되어 있다면:
```
T_base_board_i = T_base_cam @ T_cam_board_i
             = T_base_ee_i @ T_ee_board   (모든 i에서)
```
→ `T_base_cam @ T_cam_board_i = T_base_ee_i @ T_ee_board`

미지수는 `T_base_cam`과 `T_ee_board`. 원리는 같지만 수식 형태가 다르다.

### OpenCV의 트릭

`cv2.calibrateHandEye`는 본래 **eye-in-hand 전용**으로 설계되었다. 그러나 eye-on-base 문제도 **입력을 뒤집기만 하면** 같은 함수로 풀린다:

```
Eye-in-Hand:
  A = T_base_ee,         B = T_cam_board  →  X = T_ee_cam
Eye-on-Base:
  A = inv(T_base_ee),    B = T_cam_board  →  X = T_base_cam
```

현재 collector 코드(`calibration_collector_node.py`)는 `calibration_type` 파라미터에 따라 eye-on-base일 때 자동으로 `T_base_ee`를 invert해서 `calibrateHandEye`에 넣는다.

---

## 4. 체커보드의 역할

체커보드는 **캘리브레이션 타겟**일 뿐이다. 결과에 "체커보드의 뭔가"가 남지 않는다. 캘리브레이션이 끝나면 체커보드는 **떼어낸다**.

체커보드가 필요한 이유:
- 기하학적으로 정확한 3D 형상(격자)이 알려져 있어 카메라 pose 추정이 정밀
- `cv2.findChessboardCorners` + `solvePnP`로 robust하게 검출 가능

즉 체커보드는 "카메라가 정확한 pose를 볼 수 있도록" 해주는 **임시 기준**일 뿐이고, 진짜 목표는 `T_base_cam` (혹은 `T_ee_cam`)을 얻는 것.

---

## 5. 결과 해석

### 주 결과 (Primary)

Eye-on-Base:
- `T_base_cam`: base 프레임에서 본 카메라의 pose
- translation (x, y, z) 단위는 **미터**
- rotation은 쿼터니언 `(x, y, z, w)` (ROS 규약)

이 값이 **실제 pick-and-place에서 사용되는 단 하나의 캘리브레이션 결과**.

### 부 결과 (Secondary, 검증용)

Eye-on-Base: `T_ee_board`
- 체커보드가 EE에 얼마나 떨어져/기울어져 부착되었는지
- **캘리브레이션 품질 점검용**이지, 실사용에 쓰이지 않음
- 이유: 체커보드는 캘리브레이션 후 제거되고, 그 자리에 진짜 핸드가 부착됨

### 왜 `T_ee_board`만 평균(`_average_transforms`)하고 `T_base_cam`은 안 하는가

- `T_base_cam`은 `cv2.calibrateHandEye`가 **모든 샘플을 한꺼번에 최적화해서 단일 값**으로 출력. 이미 "종합 결과" — 평균할 대상이 없음
- `T_ee_board`는 방정식 `T_ee_board_i = inv(T_base_ee_i) @ T_base_cam @ T_cam_board_i`로 **샘플마다 N개의 후보**가 나옴. 이상적이면 모두 동일해야 하지만 노이즈 때문에 조금씩 다름. 하나의 대표값으로 합치려면 평균 필요
- 회전 행렬은 `np.mean`으로 element-wise 평균하면 SO(3) 바깥으로 떨어지므로, 쿼터니언으로 변환 후 평균하고 다시 정규화하는 방식을 사용 (`_average_transforms`)

### Residual 검증

같은 `T_ee_board_avg`를 다시 방정식에 대입해 `T_cam_board_computed`를 만들고, 측정된 `T_cam_board`와의 translation 차이를 계산:
```
residual_i = ||T_cam_board[:3,3] - T_cam_board_computed[:3,3]||
```
- 평균 residual < 5 mm: Excellent
- < 20 mm: Moderate
- 그 이상: Poor — 샘플 재수집 권장

---

## 6. 실제 사용: Pick-and-Place 파이프라인

### 전제

캘리브레이션 후:
- 체커보드 제거
- EE에 custom 핸드(그리퍼) 장착
- 핸드의 기하학적 offset `T_ee_gripper`는 **CAD/URDF에서 아는 상수** (캘리브레이션 결과가 아님)
- 비전 파이프라인은 체커보드 대신 **실제 타겟**(FoundationPose, YOLO+RGBD 등)으로 교체

### 필요한 변환들

| 변환 | 출처 | 성격 |
|------|------|------|
| `T_base_cam` | 캘리브레이션 결과 | 고정 상수 |
| `T_base_ee` | 로봇 TF (FK) | 실시간 변함 |
| `T_ee_gripper` | URDF/CAD | 고정 상수 (기계 설계값) |
| `T_cam_object` | 비전 모델 | 실시간 변함 |

### 계산 흐름

1. 비전으로 물체 검출:
   ```
   T_cam_object = FoundationPose(image, depth)
   ```
2. base 좌표로 변환:
   ```
   T_base_object = T_base_cam @ T_cam_object
   ```
3. 그리퍼 tip이 object에 맞도록 목표 ee pose 계산:
   ```
   T_base_ee_goal = T_base_object @ inv(T_ee_gripper)
   ```
4. MoveIt이나 IK 솔버에 `T_base_ee_goal`을 넘기면 로봇이 이동

### 단순 top-down pick의 경우

물체 방향(rotation)은 무시하고 중심 위치만 필요하면:
```python
p_base = (T_base_cam @ T_cam_object)[:3, 3]   # 또는 동등하게
p_base = T_base_cam[:3, :3] @ T_cam_object[:3, 3] + T_base_cam[:3, 3]
```
EE 자세는 "그리퍼가 아래를 향함"으로 고정.

---

## 7. Eye-in-Hand일 때의 차이

카메라가 EE에 붙어 있어서 움직인다:

```
T_base_object = T_base_ee @ T_ee_cam @ T_cam_object
```
- `T_ee_cam`: 캘리브레이션 결과 (고정)
- `T_base_ee`: 실시간 FK
- `T_cam_object`: 실시간 비전

체인에 `T_base_ee`가 추가로 들어가는 것 외에 사고방식은 동일.

Eye-in-Hand가 유리한 경우:
- 작업 영역이 넓어 고정 카메라로는 시야를 다 못 커버할 때
- 물체에 접근하면서 정밀한 close-up 측정이 필요할 때 (visual servoing)

Eye-on-Base가 유리한 경우:
- 작업 영역 전체를 한 번에 내려다보고 싶을 때
- 카메라 케이블/장착 부담이 싫을 때
- 대부분의 산업 pick-and-place에서 이 방식

---

## 8. 캘리브레이션 결과의 실전 검증

### 줄자로 검증 가능한 것
Translation만:
- base 원점에서 카메라 광학 중심까지의 거리 (ZED 2i의 경우 **왼쪽 렌즈 이미저** 위치)
- 결과 (x, y, z)와 ±1~2 cm 이내로 맞아야 정상

### 줄자로 검증 불가능한 것
Rotation은 각도라 자로 잴 수 없음. 대안:
- **RViz2 시각화**: `static_transform_publisher`로 결과를 TF 트리에 등록하고 RViz에서 카메라 프레임 축이 실제 카메라 방향과 맞는지 육안 확인
- **Ground truth 테스트**: base 좌표를 아는 위치에 마커/물체를 두고 비전으로 측정한 값이 그 좌표와 맞는지 비교
- **Residual 수치**: collector가 리포트하는 mm 값이 5 mm 미만이면 내부 일관성 있음

---

## 9. 요약

1. Hand-Eye Calibration은 **카메라와 로봇 base(또는 EE) 사이의 pose**를 구하는 수학적 절차
2. 체커보드는 임시 기준일 뿐, 결과에 남지 않음
3. 주 결과(`T_base_cam` 또는 `T_ee_cam`)만 실사용에 필요하고, 부 결과는 검증용
4. 실제 pick-and-place에서는 **캘리브레이션 결과 + 비전 측정값 + 로봇 FK + 핸드 offset**을 체인 곱으로 엮어 `T_base_ee_goal`을 얻는다
5. 핸드 offset(`T_ee_gripper`)은 캘리브레이션과 무관한 기계 설계값 — URDF/CAD에서 가져옴

### 전체 파이프라인 한눈에

```
[캘리브레이션 단계, 1회]
  체커보드 부착 → 여러 자세로 샘플 수집 → cv2.calibrateHandEye → T_base_cam
  (체커보드 제거, 핸드 부착)

[실시간 운용]
  카메라 영상  → 비전 모델         → T_cam_object
  로봇 TF      → /tf lookup         → T_base_ee
  캘리브레이션 → YAML 로드          → T_base_cam
  URDF/CAD     → TF or 상수         → T_ee_gripper
                     │
                     ▼
  T_base_object  = T_base_cam @ T_cam_object
  T_base_ee_goal = T_base_object @ inv(T_ee_gripper)
                     │
                     ▼
  MoveIt / IK 솔버 → 관절 목표 → 로봇 이동
```
