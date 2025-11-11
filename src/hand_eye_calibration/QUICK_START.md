# 빠른 시작 가이드

## ⚡ 5분 안에 시작하기

### 1. 빌드 (최초 1회만)
```bash
cd ~/Desktop/hand_eye_ws
colcon build --packages-select hand_eye_calibration --symlink-install
source install/setup.bash
```

### 2. 실행
```bash
# 터미널 1: 모든 노드 실행
ros2 launch hand_eye_calibration hand_eye_prep.launch.py
```

```bash
# 터미널 2: 자동 샘플 수집 (키보드 제어)
source ~/Desktop/hand_eye_ws/install/setup.bash
python3 ~/Desktop/hand_eye_ws/src/hand_eye_calibration/scripts/auto_capture.py
```

### 3. 키보드 명령
- **Enter** - 다음 단계 진행
- **s** - 건너뛰기
- **q** - 종료
- **c** - 캘리브레이션 실행

### 4. 결과 확인
- 터미널 1에서 캘리브레이션 결과 확인
- `calibration_result_YYYYMMDD_HHMMSS.yaml` 파일 생성됨
- 샘플은 `calibration_samples.npz`에 자동 저장

---

## 🎯 핵심 개선사항 (방금 추가된 기능)

### ✅ 1. 키보드 제어 자동화 스크립트
- **이전**: 타이머 기반 자동 실행 (위험)
- **현재**: 사용자가 Enter 키로 각 단계 확인
- **장점**: 안전하고 유연한 제어

### ✅ 2. 실시간 시각화
- 체커보드 검출 시각화 (코너, 좌표축)
- 거리 및 포즈 정보 표시
- 검출 실패 시 메시지 표시

### ✅ 3. 샘플 영구 저장
- NPZ 형식으로 디스크에 자동 저장
- 노드 재시작 시 자동 로드
- 중단-재개 워크플로우 지원

### ✅ 4. 캘리브레이션 결과 저장
- YAML 형식으로 결과 저장
- 타임스탬프 포함
- 변환 행렬, 회전/이동 벡터, 쿼터니언 모두 포함

### ✅ 5. YAML 기반 포즈 관리
- `config/poses.yaml`에서 포즈 목록 관리
- 쉽게 수정/추가 가능
- 버전 관리 가능

---

## 📁 생성된 파일들

```
hand_eye_calibration/
├── config/
│   ├── checkerboard_params.yaml   (기존)
│   └── poses.yaml                 (신규 ⭐)
├── scripts/
│   └── auto_capture.py            (신규 ⭐)
├── hand_eye_calibration/
│   ├── checkerboard_detector_node.py  (시각화 추가 ⭐)
│   ├── calibration_collector_node.py  (저장/불러오기 추가 ⭐)
│   └── mock_robot_publisher_node.py   (기존)
├── launch/
│   └── hand_eye_prep.launch.py    (ZED 경로 수정 ⭐)
├── setup.py                       (scripts 추가 ⭐)
├── USAGE_GUIDE.md                 (신규 ⭐)
└── QUICK_START.md                 (이 파일 ⭐)
```

---

## 🔍 파일별 역할 요약

| 파일 | 역할 | 주요 기능 |
|------|------|-----------|
| `checkerboard_detector_node.py` | 체커보드 검출 | OpenCV 검출, 포즈 추정, 시각화 |
| `calibration_collector_node.py` | 샘플 수집/캘리브레이션 | 샘플 저장/로드, Hand-Eye 계산 |
| `mock_robot_publisher_node.py` | 가상 로봇 | 파라미터로 포즈 발행 |
| `auto_capture.py` | 자동화 스크립트 | YAML 읽기, 키보드 제어 |
| `poses.yaml` | 포즈 설정 | 로봇 포즈 목록 정의 |
| `hand_eye_prep.launch.py` | 통합 실행 | 모든 노드 한 번에 시작 |
| `setup.py` | 빌드 설정 | 노드/스크립트 등록, 의존성 |
| `__init__.py` | 패키지 인식 | Python 패키지로 인식 (비어있음) |

---

## 🚨 자주 묻는 질문

### Q1: 샘플을 저장하면 어디에 저장되나요?
**A**: 현재 작업 디렉토리에 `calibration_samples.npz` 파일로 저장됩니다.

### Q2: 샘플을 초기화하려면?
**A**: `rm calibration_samples.npz` 명령으로 파일을 삭제하세요.

### Q3: 중간에 종료하고 나중에 이어서 샘플을 수집할 수 있나요?
**A**: 네! 자동으로 기존 샘플을 로드하여 이어서 수집합니다.

### Q4: 실제 로봇 팔과 연동하려면?
**A**: `mock_robot_publisher_node.py` 대신 실제 로봇의 FK를 `/base_to_end_effector` 토픽으로 발행하는 노드를 만들면 됩니다.

### Q5: 체커보드 크기를 어떻게 측정하나요?
**A**: 자로 여러 칸(예: 5칸)을 재고 칸 수로 나누어 평균값을 사용하세요.

---

## 💡 프로 팁

1. **포즈 다양성**: 작업 영역 전체에 골고루 샘플 수집
2. **샘플 개수**: 10~15개가 적당, 20개 이상이면 더 정확
3. **체커보드 각도**: 정면, 왼쪽, 오른쪽, 위, 아래 등 다양하게
4. **검증**: 캘리브레이션 여러 번 실행하여 결과 일관성 확인

---

**더 자세한 내용은 `USAGE_GUIDE.md`를 참고하세요!**
