# Verification Pipeline — 시뮬과 실로봇 양쪽

`verification_node.py`로 캘리브레이션 결과를 검증하는 전체 환경 셋업과 명령어 모음.
시뮬레이션(URSim)과 실로봇(UR5e + 컨트롤러박스) 양쪽을 다룹니다.

---

## 0. 전체 그림

검증은 항상 **3단 파이프라인**입니다.

```
[verification_node]  →  [ur_robot_driver]  →  [URSim 또는 실 UR5e 컨트롤러]
   (ROS 2 노드)        (ROS ↔ TCP 번역기)        (URScript 인터프리터)
```

- `verification_node`: 클릭한 픽셀을 base 좌표로 변환 → URScript 문자열 생성 → ROS 2 토픽으로 publish
- `ur_robot_driver`: ROS 2 토픽 ↔ UR 컨트롤러 TCP socket 번역
- 시뮬에선 URSim 컨테이너가 실 컨트롤러를 가상화. 실로봇이면 진짜 컨트롤러박스.

**전환 핵심: `robot_ip` 인자 하나만 바꾸면 시뮬 ↔ 실로봇.**

---

## 1. 시뮬레이션 셋업 (URSim)

### 1-1. 한 번만 하면 되는 사전 작업

```bash
# UR ROS 2 driver 설치 (ros-humble-ur 메타패키지)
sudo apt update
sudo apt install -y ros-humble-ur

# URSim Docker 이미지 (3GB, 한 번만)
docker pull universalrobots/ursim_e-series:5.18

# 호스트 영구 디렉토리 + 자동 로드용 URCap 복사
mkdir -p ~/ursim/programs ~/ursim/urcaps ~/ursim/dot_polyscope ~/ursim/dot_urcaps
cp /opt/ros/humble/share/ur_robot_driver/resources/externalcontrol-1.0.5.urcap ~/ursim/urcaps/
chmod -R a+rwx ~/ursim
```

> URCap 1.0.5는 PolyScope 5.18까지 호환. 그 이상에선 `NoClassDefFoundError`. 5.18 사용 권장.
> Headless 모드를 쓰면 URCap 자체를 우회하므로 호환성 문제와 무관해진다 (1-3 참고).

### 1-2. URSim 컨테이너 실행

```bash
docker run --rm -d --name ursim \
  -p 5900:5900 \
  -p 6080:6080 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  -v "$HOME/ursim/programs:/ursim/programs" \
  -v "$HOME/ursim/urcaps:/urcaps" \
  -v "$HOME/ursim/dot_polyscope:/ursim/.polyscope" \
  -v "$HOME/ursim/dot_urcaps:/ursim/.urcaps" \
  universalrobots/ursim_e-series:5.18
```

**중요: `--net=host` 쓰지 말 것.** 호스트의 X server abstract socket과 충돌해서 컨테이너 내부 X 부팅 실패 → noVNC가 응답 안 함. 위처럼 명시적 포트 매핑이 안전.

부팅 30초 정도 기다린 뒤 브라우저에서:
```
http://127.0.0.1:6080/vnc.html?host=127.0.0.1&port=6080
```
→ Connect 클릭 → PolyScope 데스크톱.

### 1-3. PolyScope 초기화 (Headless 모드 기준 — 단순)

1. **로봇 ON**: 좌하단 빨간 원 → ON → START → 좌하단 **Normal** (녹색)
2. **Remote Control 켜기**:
   - Settings → System → Remote Control → Enable
   - 우상단 `Local` 클릭 → **Remote** 선택
3. **External Control 프로그램은 만들지 않음.** Headless 모드에서는 driver가 secondary socket(30002)로 직접 URScript를 보내기 때문에 PolyScope에서 ▶ Play 안 눌러도 됨.

> URCap 모드(정식)는 5단계 더 거쳐야 하고 호환성 함정이 있어서, 처음 검증엔 headless가 훨씬 빠르다.
> 우리 코드(`movel`/`stopl`만 보냄)는 두 모드 모두 동일하게 작동.

### 1-4. ur_robot_driver 시작 (호스트에서)

새 터미널:
```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=127.0.0.1 \
  headless_mode:=true \
  launch_rviz:=true \
  tf_prefix:=ur5e_
```

각 인자:
- `ur_type:=ur5e`: **로봇 모델 선택** (URDF, kinematics). prefix 아님. 가능값: `ur3, ur5, ur10, ur3e, ur5e, ur10e, ur16e, ur20, ur30` 등
- `robot_ip:=127.0.0.1`: URSim이 호스트 포트 매핑돼있어 localhost
- `headless_mode:=true`: URCap 우회
- `launch_rviz:=true`: RViz로 가상 UR5e 모델 시각화
- `tf_prefix:=ur5e_`: **TF frame 이름 앞에 `ur5e_` 붙이기**. 현장 PC1이 같은 prefix를 쓰므로 환경 통일 (그래야 verify_node default와 일치)

성공 신호:
```
[INFO] Connected to robot
[INFO] Configuration parameters loaded successfully
[INFO] Starting controllers
```
RViz 창이 뜨고 UR5e 모델이 home 자세로 표시됨.

> `tf_prefix:=ur5e_`를 안 주면 frame 이름이 `base`/`tool0`이 되어 현장 환경과 다르고, 그러면 verify_node 실행 시 `-p base_frame:=base -p ee_frame:=tool0`을 매번 명시해야 한다. **driver launch에 prefix 한 번 붙이는 게 훨씬 단순.**

### 1-5. 토픽 / TF 검증 (또 다른 터미널)

```bash
source /opt/ros/humble/setup.bash

# 핵심 토픽 발행 확인
ros2 topic list | grep -E "tcp_pose|joint_states|urscript"
# 기대:
# /joint_states
# /tcp_pose_broadcaster/pose
# /urscript_interface/script_command

# 현재 EE 위치 (URSim home)
ros2 topic echo /tcp_pose_broadcaster/pose --once
# frame_id가 'ur5e_base'로 출력되어야 함

# TF 이름 확인
ros2 run tf2_ros tf2_echo ur5e_base_link ur5e_tool0
# 정상이면 translation/rotation 출력
```

> `tf_prefix:=ur5e_` 주었을 때 frame 이름은 **`ur5e_base`**, **`ur5e_base_link`**, **`ur5e_tool0`**, **`ur5e_wrist_3_link`** 등.
> verify_node 기본 파라미터(`ur5e_base_link`/`ur5e_tool0`)와 정확히 일치하므로 별도 옵션 추가 불필요.
>
> 만약 driver launch에 `tf_prefix`를 안 줬다면 frame은 `base`/`tool0` (접두사 없음)이고, 그땐 verify_node에 `-p base_frame:=base -p ee_frame:=tool0`을 추가해야 한다.

### 1-6. URScript 송신 sanity check

```bash
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String \
  '{data: "movel(p[0.4,0.0,0.3,3.14,0.0,0.0],a=0.1,v=0.05)"}'
```

기대: RViz의 가상 팔이 5~10초에 걸쳐 천천히 (x=0.4, y=0, z=0.3) top-down 자세로 이동.
이게 작동하면 **driver ↔ URSim 채널은 정상**.

### 1-7. verification_node 실행

ZED ROS 2 wrapper가 **별도 터미널**에서 떠 있어야 한다 (RGB / depth_registered / camera_info 발행).

```bash
source ~/Desktop/Hand_Eye_Calibration/install/setup.bash

ros2 run hand_eye_calibration verify --ros-args \
  -p calibration_file:=/home/kist/Desktop/Hand_Eye_Calibration/<YYYYMMDD_HHMMSS>.yaml
```

driver를 `tf_prefix:=ur5e_`로 띄웠으니 verify_node default(`ur5e_base_link`/`ur5e_tool0`)와 일치 → **frame 옵션 추가 안 해도 됨**.

만약 driver를 prefix 없이 띄웠다면(또는 잘못 띄웠다면):
```bash
ros2 run hand_eye_calibration verify --ros-args \
  -p calibration_file:=...yaml \
  -p base_frame:=base \
  -p ee_frame:=tool0          # ← driver의 실제 frame 이름에 맞춰 명시
```

> URSim에서는 캘리브레이션이 합성 데이터(예: Dec 10 mock)거나 카메라가 base에서 0.85m 이상 떨어졌으면
> URControl이 `Protective Stop (C204A2: Path sanity check failed)` 띄우고 정지한다.
> 이건 우리 코드가 아니라 UR 컨트롤러 안전 검사가 잡는 것 — 정상 동작.
> 풀려면 PolyScope의 Safety Message에서 `Enable Robot` 클릭.
> URSim에서 클릭이 실제 EE 이동까지 가게 하려면 카메라 위치가 base 가까이에 있는 합성 캘리브레이션이 필요.

### 1-8. 종료 / 재기동

```bash
# URSim 정지
docker stop ursim

# 다시 띄울 때 영구 마운트 덕분에 URCap 등록 정보 유지됨
docker run --rm -d --name ursim ... (1-2 와 동일)
```

> PolyScope의 `Restart` 버튼은 컨테이너 PID 1까지 종료시켜 `--rm`으로 컨테이너가 사라진다.
> 영구 마운트(`dot_polyscope`, `dot_urcaps`) 때문에 등록 정보는 보존되니, 다시 `docker run`만 하면 된다.

---

## 2. 실로봇 셋업 (UR5e + 컨트롤러박스)

### 2-1. 물리 연결

```
사용자 PC (PC2)              컨트롤러박스 PC (PC1)              UR5e 컨트롤러박스
   ┌─────────┐                  ┌─────────┐                    ┌─────────┐
   │  ZED    │                  │ ROS 2   │  Ethernet (UR전용)  │  UR5e   │
   │ verify  │ ◄─── LAN ─────►  │ driver  │ ◄────────────────► │ 검정박스 │
   │  node   │   (DDS 토픽)      │         │                    │         │
   └─────────┘                  └─────────┘                    └─────────┘
```

- PC1과 컨트롤러박스 사이: **Ethernet 직결** (또는 같은 로봇 전용 LAN)
- PC1과 PC2: **같은 LAN, 같은 ROS_DOMAIN_ID**
- ZED는 PC2에 USB 직결

> PC1, PC2를 한 컴퓨터로 통합해도 동일하게 작동 — 그땐 모든 노드가 같은 PC에서 돌고 LAN 의존이 없어진다.

### 2-2. 컨트롤러박스 IP 알아내기

Teach pendant에서:
1. 우상단 햄버거(≡) → **Settings → System → Network**
2. **Static Address** 영역의 IP 확인 (예: `192.168.56.101`)
3. Subnet 확인 (예: `255.255.255.0`)

PC1의 NIC가 같은 서브넷에 있어야 한다. 보통 PC1을 `192.168.56.10` 같은 고정 IP로 설정.

```bash
# PC1에서 ping으로 연결 확인
ping <컨트롤러박스 IP>     # 예: ping 192.168.56.101
```

### 2-3. 로봇 캘리브레이션 (한 번만 권장)

UR 공장 출하 캘리브레이션을 PC1에 가져와 정밀도를 높이는 절차. 생략 가능하지만 권장.

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.56.101 \
  target_filename:="${HOME}/my_robot_calibration.yaml"
```

생성된 `my_robot_calibration.yaml`을 driver launch 시 같이 넘긴다.

### 2-4. PolyScope 준비 (Teach pendant)

1. **로봇 Power ON + Brake release** (Normal 녹색)
2. **Network 설정 검증**: PC1 IP 핑 가능 확인
3. **(URCap 모드면)** External Control URCap 설치 + Host IP를 PC1 IP로 + 프로그램 Play
4. **(Headless 모드면)** Remote Control Enable + 우상단 Remote 선택. 프로그램 안 만들어도 됨.

### 2-5. ur_robot_driver 시작 (PC1에서)

```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.56.101 \              # ← 컨트롤러박스 실제 IP
  kinematics_params_file:=${HOME}/my_robot_calibration.yaml \   # 2-3에서 만든 거 (선택)
  headless_mode:=true \
  launch_rviz:=true \
  tf_prefix:=ur5e_                        # ← Hand-Eye Calibration / verify_node default와 일치
```

성공 신호: `Connected to robot`, RViz에 실제 자세로 UR5e 표시.

> 현장 PC1에서 Hand-Eye Calibration을 진행할 때도 동일하게 `tf_prefix:=ur5e_` 인자로 driver를 띄워야 한다 (collector / tf_to_pose 노드가 그 prefix가 붙은 frame을 구독하도록 설정됨).

### 2-6. PC2에서 토픽 발견 확인

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "tcp_pose|joint_states|urscript"
```

PC1이 발행한 토픽들이 PC2에 보여야 한다. 안 보이면:
- ROS_DOMAIN_ID 일치 확인 (`echo $ROS_DOMAIN_ID`)
- ROS_DISTRO 일치 (둘 다 humble)
- 방화벽 (`sudo ufw status` 둘 다 inactive 권장)
- LAN multicast 가능 여부 (회사망은 막힌 경우 있음 → 별도 LAN 권장)

### 2-7. URScript 송신 sanity (실로봇)

PC2 또는 PC1 어느 쪽에서든:

```bash
# 현재 EE 위치 확인
ros2 topic echo /tcp_pose_broadcaster/pose --once

# 짧은 movel 한 번 (현재 위치에서 z만 5cm 위로 이동 같은 안전한 명령 추천)
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String \
  '{data: "movel(p[<x>,<y>,<z+0.05>,<rx>,<ry>,<rz>],a=0.1,v=0.05)"}'
```

**실로봇 첫 명령은 위처럼 현재 위치에서 작은 변화량만 주기.** 갑자기 절대좌표를 박으면 IK가 큰 회전을 풀어내며 예상 못한 자세로 이동할 수 있다.

### 2-8. verification_node 실행 (PC2)

```bash
source ~/Desktop/Hand_Eye_Calibration/install/setup.bash

ros2 run hand_eye_calibration verify --ros-args \
  -p calibration_file:=/home/kist/Desktop/Hand_Eye_Calibration/<calibration_result>.yaml
```

driver를 `tf_prefix:=ur5e_`로 띄웠으니 verify_node default(`ur5e_base_link`/`ur5e_tool0`)와 일치 → frame 옵션 추가 안 해도 됨.

> driver의 tf_prefix와 verify_node의 base_frame/ee_frame 파라미터는 **반드시 일치**해야 한다 (Eye-in-Hand 모드 한정).
> 캘리브레이션 시 사용한 frame 이름과 검증 시 사용하는 frame 이름도 일치해야 의미가 있다.

### 2-9. 안전 절차 (실로봇)

- **Teach pendant E-stop**을 즉시 누를 수 있는 사람이 항상 대기
- 첫 클릭은 책상 위 가까운 안전한 점 (충돌 가능 물체 없는 곳)
- verify_node의 `move_speed=0.05` (5cm/s), `move_accel=0.1`, `stop_decel=2.0` 기본값을 처음엔 더 낮추는 것도 가능
- 클릭 후 **Enter 누르기 전에 콘솔에 출력된 base 좌표를 줄자로 sanity check**
- 이상하면 r 키로 리셋, q로 종료

---

## 3. 시뮬 vs 실로봇 차이 요약

| 항목 | URSim 시뮬 | 실로봇 |
|---|---|---|
| `robot_ip` | `127.0.0.1` (host 매핑) | 컨트롤러박스 실제 IP (`192.168.x.x`) |
| 컨트롤러 SW | URSim Docker (PolyScope+URControl 가상화) | 진짜 컨트롤러박스 |
| Teach pendant | noVNC 브라우저 | 진짜 터치스크린 |
| ROS 2 driver | 같음 | 같음 |
| verify_node | 같음 | 같음 |
| frame 이름 | `ur5e_base_link`, `ur5e_tool0` (driver에 `tf_prefix:=ur5e_` 권장) | `ur5e_base_link`, `ur5e_tool0` (현장 PC1이 같은 prefix 사용) |
| `tf_prefix` 인자 | driver launch에 `tf_prefix:=ur5e_` 추가 | driver launch에 `tf_prefix:=ur5e_` 추가 |
| 안전 위험 | 가상이라 무관 | 실 충돌, 사람 부상 위험 — E-stop 대기 필수 |
| 캘리브레이션 의미 | 합성/모사라 물리적 정합 검증 불가 | 토마토 손끝 거리 등 실측 가능 |

---

## 4. 흔한 에러와 진단

| 증상 | 가능 원인 | 점검 |
|---|---|---|
| `urscript_interface` 토픽 안 보임 | driver 미실행 또는 ROS_DOMAIN_ID 불일치 | PC1/PC2 둘 다 `echo $ROS_DOMAIN_ID` |
| `tf2_echo ur5e_base_link ur5e_tool0` 실패 | driver 미연결 또는 robot Power off | PolyScope/teach pendant 좌하단 Normal 확인 |
| Protective Stop C204A2 | 목표점이 작업영역 밖 | `Enable Robot` 클릭 → 캘리브레이션 또는 reach 파라미터 점검 |
| `frame ur5e_base_link does not exist` (Eye-in-Hand) | driver를 `tf_prefix:=ur5e_` 없이 띄움 | driver launch에 `tf_prefix:=ur5e_` 추가하거나, verify_node에 `-p base_frame:=base -p ee_frame:=tool0` 명시 |
| `NoClassDefFoundError` (URCap 모드) | URCap 1.0.5 ↔ PolyScope 새 버전 비호환 | URSim 5.18 이하 사용 또는 headless 모드로 우회 |
| Insufficient depth 0/9 (verify) | ZED depth 노이즈 (무늬 없는 표면) | 무늬/광원 양호한 표면 클릭 |
| EE가 엉뚱한 곳 1m+ 어긋남 | frame 혼동 (optical vs ROS) 또는 캘리브레이션 자체 오류 | YAML의 T_base_cam translation을 줄자로 sanity check |
| 클릭 후 Enter 눌렀는데 무반응 | Remote Control 모드 아님 또는 driver 끊김 | PolyScope 우상단 Remote 확인, driver 콘솔 확인 |

---

## 5. 빠른 참조 (시뮬 1회 검증, 명령 모음)

```bash
# 1. URSim 시작
docker run --rm -d --name ursim -p 5900:5900 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 \
  -v "$HOME/ursim/programs:/ursim/programs" -v "$HOME/ursim/urcaps:/urcaps" \
  -v "$HOME/ursim/dot_polyscope:/ursim/.polyscope" -v "$HOME/ursim/dot_urcaps:/ursim/.urcaps" \
  universalrobots/ursim_e-series:5.18

# 2. 브라우저: http://127.0.0.1:6080/vnc.html?host=127.0.0.1&port=6080
#    → Power ON + Remote 모드

# 3. driver (터미널 A) — tf_prefix:=ur5e_ 필수 (현장 환경과 통일)
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e robot_ip:=127.0.0.1 headless_mode:=true launch_rviz:=true \
  tf_prefix:=ur5e_

# 4. ZED wrapper (터미널 B) — 별도 ZED 워크스페이스에서
source ~/zed_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

# 5. verify (터미널 C) — frame 옵션 불필요 (default가 driver와 일치)
source ~/Desktop/Hand_Eye_Calibration/install/setup.bash
ros2 run hand_eye_calibration verify --ros-args \
  -p calibration_file:=<YAML 경로>
```

---

## 6. 다음 날 빠른 재시작 (시뮬, headless 모드)

### 터미널 1 — URSim Docker 시작 (한 번만)

```bash
docker run --rm -d --name ursim \
  -p 5900:5900 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 \
  -v "$HOME/ursim/programs:/ursim/programs" \
  -v "$HOME/ursim/urcaps:/urcaps" \
  -v "$HOME/ursim/dot_polyscope:/ursim/.polyscope" \
  -v "$HOME/ursim/dot_urcaps:/ursim/.urcaps" \
  universalrobots/ursim_e-series:5.18
```

부팅 확인 (30~60초):
```bash
nc -zv 127.0.0.1 5900 && nc -zv 127.0.0.1 30002
# 둘 다 succeeded 나오면 OK
```

### 브라우저 — PolyScope 접속

```
http://127.0.0.1:6080/vnc.html?host=127.0.0.1&port=6080
```
→ Connect 클릭 → PolyScope 데스크톱

### PolyScope 안에서 (마우스 조작)

1. **로봇 ON**: 좌하단 빨간 원 클릭 → ON → START → 좌하단 Normal(녹색) → Exit
2. **Remote 모드**: 우상단 `Local` 클릭 → **Remote** 선택
3. (External Control 프로그램 안 만듦 — headless라 불필요)

### 터미널 2 — UR ROS 2 driver

```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=127.0.0.1 \
  headless_mode:=true \
  launch_rviz:=true \
  tf_prefix:=ur5e_
```

> **`tf_prefix:=ur5e_` 빠지면 verify_node frame 안 맞아서 에러난다.**

성공 신호: `[INFO] Connected to robot`, RViz에 UR5e 모델.

### 터미널 3 (선택) — URScript sanity

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String \
  '{data: "movel(p[0.4,0.0,0.3,3.14,0.0,0.0],a=0.1,v=0.05)"}'
```
→ RViz의 가상 팔이 천천히 움직이면 driver↔URSim 채널 OK.

### 터미널 4 — ZED ROS 2 wrapper

```bash
source ~/zed_ws/install/setup.bash    # 본인 ZED 워크스페이스 경로
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

### 터미널 5 — verification_node

```bash
cd ~/Desktop/Hand_Eye_Calibration
source install/setup.bash

ros2 run hand_eye_calibration verify --ros-args \
  -p calibration_file:=$(pwd)/calibration_result_20251210_112241.yaml \
  -p reach_max:=3.85
```

옵션:
- `calibration_file`: 검증할 YAML (Eye-on-Base인 Dec 10 파일 권장 — 가장 단순)
- `reach_max:=3.85`: Dec 10 합성 캘리브레이션은 카메라가 base에서 1m+ 떨어진 위치로 잡혀있어 클릭 좌표가 0.85m 넘으니 임시로 풀어둠. URControl이 작업영역 밖이면 Protective Stop 발생 (정상)
- frame 옵션 안 줘도 됨 (default `ur5e_base_link`/`ur5e_tool0`이 driver tf_prefix와 일치)

GUI 윈도우 뜨면 클릭 → Enter → URSim 가상 팔 움직임 또는 Protective Stop 관찰.

### 종료

각 터미널 Ctrl+C, 마지막:
```bash
docker stop ursim
```

### 한 페이지 요약 (복사용)

```bash
# 1. URSim
docker run --rm -d --name ursim -p 5900:5900 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 \
  -v "$HOME/ursim/programs:/ursim/programs" -v "$HOME/ursim/urcaps:/urcaps" \
  -v "$HOME/ursim/dot_polyscope:/ursim/.polyscope" -v "$HOME/ursim/dot_urcaps:/ursim/.urcaps" \
  universalrobots/ursim_e-series:5.18

# [브라우저] http://127.0.0.1:6080/vnc.html?host=127.0.0.1&port=6080
# [PolyScope] Power ON + START → Remote 모드

# 2. driver
source /opt/ros/humble/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e robot_ip:=127.0.0.1 headless_mode:=true launch_rviz:=true tf_prefix:=ur5e_

# 3. ZED
source ~/zed_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

# 4. verify
cd ~/Desktop/Hand_Eye_Calibration && source install/setup.bash
ros2 run hand_eye_calibration verify --ros-args \
  -p calibration_file:=$(pwd)/calibration_result_20251210_112241.yaml \
  -p reach_max:=3.85

# 종료: 각 터미널 Ctrl+C → 마지막에 `docker stop ursim`
```
