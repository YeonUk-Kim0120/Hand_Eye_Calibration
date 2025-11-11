# WSL2 Setup Guide for Hand-Eye Calibration

## Windows에서 실행 (PowerShell 관리자 권한)

```powershell
# 1. WSL2 설치
wsl --install -d Ubuntu-22.04
wsl --update

# 2. USBIPD 설치 (USB 장치 지원)
winget install --interactive --exact dorssel.usbipd-win

# 3. ZED 카메라 연결 (WSL 실행 후)
# 먼저 장치 확인
usbipd list

# ZED 카메라의 busid 확인 후 (예: 2-4)
usbipd bind --busid 2-4
usbipd attach --wsl --busid 2-4
```

## WSL2 Ubuntu에서 실행

```bash
# 1. 기본 설정
sudo apt update && sudo apt upgrade -y

# 2. ROS 2 Humble 설치
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools -y

# 3. bashrc 설정
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# 4. Python 의존성
pip install numpy==1.26.4 tf-transformations opencv-python pyyaml

# 5. ROS 2 패키지
sudo apt install ros-humble-message-filters -y

# 6. ZED SDK (선택사항, 실제 ZED 사용시)
wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22 -O zed_sdk.run
chmod +x zed_sdk.run
./zed_sdk.run

# ZED ROS 2 Wrapper
sudo apt install ros-humble-zed-wrapper -y

# 7. USB 장치 권한
sudo usermod -aG video $USER

# 8. 프로젝트 클론
cd ~
git clone https://github.com/YeonUk-Kim0120/Hand_Eye_Calibration.git hand_eye_ws
cd hand_eye_ws

# 9. 빌드
colcon build
source install/setup.bash
```

## 매번 ZED 카메라 연결 (Windows PowerShell)

```powershell
# WSL이 실행 중일 때
usbipd list
usbipd attach --wsl --busid 2-4  # busid는 실제 값으로 변경
```

## 카메라 확인 (WSL)

```bash
# 카메라 장치 확인
lsusb | grep -i stereo
ls /dev/video*

# ZED 카메라 테스트
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

## 문제 해결

### GUI가 안 보일 때
```bash
echo "export DISPLAY=:0" >> ~/.bashrc
source ~/.bashrc
```

### USB 장치 권한 오류
```bash
sudo chmod 666 /dev/video*
```

### ROS 2 통신 안됨
```bash
# ~/.bashrc에 추가
export ROS_LOCALHOST_ONLY=1
```

## 성능 최적화

### WSL2 메모리 제한 설정
Windows 사용자 폴더에 `.wslconfig` 파일 생성:

```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
```

저장 후:
```powershell
wsl --shutdown
wsl
```
