# Jetson nano ROS1 시작

1. ROS Noetic 설치
```
# 필수 툴
sudo apt update
sudo apt install -y curl gnupg2 lsb-release

# ROS 저장소 키 추가
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
> /etc/apt/sources.list.d/ros-latest.list'
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
| sudo apt-key add -

# 설치
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# 환경설정
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# rosdep 초기화
sudo rosdep init
rosdep update
```

2. TurtleBot3 ROS 패키지 설치
```
sudo apt install -y \
  ros-noetic-dynamixel-sdk \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3 \
  ros-noetic-turtlebot3-bringup \
  ros-noetic-turtlebot3-slam \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-teleop-twist-keyboard
```
환경변수 추가:
```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

3. OpenCR 펌웨어 업로드
```
# armhf 지원 추가 (업데이트 유틸리티 실행용)
sudo dpkg --add-architecture armhf
sudo apt update
sudo apt install -y libc6:armhf

# 펌웨어 스크립트 다운로드
cd ~
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd opencr_update

# 포트/모델 지정 후 업로드 (burger 기준)
export OPENCR_PORT=/dev/ttyACM0
./update.sh $OPENCR_PORT burger_noetic.opencr
```

4. Bringup 실행
```
# 로봇 하드웨어 기동
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

5. Teleop (키보드 조종)
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
