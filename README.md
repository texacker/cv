# Computational Vision

## Edge-based CV

## Texture & Pattern Segmentation

## 嵌入式系统开发
### CAN Utilities
```bash
sudo debfoster build-essential
sudo debfoster libusb-1.0-0
sudo debfoster libeigen3-dev

# SocketCAN
sudo debfoster libsocketcan-dev
sudo debfoster can-utils
```

### 串口
```bash
sudo debfoster microcom
microcom -s 115200 -p /dev/ttyUSB0

sudo debfoster minicom
sudo minicom -s
minicom -c on
```

### 3D Graphics
```bash
# nVidia Driver:
#   - https://wiki.debian.org/NvidiaGraphicsDrivers#stretch
lspci -nn | grep -i nvidia
lsmod | grep -i nouveau
sudo cp /etc/apt/sources.list /etc/apt/sources.list.orig
sudo sed -i 's/stretch main$/stretch main contrib non-free/' /etc/apt/sources.list
sudo apt update && sudo apt upgrade
sudo debfoster linux-headers-$(uname -r | sed 's/[^-]*-[^-]*-//')
sudo debfoster nvidia-driver
ll /etc/modprobe.d/
sudo reboot

# Backing out in case of failure:
#   - https://wiki.debian.org/NvidiaGraphicsDrivers/#Backing_out_in_case_of_failure
sudo apt-get purge nvidia.    # don't forget the "." dot, it erases every package with "nvidia" on its name.
sudo apt-get install --reinstall xserver-xorg
sudo apt-get install --reinstall xserver-xorg-video-nouveau
sudo reboot

# OpenGL:
sudo debfoster libglew-dev                  # OpenGL Loading Library
sudo debfoster freeglut3-dev libglfw3-dev   # API for windowing sub-systems(GLX, WGL, CGL ...)
sudo debfoster libglm-dev libassimp-dev libmagick++-dev libsoil-dev

# OpenSceneGraph:
sudo debfoster openscenegraph-3.4 libopenscenegraph-3.4-dev openscenegraph-3.4-doc openscenegraph-3.4-examples

# OGRE:
sudo debfoster libogre-1.9-dev ogre-1.9-tools blender-ogrexml-1.9 libois-dev
```

### debian 下交叉编译 ZeroMQ for ARM (i.MX6UL)
```bash
# 一：安装交叉编译工具

# apt 安装，最新版
sudo debfoster g++-arm-linux-gnueabihf

# 手动安装，可以选择旧版本
# 因为硬件环境下的 stdc++ 等的版本一般滞后，太新的交叉编译器编译出来的，拷贝过去不能运行

# zlg.cn 的 M6G2C 系统目前（2018-09-12）只支持 Latest 4
# 更新版本的交叉编译工具编译出来的可执行程序会输出类似错误：
# /usr/lib/arm-linux-gnueabihf/libstdc++.so.6: version `CXXABI_1.3.9' not found (required by /opt/usr/local/lib/libzmq.so.5)
# /usr/lib/arm-linux-gnueabihf/libstdc++.so.6: version `GLIBCXX_3.4.21' not found (required by /opt/usr/local/lib/libzmq.so.5)

wget https://releases.linaro.org/components/toolchain/binaries/latest-4/arm-linux-gnueabihf/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf.tar.xz
tar Jxvf -C ~/.local gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf.tar.xz
export PATH=~/.local/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf/bin:$PATH

# 二：编译 libzmq
wget https://github.com/zeromq/libzmq/releases/download/v4.2.3/zeromq-4.2.3.tar.gz
tar zxvf zeromq-4.2.3.tar.gz
cd zeromq-4.2.3
./configure --help
./configure --prefix=`pwd`/dist-build/ --host=arm-linux-gnueabihf
make
make check
make install
make clean
make distclean
```

### OpenCV
```bash
# Prosilica/Aravis GigE API:
# https://wiki.gnome.org/action/show/Projects/Aravis?action=show&redirect=Aravis
# https://github.com/AravisProject/aravis
# https://www.alliedvision.com/en/support/software-downloads.html

# OpenCV:
# https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html

sudo debfoster build-essential                                                                                  # compiler
sudo debfoster cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev                 # required
sudo debfoster python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff5-dev libdc1394-22-dev  # optional
sudo debfoster libjasper-dev                                                                                    # sid (unstable) ?

mkdir -p <your_opencv_ws> && cd <your_opencv_ws>

git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
git clone https://github.com/opencv/opencv_extra.git

mkdir build && cd build && \
env PVAPI_ROOT="$HOME/.opt/PvAPI_1.28_Linux/AVT_GigE_SDK" cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local -DBUILD_EXAMPLES=ON -DINSTALL_C_EXAMPLES=ON -DBUILD_TESTS=ON -DINSTALL_TESTS=ON -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules -DWITH_PVAPI=ON ../opencv && \
make -j && \
env OPENCV_TEST_DATA_PATH=../opencv_extra/testdata/ ./bin/opencv_test_core && \
make install clean

# Note: if you don't want it any more:
# make uninstall
```

### Fiducial Markers
#### ArUco
```bash
# http://www.uco.es/investiga/grupos/ava/node/25
# https://sourceforge.net/projects/aruco/

sudo debfoster libeigen3-dev

mkdir -p <your_aruco_ws> && cd <your_aruco_ws>
# Download ArUco from : https://sourceforge.net/projects/aruco/files/latest/download
unzip aruco-3.0.13.zip

mkdir build && cd build && \
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$HOME/.local -DCMAKE_INSTALL_PREFIX=$HOME/.local -DUSE_OWN_EIGEN3=OFF ../aruco-3.0.13
make -j
make install
make clean

# Calibrate Camera :
env LD_LIBRARY_PATH=~/.local/lib/ ~/.local/bin/aruco_calibration_fromimages LUMIX-LX3.yml . -size 0.035

# Print marker :
env LD_LIBRARY_PATH=~/.local/lib/ ~/.local/bin/aruco_print_marker 70 ./ARUCO_MIP_36h12_00070.png -e -bs 300

# Print Customized Dictionary :
env LD_LIBRARY_PATH=~/.local/lib/ ~/.local/bin/aruco_print_dictionary <path_to_save_all_images> <path_to/my_own.dict>

# Detect Marker :
env LD_LIBRARY_PATH=~/.local/lib/ ~/.local/bin/aruco_simple ./P1010976.png -c ./calibration/png/LUMIX-LX3.yml -s 0.166

# Note: OpenCV4/aruco 与 aruco-3.0.13 版本差异见：opencv2/aruco.hpp

# Note: if you don't want it any more:
# make uninstall
```

#### MarkerMapper
```bash
# http://www.uco.es/investiga/grupos/ava/node/25
# https://sourceforge.net/projects/markermapper/

mkdir -p <your_markermapper_ws> && cd <your_markermapper_ws>
# Download MarkerMapper from : https://sourceforge.net/projects/markermapper/files/latest/download
unzip marker_mapper1.0.12.zip

mkdir build && cd build && \
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$HOME/.local -DCMAKE_INSTALL_PREFIX=$HOME/.local -DUSE_OWN_EIGEN3=OFF ../marker_mapper1.0.12
make -j
make install
make clean

# Note: if you don't want it any more:
# make uninstall
```

### ROS
```bash
# http://wiki.ros.org/noetic/Installation/Debian
# http://wiki.ros.org/noetic/Installation/Source

sudo debfoster chrony ntpdate

# update system time through ntp client(chrony)
sudo systemctl status chronyd.service
chronyc sources
chronyc sourcestats

# or an one-time update
sudo ntpdate [ -q ] ntp.ubuntu.com
sudo ntpdate [ -q ] ntp.tuna.tsinghua.edu.cn

sudo debfoster dirmngr
sudo debfoster libpcl-dev

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update && sudo apt upgrade

# 1. Prerequisites

# 1.1 Installing bootstrap dependencies

# Ubuntu
sudo debfoster python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Generic (pip)
sudo debfoster python-pip
sudo pip install -U rosdep rosinstall rosinstall_generator wstool

# 1.2 Initializing rosdep [in case of GFWed :-)]
sudo [ env https_proxy=http://host:port ] rosdep init
[ env https_proxy=http://host:port ] rosdep update

# 2. Installation
sudo debfoster ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.profile

# Initializing catkin Workspace
mkdir -p <your_catkin_ws>/src
cw && ( cd ./src && catkin_init_workspace ) && catkin_make && source ./devel/setup.bash
echo 'export CATKIN_WS_PATH="<your_catkin_ws>"' >> ~/.profile
echo "source $CATKIN_WS_PATH/devel/setup.bash" >> ~/.profile

# Security issue on ROS build farm :
# 1. http://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/
# 2. http://wiki.ros.org/melodic/Installation/Ubuntu
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# apt update: signatures were invalid: F42ED6FBAB17C654
# https://answers.ros.org/question/379190/apt-update-signatures-were-invalid-f42ed6fbab17c654/
sudo apt-key del C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### PCL
```bash
# http://www.pointclouds.org/documentation/
sudo debfoster dirmngr libpcl-dev pcl-tools
```

### CGAL
```bash
# https://www.cgal.org/download/linux.html
sudo debfoster libmpfi-dev libmetis-dev libntl-dev libqt5svg5 ipe

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local -DWITH_examples=true -DWITH_demos=true ../CGAL-4.14
make
make demos
make examples
make install clean

# or through apt:
sudo debfoster libcgal-dev libcgal-demo libcgal-qt5-dev libcgal-ipelets

# Note: if you don't want it any more:
# see: cgal.uninstall.log
```

### laser_scan_matcher
```bash
# https://answers.ros.org/question/211789/localization-based-on-a-laserscan/
sudo debfoster libgsl-dev

mkdir -p <your_csm_ws> && cd <your_csm_ws>

git clone https://github.com/AndreaCensi/csm

mkdir build && cd build && \
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$HOME/.local -DCMAKE_INSTALL_PREFIX=$HOME/.local ../csm && \
make && \
make install | tee ../install.log && \
make clean

( cs && git clone https://github.com/ccny-ros-pkg/scan_tools.git ) && cm

# Note: if you don't want it any more:
# see: csm.uninstall.log
```

### Optimization Problems Solver
```bash
sudo debfoster libceres1 ceres-solver-doc
```
