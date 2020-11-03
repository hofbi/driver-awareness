#!/usr/bin/env bash

TURBO_JPEG='1.5.1'
PUPIL_RELEASE='v2.5'
PUPIL_VERSION="${PUPIL_RELEASE}-0-g70e93bc"

sudo apt update && sudo apt install -y apt-utils

echo 'Install Pupil Labs'
# Pupil dependencies
sudo apt install -y pkg-config git cmake build-essential nasm wget python3-setuptools libusb-1.0-0-dev python3-dev python3-pip python3-numpy python3-scipy libglew-dev libtbb-dev

# ffmpeg >= 3.2
sudo apt install -y libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libavresample-dev ffmpeg x264 x265 libportaudio2 portaudio19-dev

# OpenCV >= 3
sudo apt install -y python3-opencv libopencv-dev

# 3D Eye model dependencies
sudo apt install -y libgoogle-glog-dev libatlas-base-dev libeigen3-dev
sudo apt install -y libceres-dev

# glfw
sudo apt install -y xorg-dev
git clone https://github.com/glfw/glfw
pushd glfw || exit
git checkout 3.3.2
mkdir build
pushd build || exit
cmake -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
sudo ldconfig
popd || exit
popd || exit

# Turbojpeg
wget -O libjpeg-turbo.tar.gz https://sourceforge.net/projects/libjpeg-turbo/files/${TURBO_JPEG}/libjpeg-turbo-${TURBO_JPEG}.tar.gz/download
tar xvzf libjpeg-turbo.tar.gz
pushd libjpeg-turbo-${TURBO_JPEG} || exit
./configure --enable-static=no --prefix=/usr/local
sudo make install
sudo ldconfig
popd || exit
sudo rm -rf libjpeg-turbo-${TURBO_JPEG} libjpeg-turbo.tar.gz

# Custom Version of libusb for 200hz cameras
wget https://github.com/pupil-labs/libusb/releases/download/v1.0.21-rc6-fixes/libusb-1.0.so.0
sudo dpkg -L libusb-1.0-0-dev | grep libusb-1.0.so
rm -rf libusb-1.0.so.0

# Libuvc
git clone https://github.com/pupil-labs/libuvc
pushd libuvc || exit
mkdir build
pushd build || exit
cmake ..
make && sudo make install
popd || exit
popd || exit
rm -rf libuvc

# Run libuvc as normal user
echo 'SUBSYSTEM=="usb",  ENV{DEVTYPE}=="usb_device", GROUP="plugdev", MODE="0664"' | sudo tee /etc/udev/rules.d/10-libuvc.rules >/dev/null
sudo udevadm trigger

# Install Pupil Release
wget https://github.com/pupil-labs/pupil/releases/download/${PUPIL_RELEASE}/pupil_${PUPIL_VERSION}_linux_x64.zip
unzip pupil_${PUPIL_VERSION}_linux_x64.zip
sudo apt install -y ./pupil_${PUPIL_VERSION}_linux_x64/pupil_capture_linux_os_x64_${PUPIL_VERSION}.deb
sudo apt install -y ./pupil_${PUPIL_VERSION}_linux_x64/pupil_player_linux_os_x64_${PUPIL_VERSION}.deb
sudo apt install -y ./pupil_${PUPIL_VERSION}_linux_x64/pupil_service_linux_os_x64_${PUPIL_VERSION}.deb
rm -rf pupil_${PUPIL_VERSION}_linux_x64*

# Clone pupil repo
git clone https://github.com/pupil-labs/pupil.git ~/dev/pupil
python3 ~/dev/pupil/pupil_src/main.py capture --help

# Install requirements
python3 -m pip install --upgrade pip

pip3 install cysignals
pip3 install cython
pip3 install msgpack==0.5.6
pip3 install numexpr
pip3 install packaging
pip3 install psutil
pip3 install pyaudio
pip3 install pyopengl
pip3 install pyzmq
pip3 install scikit-learn
pip3 install scipy
pip3 install git+https://github.com/zeromq/pyre

pip3 install pupil-apriltags
pip3 install pupil-detectors
pip3 install git+https://github.com/pupil-labs/PyAV
pip3 install git+https://github.com/pupil-labs/pyuvc
pip3 install git+https://github.com/pupil-labs/pyndsi
pip3 install git+https://github.com/pupil-labs/pyglui

# Our requirements
sudo apt install -y libsdl2-dev ros-noetic-opencv-apps ros-noetic-derived-object-msgs
