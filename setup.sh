#!/bin/bash

# exit when any command fails
set -e

bar () {
    # prints a bar equal to the current terminal width
    printf '=%.0s' $(seq 1 "$(tput cols)")
}

# colors
RED='\033[0;31m'
LIGHTRED='\033[1;31m'
GREEN='\033[0;32m'
LIGHTGREEN='\033[1;32m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

VRC_DIR=~/Documents/vrc
VENV_DIR=~/Documents/vrc/.venv

REALSENSE_DIR=~/Documents/librealsense
LIBREALSENSE_VERSION=v2.31.0 # https://github.com/JetsonHacksNano/installLibrealsense/blob/master/buildLibrealsense.sh#L9

# see if sudo is installed
# mainly for testing with Docker, that doesn't have sudo
s=""
if [ -n "$(which sudo)" ]; then
    s="sudo"
fi

# check to make sure code has already been cloned
if [[ ! -d  $VRC_DIR ]]; then
    echo "VRC repository has not been cloned to $VRC_DIR"
    echo "Do this with '$s apt update && $s apt install -y git && git clone https://github.com/bellflight/VRC-2021-Phase-II vrc $VRC_DIR --recurse-submodules'"
    exit 1
fi

echo -e "${RED}"
echo "██████████████████████████████████████████████████████████████████████████"
echo -e "█████████████████████████████████████████████████████████████████████${NC}TM${RED}███"
echo "███████▌              ▀████            ████     ██████████     ███████████"
echo "█████████▄▄▄  ▄▄▄▄     ▐███    ▄▄▄▄▄▄▄▄████     ██████████     ███████████"
echo "██████████▀   █████    ████    ▀▀▀▀▀▀▀▀████     ██████████     ███████████"
echo "██████████            ▀████            ████     ██████████     ███████████"
echo "██████████    ▄▄▄▄▄     ███    ████████████     ██████████     ███████████"
echo "██████████    ████▀     ███    ▀▀▀▀▀▀▀▀████     ▀▀▀▀▀▀▀███     ▀▀▀▀▀▀▀▀███"
echo "██████████             ▄███            ████            ███             ███"
echo "██████████▄▄▄▄▄▄▄▄▄▄▄██████▄▄▄▄▄▄▄▄▄▄▄▄████▄▄▄▄▄▄▄▄▄▄▄▄███▄▄▄▄▄▄▄▄▄▄▄▄▄███"
echo "██████████████████████████████████████████████████████████████████████████"
echo "                                                                          "
echo "██████████████████████████████▄▄          ▄▄██████████████████████████████"
echo "██████████████████████████████████▄    ▄██████████████████████████████████"
echo "████████████████████████████████████  ████████████████████████████████████"
echo "███▀▀▀▀▀██████████████████████████▀    ▀██████████████████████████▀▀▀▀▀███"
echo "████▄▄          ▀▀▀▀█████████████        █████████████▀▀▀▀          ▄▄████"
echo "████████▄▄▄                ▀▀▀▀▀██████████▀▀▀▀▀                ▄▄▄████████"
echo "█████████████▄▄                   ▀████▀                   ▄▄█████████████"
echo "█████████████████▄                  ██                  ▄█████████████████"
echo "██████████████████████████████▀     ██     ▀██████████████████████████████"
echo "███████████████████████▀▀           ██           ▀▀███████████████████████"
echo "████████████████▀▀▀                 ██                 ▀▀▀████████████████"
echo "█████████▀▀                       ▄████▄                       ▀▀█████████"
echo "████▀▀                         ▄███▀  ▀███▄                         ▀▀████"
echo " ████▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄█████▀      ▀█████▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄████ "
echo " ▀███████████████████████████████▄      ▄███████████████████████████████▀ "
echo "  ▀████████████████████████████████    ████████████████████████████████▀  "
echo "    ██████████████████████████████▀    ▀██████████████████████████████    "
echo "     ▀████████████████████████████▄    ▄████████████████████████████▀     "
echo "       ▀███████████████████████████    ███████████████████████████▀       "
echo "         ▀█████████████████████████    █████████████████████████▀         "
echo "           ▀███████████████████████    ███████████████████████▀           "
echo "             ▀█████████████████████    █████████████████████▀             "
echo "               ▀███████████████████    ███████████████████▀               "
echo "                 ▀█████████████████    █████████████████▀                 "
echo "                    ▀██████████████    ██████████████▀                    "
echo "                      ▀████████████    ████████████▀                      "
echo "                        ▀██████████    ██████████▀                        "
echo "                           ▀███████    ███████▀                           "
echo "                             ▀▀████    ████▀▀                             "
echo "                                ▀███  ███▀                                "
echo "                                  ▀█▄▄█▀                                  "
echo -e "${NC}"

bar


echo -e "${CYAN}Updating package index${NC}"
bar
$s apt update
bar

echo -e "${CYAN}Updating system packages${NC}"
bar
# upgrade existing packages
$s apt upgrade -y
# autoremove a bunch
$s apt autoremove -y
bar


echo -e "${CYAN}Installing prerequisites${NC}"
bar
# install some useful prereqs
$s apt install git make apt-utils software-properties-common wget unzip htop -y
cd $VRC_DIR
# cache the git credentials (mainly during development)
git config --global credential.helper cache
# update repo
git pull
# switch to main branch
git checkout main
bar


echo -e "${CYAN}Installing Python 3.8${NC}"
bar
# Add deadsnakes PPA to sources
echo | $s add-apt-repository ppa:deadsnakes/ppa
# Install Python 3.8
$s apt install -y python3.8 python3.8-dev python3.8-venv python3-pip python3-dev

# add Python 3.8 as a python3 alternative
# $s update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1
# set it as the default
# $s update-alternatives --set python /usr/bin/python3.8

# upgrade system pip
python3 -m pip install pip --upgrade
# upgrade python3.8 pip
python3.8 -m pip install pip --upgrade
bar


echo -e "${CYAN}Creating Python virtual environment${NC}"
bar
cd $VENV_DIR
# this doesn't overwrite it if it already exists
python3.8 -m venv $VENV_DIR --prompt "VRC" --upgrade
bar


# echo -e "${CYAN}Installing Docker${NC}"
# bar
# curl -fsSL https://get.docker.com -o get-docker.sh
# sh get-docker.sh
# bar


echo -e "${CYAN}Installing mavp2p${NC}"
bar
cd ~
wget https://github.com/aler9/mavp2p/releases/download/v0.6.5/mavp2p_v0.6.5_linux_arm7.tar.gz
tar -xvzf mavp2p_v0.6.5_linux_arm7.tar.gz
rm mavp2p_v0.6.5_linux_arm7.tar.gz
$s mv mavp2p /usr/bin/mavp2p

# start service
# TODO create a service file
cd $MAVLINK_DIR
$s systemctl enable mavp2p.service
$s systemctl start mavp2p.service

# Update the mavlink dialect in the venv
# TODO update for new FCC and mavsdk
# Make sure you're NOT in the virtual environment
# deactivate || true
# mkdir $VENV_DIR/lib/python3.8/site-packages/pymavlink/dialects/v20/ -p
# cp $VRC_DIR/vmc-core/mavlink/bell.py $VENV_DIR/lib/python3.8/site-packages/pymavlink/dialects/v20/
# grep -qxF 'export MAVLINK20=1' $VENV_DIR/bin/activate || echo 'export MAVLINK20=1' >> $VENV_DIR/bin/activate
# grep -qxF 'export MAVLINK_DIALECT=bell' $VENV_DIR/bin/activate || echo 'export MAVLINK_DIALECT=bell' >> $VENV_DIR/bin/activate
# done
bar


echo -e "${CYAN}Installing librealsense${NC}"
bar

# old method of downloading prebuilt binaries from VRC github
# cd $VRC_DIR/utilities/
# wget https://github.com/bellflight/vrc/releases/download/0.0.2/librealsense_files.zip
# unzip librealsense_files.zip
# $s rm librealsense_files.zip
# cp -a $VRC_DIR/utilities/librealsense_files/. $VENV_DIR/lib/

# install librealsense dependencies
$s apt install -y libssl-dev libusb-1.0-0-dev pkg-config build-essential cmake cmake-curses-gui libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev python3-dev

# clone librealsense
git clone https://github.com/IntelRealSense/librealsense $REALSENSE_DIR --recurse-submodules || true
cd $REALSENSE_DIR
git pull origin $LIBREALSENSE_VERSION
git submodule update --init --recursive
git checkout $LIBREALSENSE_VERSION

# build it
export CUDACXX=/usr/local/cuda/bin/nvcc
export PATH=${PATH}:/usr/local/cuda/bin
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64

mkdir -p build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DFORCE_LIBUVC=true -DBUILD_WITH_CUDA=bool:true -DCMAKE_BUILD_TYPE=release -DBUILD_PYTHON_BINDINGS=bool:true -DPYTHON_EXECUTABLE=/usr/bin/python3.8
# build with one cpu core less than we have to keep things responsive
make -j$(($(nproc) - 1))
$s make install
# apply udev rules
cd ..
$s cp config/99-realsense-libusb.rules /etc/udev/rules.d/
$s udevadm control --reload-rules && udevadm trigger

# https://lieuzhenghong.com/how_to_install_librealsense_on_the_jetson_nx/
# https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python
grep -qxF 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib' $VENV_DIR/bin/activate || echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib' >> $VENV_DIR/bin/activate
bar


echo -e "${CYAN}Installing VRC Python requirements${NC}"
bar
cd $VRC_DIR
# make sure pip and wheek are up to date
python -m pip install pip wheel --upgrade
# install overall dependencies
python -m pip install -r requirements.txt
deactivate
bar


echo -e "${CYAN}Installing final utilities${NC}"
bar
# install jtop and nano text editor
$s -H python3 -m pip install -U jetson-stats
$s apt install nano
bar


echo -e "${CYAN}Performing setup self-test${NC}"
bar
if [ -z "$(which python3.8)" ]; then
    echo -e "${RED}python3.8 not installed!${NC}"
    exit 1
fi

if [[ ! -d  $VENV_DIR ]]; then
    echo -e "${RED}Virtual environment not created!${NC}"
    exit 1
fi

source $VENV_DIR/bin/activate

python -c "import cv2" || (echo -e "${RED}OpenCV2 not installed!${NC}" && exit 1)
python -c "import pyrealsense2" || (echo -e "${RED}realsense not installed!${NC}" && exit 1)

deactivate

if [ -z "$(which mavp2p)" ]; then
    echo -e "${RED}mavp2p not installed!${NC}"
    exit 1
fi

if [ -z "$(which jtop)" ]; then
    echo -e "${RED}jtop not installed!${NC}"
    exit 1
fi

echo -e "${GREEN}All systems go.${NC}"
bar


echo  -e "${CYAN}VRC Finished setting up!${NC}"
echo  -e "${CYAN}Please reboot your VMC now.${NC}"
bar
