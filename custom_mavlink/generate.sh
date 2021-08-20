#!/bin/bash

set -e
shopt -s dotglob

MAVLINK_VERSION=master
PX4_VERSION=v1.10.2

# record starting directory
startdir=$(pwd)

echo "--- Cleaning old data"
cd "$(dirname "$0")"
rm -rf build/
mkdir -p build
cd build

echo "--- Cloning MAVLink"
mavlinkdir="mavlink"
git clone https://github.com/mavlink/mavlink.git $mavlinkdir --branch $MAVLINK_VERSION --recursive

echo "--- Cloning MAVLink C Library"
cdir="c_library_v2"
git clone https://github.com/mavlink/c_library_v2 $cdir --branch master --recursive

echo "--- Copying MAVLink dialect"
cp ../bell.xml $mavlinkdir/message_definitions/v1.0/bell.xml

cp $mavlinkdir/message_definitions/v1.0/bell.xml ../target/bell.xml
cp $mavlinkdir/message_definitions/v1.0/common.xml ../target/common.xml
cp $mavlinkdir/message_definitions/v1.0/minimal.xml ../target/minimal.xml

echo "--- Creating Python venv"
deactivate || true
rm -rf .tmpvenv/
python3 -m venv .tmpvenv
source .tmpvenv/bin/activate
python3 -m pip install pip wheel --upgrade
python3 -m pip install -r $mavlinkdir/pymavlink/requirements.txt

# generate Python code
echo "--- Generating Python MAVLink code"
mkdir -p ../target
cd $mavlinkdir
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=../../target/bell.py message_definitions/v1.0/bell.xml
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=../../target/common.py message_definitions/v1.0/common.xml
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=../../target/minimal.py message_definitions/v1.0/minimal.xml

echo "--- Generating Wireshark MAVLink Lua plugins"
# https://mavlink.io/en/guide/wireshark.html
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=../../target/bell.lua message_definitions/v1.0/bell.xml
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=../../target/common.lua message_definitions/v1.0/common.xml
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=../../target/minimal.lua message_definitions/v1.0/minimal.xml

echo "--- Generating C code"
# https://github.com/mavlink/mavlink/blob/master/scripts/update_c_library.sh
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=../$cdir message_definitions/v1.0/ardupilotmega.xml
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=../$cdir message_definitions/v1.0/matrixpilot.xml
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=../$cdir message_definitions/v1.0/test.xml
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=../$cdir message_definitions/v1.0/ASLUAV.xml
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=../$cdir message_definitions/v1.0/standard.xml
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=../$cdir message_definitions/v1.0/development.xml
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=../$cdir message_definitions/v1.0/bell.xml
cp message_definitions/v1.0/* ../$cdir/message_definitions/.

echo "--- Cloning PX4"
cd ..
px4dir="PX4-Autpilot"
# DO NOT recurse submodules yet, we need to apply patches first
git clone https://github.com/PX4/PX4-Autopilot --depth 5 $px4dir --branch $PX4_VERSION 

echo "--- Applying PX4 patches"
# apply patch
cd $px4dir
git apply ../../hil_gps_heading_$PX4_VERSION.patch
# changes need to be staged to refresh submodules
git add . 
# removed submodule cache for mavlink
git rm --cached mavlink/include/mavlink/v2.0/
cd ..
# copy C code
cp -r $cdir/* $px4dir/mavlink/include/mavlink/v2.0/.
cd $px4dir

echo "--- Loading PX4 submodules"
git submodule init
git submodule update

base_docker_cmd="docker run --rm -w \"$(pwd)\" --volume=\"$(pwd)\":\"$(pwd)\":rw px4io/px4-dev-nuttx-focal:latest /bin/bash -c"
# build Pixhawk firmware
# echo "--- Building Pixhawk firmware"
# docker run --rm -w "$(pwd)" --volume="$(pwd)":"$(pwd)":rw px4io/px4-dev-nuttx-focal:latest /bin/bash -c "make px4_fmu-v5_default"

# build NXP firmware
echo "--- Building NXP firmware"
echo "$base_docker_cmd" "make nxp_fmuk66-v3_default"
"$base_docker_cmd" "make nxp_fmuk66-v3_default"

echo "--- Cleaning up"
cd ..
deactivate

cd ..
rm -rf build/

echo "--- Copying outputs"

cp target/*.xml ../vmc/flight_control/mavlink/
cp target/*.py ../vmc/flight_control/mavlink/

cd "$startdir"