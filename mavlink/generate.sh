#!/bin/bash

set -e
shopt -s dotglob

PX4_VERSION=v1.11.0

# record starting directory
startdir=$(pwd)

echo "--- Cleaning old data"
cd "$(dirname "$0")"
rm -rf build/
mkdir -p build
mkdir -p target
cd build

echo "--- Cloning MAVLink"
mavlinkdir="mavlink"
git clone https://github.com/mavlink/mavlink.git $mavlinkdir --recursive

echo "--- Copying MAVLink dialect"
cp ../bell.xml ../target/bell.xml
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
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=../../target/bell.py ../../target/bell.xml
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=../../target/common.py ../../target/common.xml
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=../../target/minimal.py ../../target/minimal.xml

echo "--- Generating Wireshark MAVLink Lua plugins"
# https://mavlink.io/en/guide/wireshark.html
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=../../target/bell.lua ../../target/bell.xml
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=../../target/common.lua ../../target/common.xml
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=../../target/minimal.lua ../../target/minimal.xml

echo "--- Cloning PX4 $PX4_VERSION"
cd ..
px4dir="PX4-Autopilot"
git clone https://github.com/PX4/PX4-Autopilot --depth 5 $px4dir --branch $PX4_VERSION --recurse-submodules

echo "--- Applying PX4 patch"
# apply patch
cd $px4dir
git apply ../../hil_gps_heading_$PX4_VERSION.patch

echo "--- Injecting Bell MAVLink message"
cp ../../bell.xml mavlink/include/mavlink/v2.0/message_definitions/bell.xml
cd ../$mavlinkdir
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=../$px4dir/mavlink/include/mavlink/v2.0/ ../$px4dir/mavlink/include/mavlink/v2.0/message_definitions/bell.xml
cd ../$px4dir

# changes need to be committeed to build
git add . 
git commit -m "Local commit to facilitate build"

base_docker_cmd="docker run --rm -w \"$(pwd)\" --volume=\"$(pwd)\":\"$(pwd)\":rw px4io/px4-dev-nuttx-focal:latest /bin/bash -c"

# echo "--- Building PX4 SITL"
# echo "$base_docker_cmd 'make px4_sitl_default'"
# eval "$base_docker_cmd 'make px4_sitl_default -j$(nproc)'"

# build Pixhawk firmware
echo "--- Building Pixhawk firmware"
echo "$base_docker_cmd 'make px4_fmu-v5_default'"
eval "$base_docker_cmd 'make px4_fmu-v5_default -j$(nproc)'"
cp build/px4_fmu-v5_default/px4_fmu-v5_default.px4 ../../target/px4_fmu-v5_default.$PX4_VERSION.px4

# build NXP firmware
echo "--- Building NXP firmware"
echo "$base_docker_cmd 'make nxp_fmuk66-v3_default'"
eval "$base_docker_cmd 'make nxp_fmuk66-v3_default -j$(nproc)'"
cp build/nxp_fmuk66-v3_default/nxp_fmuk66-v3_default.px4 ../../target/nxp_fmuk66-v3_default.$PX4_VERSION.px4

echo "--- Cleaning up"
cd ..
deactivate

echo "--- Copying outputs"

cp target/*.xml ../vmc/flight_control/mavlink/
cp target/*.py ../vmc/flight_control/mavlink/

cd "$startdir"