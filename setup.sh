#!/bin/bash

# exit when any command fails
set -e

bar () {
    # prints a bar equal to the current terminal width
    printf '=%.0s' $(seq 1 "$(tput cols)") && printf "\n"
}

# colors
RED='\033[0;31m'
LIGHTRED='\033[1;31m'
GREEN='\033[0;32m'
LIGHTGREEN='\033[1;32m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

VRC_DIR=~/Documents/VRC-2021-Phase-II

# see if sudo is installed
# mainly for testing with Docker, that doesn't have sudo
s=""
if [ -n "$(which sudo)" ]; then
    s="sudo"
fi

# check to make sure code has already been cloned
if [[ ! -d  $VRC_DIR ]]; then
    echo "VRC repository has not been cloned to $VRC_DIR"
    echo "Do this with '$s apt update && $s apt install -y git && git clone https://github.com/bellflight/VRC-2021-Phase-II $VRC_DIR'"
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
export DEBIAN_FRONTEND=noninteractive
# upgrade existing packages
$s apt upgrade -y
bar


echo -e "${CYAN}Installing prerequisites${NC}"
bar
# install some useful prereqs
$s apt install -y git apt-transport-https ca-certificates apt-utils software-properties-common gnupg lsb-release unzip curl htop nano python3 python3-wheel python3-pip
$s -H python3 -m pip install -U jetson-stats
cd $VRC_DIR
# cache the git credentials (mainly during development)
git config --global credential.helper cache
# update repo
git pull
# switch to main branch
git checkout main
bar


echo -e "${CYAN}Installing Docker${NC}"
bar
$s apt remove -y docker || true
$s apt remove -y docker-engine|| true
$s apt remove -y docker.io || true
$s apt remove -y containerd || true
$s apt remove -y runc || true

# add docker GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | $s gpg --batch --yes --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
# add docker repository
echo \
  "deb [arch=arm64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | $s tee /etc/apt/sources.list.d/docker.list > /dev/null

# install docker
$s apt update
$s apt install -y docker-ce:arm64 docker-ce-cli:arm64 containerd.io:arm64 docker-compose:arm64

# set up group rights for docker
$s groupadd docker || true
$s usermod -aG docker "$USER"
newgrp docker 

cd $VRC_DIR
$s docker-compose pull
bar


echo -e "${CYAN}Cleaning up${NC}"
bar
$s apt autoremove -y
bar


echo  -e "${GREEN}VRC Phase 2 finished setting up!${NC}"
echo  -e "${GREEN}Please reboot your VMC now.${NC}"
bar