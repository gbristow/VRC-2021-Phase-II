#!/bin/bash

# exit when any command fails
set -e

# see if sudo is installed
# mainly for testing with Docker, that doesn't have sudo
s=""
if [ -n "$(which sudo)" ]; then
    s="sudo"
fi

$s service mosquitto start