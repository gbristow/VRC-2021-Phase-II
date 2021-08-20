#!/bin/bash
set -e

# copy the new dialect files
python3 -m pip download pymavlink --no-deps
tar -xvzf pymavlink*.tar.gz --wildcards pymavlink*/dialects/v20/common.* --strip-components 3
tar -xvzf pymavlink*.tar.gz --wildcards pymavlink*/dialects/v20/minimal.* --strip-components 3

# use mavgen tool
git clone https://github.com/mavlink/mavlink.git --recursive mavlink2 || true
cd mavlink2
python3 -m pip install -r pymavlink/requirements.txt
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=../bell.py ../bell.xml

# cleanup
cd ..
rm -rf mavlink2/
rm -rf pymavlink*.tar.gz