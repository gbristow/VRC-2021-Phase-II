FROM nvcr.io/nvidia/l4t-ml:r32.6.1-py3

ENV OPENBLAS_CORETYPE=CORTEXA57

RUN apt-get update -y
RUN apt-get install -y \
    python3-pip \
    libssl-dev

WORKDIR /app

COPY . .

RUN python3 -m pip install pip wheel --upgrade \
 && python3 -m pip install -r python/requirements.txt

# Install paho C library
RUN git clone https://github.com/eclipse/paho.mqtt.c \
 && cd paho.mqtt.c \
 && git checkout v1.3.8 \
 && cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON \
 && cmake --build build/ --target install \
 && ldconfig

# Install paho C++ library
RUN git clone https://github.com/eclipse/paho.mqtt.cpp \
 && cd paho.mqtt.cpp \
 && git checkout v1.2.0 \
 && cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON -DPAHO_BUILD_DOCUMENTATION=FALSE -DPAHO_BUILD_SAMPLES=FALSE \
 && cmake --build build/ --target install \
 && ldconfig

# Clone dependencies
RUN mkdir -p c/libraries

RUN git clone https://github.com/nlohmann/json c/libraries/json \
 && cd c/libraries/json \
 && git checkout v3.10.2

RUN git clone https://github.com/NVIDIA-AI-IOT/isaac_ros_apriltag \
 && cd isaac_ros_apriltag \
 && git checkout v0.9.0-ea1 \
 && cp -r ../isaac_ros_apriltag/isaac_ros_apriltag/nvapriltags c/libraries

# Build the april tag application
RUN mkdir -p c/build \
 && cd c/build \ 
 && cmake .. \
 && make -j4

CMD ["/bin/bash", "./start_module.sh"]