FROM nvcr.io/nvidia/l4t-base:r32.6.1

ENV LIBREALSENSE_VERSION=v2.31.0 
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /

RUN apt update -y
RUN apt install -y \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    build-essential \
    cmake \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    software-properties-common 
RUN add-apt-repository -y ppa:deadsnakes/ppa \
 && apt install -y \
    python3.8 \
    python3.8-dev \
    python3-pip 

RUN git clone https://github.com/IntelRealSense/librealsense --recurse-submodules \
 && cd librealsense \
 && git pull origin $LIBREALSENSE_VERSION \
 && git submodule update --init --recursive \
 && git checkout $LIBREALSENSE_VERSION

RUN mkdir -p build \
 && cd build \
 && cmake /librealsense \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=true \
    -DBUILD_WITH_CUDA=bool:true \
    -DCMAKE_BUILD_TYPE=release \
    -DBUILD_PYTHON_BINDINGS=bool:true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
 && make -j$(($(nproc) - 1)) \
 && make install

RUN cp /usr/local/lib/pyrealsense2* /usr/local/lib/python3.8/dist-packages/ \
 && cp /usr/local/lib/pybackend2* /usr/local/lib/python3.8/dist-packages/ \
 && cp /usr/local/lib/librealsense2* /usr/local/lib/python3.8/dist-packages/

WORKDIR /app

COPY requirements.txt requirements.txt
RUN python3 -m pip install pip wheel --upgrade && \
    python3 -m pip install -r requirements.txt

COPY . .

CMD ["python3", "vio.py"]