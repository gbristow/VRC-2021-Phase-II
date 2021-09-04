FROM nvcr.io/nvidia/l4t-ml:r32.6.1-py3

RUN apt-get update -y
RUN apt-get install -y \
    python3-pip 

RUN apt update

RUN apt install software-properties-common -y

RUN add-apt-repository deb https://repo.download.nvidia.com/jetson/common 32.6 main
RUN add-apt-repository deb https://repo.download.nvidia.com/jetson/<platform> 32.6 main

RUN apt install libnvvpi1 vpi1-dev vpi1-cross-aarch64-l4t -y

WORKDIR /app

COPY . .

WORKDIR /app/c/
RUN g++ -I/usr/include/opencv4/ -I/usr/local/cuda-10.2/include -I/opt/nvidia/vpi/include/vpi  -I/opt/nvidia/vpi1/include/vpi -L./lib_aarch64_jetpack44 -L/usr/local/cuda-10.2/lib64  -L/opt/nvidia/vpi1/lib64 -L/opt/nvidia/vpi/lib vrcapriltags.cpp  -lapril_tagging -lcudart -lcuda -lcublas -lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui -lnvvpi

WORKDIR /app

COPY python/requirements.txt requirements.txt
RUN python3 -m pip install pip wheel --upgrade && \
    python3 -m pip install -r requirements.txt


CMD ["python3", "python/vrcapriltag.py"]

#sudo docker run -it --entrypoint /bin/bash --privileged --runtime nvidia  -v /tmp/argus_socket:/tmp/argus_socket 55c0c0712954
