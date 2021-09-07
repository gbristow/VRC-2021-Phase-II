FROM nvcr.io/nvidia/l4t-ml:r32.6.1-py3

ENV OPENBLAS_CORETYPE=ARMV8

RUN apt-get update -y
RUN apt-get install -y \
    python3-pip -y \
    libssl-dev

WORKDIR /app

COPY . .

COPY python/requirements.txt requirements.txt

RUN python3 -m pip install pip wheel --upgrade && \
    python3 -m pip install -r requirements.txt

#INSTALL PAHO
RUN git clone https://github.com/eclipse/paho.mqtt.c.git
WORKDIR /app/paho.mqtt.c
RUN git checkout v1.3.8

RUN cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON \
    -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
RUN cmake --build build/ --target install
RUN ldconfig

WORKDIR /app

RUN git clone https://github.com/eclipse/paho.mqtt.cpp
WORKDIR /app/paho.mqtt.cpp

RUN cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON \
    -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE
RUN cmake --build build/ --target install
RUN ldconfig


# BUILD THE APRILTAG APPLICATION
WORKDIR /app/c

WORKDIR /app/c/build

RUN cmake ..

RUN make

#RUN g++ -std=c++11 -I/usr/include/opencv4/ -I/usr/local/cuda-10.2/include -I/opt/nvidia/vpi1/include/ -I./libraries/json/inlcude  -L./libraries/lib_aarch64_jetpack44 -L/usr/local/cuda-10.2/lib64  -L/opt/nvidia/vpi1/lib64 -L/opt/nvidia/vpi/lib vrcapriltags.cpp  -lapril_tagging -lcudart -lcuda -lcublas -lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui -lnvvpi

#for some reason this has to be done at runtime...
#RUN ldconfig /opt/nvidia/vpi1/lib64 

WORKDIR /app

#CMD ["python3", "vrcapriltag.py"]

CMD ["/bin/bash", "./start_module.sh"]