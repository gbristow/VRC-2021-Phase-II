FROM nvcr.io/nvidia/l4t-ml:r32.6.1-py3

ENV OPENBLAS_CORETYPE=ARMV8

RUN apt-get update -y
RUN apt-get install -y \
    python3-pip

WORKDIR /app

COPY . .

WORKDIR /app/c/

RUN echo $PATH

RUN g++ -I/usr/include/opencv4/ -I/usr/local/cuda-10.2/include -I/opt/nvidia/vpi/include/vpi  -I/opt/nvidia/vpi1/include/vpi -L./lib_aarch64_jetpack44 -L/usr/local/cuda-10.2/lib64  -L/opt/nvidia/vpi1/lib64 -L/opt/nvidia/vpi/lib vrcapriltags.cpp  -lapril_tagging -lcudart -lcuda -lcublas -lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui -lnvvpi

WORKDIR /app

COPY python/requirements.txt requirements.txt

RUN python3 -m pip install pip wheel --upgrade && \
    python3 -m pip install -r requirements.txt

#CMD ["python3", "vrcapriltag.py"]

CMD ["/bin/bash", "./start_module.sh"]