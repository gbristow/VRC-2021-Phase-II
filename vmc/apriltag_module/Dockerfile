FROM nvcr.io/nvidia/l4t-ml:r32.6.1-py3

ENV OPENBLAS_CORETYPE=ARMV8

RUN apt-get update -y
RUN apt-get install -y \
    python3-pip

WORKDIR /app

COPY . .

COPY python/requirements.txt requirements.txt

RUN python3 -m pip install pip wheel --upgrade && \
    python3 -m pip install -r requirements.txt

WORKDIR /app/c/

RUN g++ -I/usr/include/opencv4/ -I/usr/local/cuda-10.2/include -I/opt/nvidia/vpi/include/  -L./libraries/lib_aarch64_jetpack44 -L/usr/local/cuda-10.2/lib64  -L/opt/nvidia/vpi1/lib64 -L/opt/nvidia/vpi/lib vrcapriltags.cpp  -lapril_tagging -lcudart -lcuda -lcublas -lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui -lnvvpi
#for some reason this has to be done at runtime...
#RUN ldconfig /opt/nvidia/vpi1/lib64 

WORKDIR /app

#CMD ["python3", "vrcapriltag.py"]

CMD ["/bin/bash", "./start_module.sh"]