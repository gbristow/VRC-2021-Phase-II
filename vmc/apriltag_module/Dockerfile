FROM nvcr.io/nvidia/l4t-ml:r32.6.1-py3

ENV OPENBLAS_CORETYPE=ARMV8

RUN apt-get update -y
RUN apt-get install -y \
    python3-pip

WORKDIR /app

COPY python/requirements.txt requirements.txt

RUN python3 -m pip install pip wheel --upgrade && \
    python3 -m pip install -r requirements.txt

COPY . .

#CMD ["python3", "vrcapriltag.py"]

CMD ["./c/vrcapriltags"]