FROM nvcr.io/nvidia/l4t-ml:r32.6.1-py3

RUN apt-get update -y
RUN apt-get install -y \
    python3-pip

WORKDIR /app

COPY requirements.txt requirements.txt
RUN python3 -m pip install pip wheel --upgrade && \
    python3 -m pip install -r requirements.txt

COPY . .

CMD ["python3", "fusion.py"]

#sudo docker run -it --entrypoint /bin/bash --privileged --runtime nvidia  -v /tmp/argus_socket:/tmp/argus_socket 55c0c0712954
