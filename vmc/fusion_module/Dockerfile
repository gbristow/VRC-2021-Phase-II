FROM docker.io/library/python:3.9-buster

RUN apt-get update -y
RUN apt-get install -y \
    python3-pip

WORKDIR /app

COPY requirements.txt requirements.txt
RUN python3 -m pip install pip wheel --upgrade && \
    python3 -m pip install -r requirements.txt

COPY . .

CMD ["python3", "fusion.py"]