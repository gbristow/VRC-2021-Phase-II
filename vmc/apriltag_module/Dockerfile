FROM mdegans/tegra-opencv:latest

ADD --chown=root:root https://repo.download.nvidia.com/jetson/jetson-ota-public.asc /etc/apt/trusted.gpg.d/jetson-ota-public.asc
RUN chmod 644 /etc/apt/trusted.gpg.d/jetson-ota-public.asc \
    && apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates

RUN apt-get install -y python3-pip

# Change the working directory to /app
WORKDIR /app

# Copy the file requirements.txt from the host into the container
COPY requirements.txt requirements.txt

RUN python3 -m pip install pip wheel --upgrade && \
    python3 -m pip install -r requirements.txt

# Now, copy everything else in this folder into the container.
COPY . .

# Finally, start the container running our code.
CMD ["python3", "capture_device.py"]