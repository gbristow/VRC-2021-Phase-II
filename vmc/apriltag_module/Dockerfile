FROM nvcr.io/nvidia/l4t-base:r32.6.1

RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates

RUN apt-get install -y python3-pip

FROM mdegans/tegra-opencv:latest

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