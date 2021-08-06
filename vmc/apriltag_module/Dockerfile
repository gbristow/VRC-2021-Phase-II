FROM nvcr.io/nvidia/l4t-base:r32.6.1

# Change the working directory to /app
WORKDIR /app

# Copy the file requirements.txt from the host into the container
COPY requirements.txt requirements.txt

RUN python3 -m pip3 install pip wheel --upgrade && \
    python3 -m pip3 install -r requirements.txt

# Now, copy everything else in this folder into the container.
COPY . .

# Finally, start the container running our code.
CMD ["python3", "capture-device.py"]