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