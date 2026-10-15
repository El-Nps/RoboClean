
# Deployment

[← Docs](../README.md)

To communicate with the robot, devices have to share the same network. We recommend using a mobile hotspot for this. The credentials (and a custom hostname) can be flashed into the OS for the Raspberry Pi. The Pi can then be accessed under the assigned hostname, as shown below.

## Build the production image for the Raspberry Pi

In most cases, the Docker image is built on a different platform than the Pi. This can be done via:

```bash
docker buildx build --platform linux/arm64 -f Dockerfile.prod -t roboclean:prod --output type=docker,dest=./roboclean.tar .
```

The resulting image will be about 1GB in size.

## Setup the Raspberry Pi

The deployment on the Raspberry Pi uses the docker engine running on Ubuntu Server 24.04 LTS (64-bit).

```bash
# SSH into the Pi
ssh root@roboclean.local

# Download and run the install script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add your user to the docker & dialout group
sudo usermod -aG docker $USER
sudo usermod -aG dialout $USER

# Enable Docker to start on boot
sudo systemctl enable docker

# Reboot to apply group changes
sudo reboot
```

These steps are only required once.
Copy the deployment files onto the Pi:

```bash
# On the host machine
scp docker-compose.prod.yml root@roboclean.local:~/
scp roboclean.tar root@roboclean.local:~/
```

## Persistent USB Device Paths

The setup requires fixed USB device paths for both the Arduino and the LiDAR.

1. Plug in both devices and run lsusb to find their Vendor/Product IDs.

2. Create a file `/etc/udev/rules.d/99-robot-serial.rules`:

    ```bash
    # If the devices are changed, insert the Vendor and Product IDs here
    SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="sensors/rplidar"
    SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="sensors/arduino"
    ```

3. Reload rules:

    ```bash
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

This procedure creates two persistent paths `/dev/sensors/arduino` and `/dev/sensors/rplidar`.

## Run the production image

```bash
docker load -i roboclean.tar
docker compose -f docker-compose.prod.yml up -d --force-recreate
```

_**Note:** The docker container is configured to always restart._

To debug the production image, you can open an interactive bash shell using:

```bash
# The entrypoint sources ROS
docker exec -it root-core-1 /entrypoint.sh bash
```

## Automation

The remote Docker image can be reloaded with the local version by using the [`/utils/reload_image.sh`](./utils/reload_image.sh) script. The script performs the steps described above.

[Firmware →](../firmware/README.md)
