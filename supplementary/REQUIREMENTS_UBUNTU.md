# Pre-installation Steps for AAS on Ubuntu

> These instructions are tested using Ubuntu 22.04.5 LTS and Ubuntu 24.04.3 LTS

> [!TIP]
> Run [`check_requirements.sh`](/scripts/check_requirements.sh) to verify whether you need to follow the steps below

## Install Ubuntu with NVIDIA Driver

- Get/install an OS from a startup disk based on Ubuntu 22 or newer (e.g. `ubuntu-22.04.5-desktop-amd64.iso`)
  - Choose "Normal installation", "Download updates while installing Ubuntu", no "Install third-party software"
- Update the OS
  - Run "Software Updater" and restart
  - "Update All" in "Ubuntu Software" (including `killall snap-store && sudo snap refresh snap-store`)
  - Update and restart for "Device Firmware" as necessary
- In "Software & Updates", select "Using NVIDIA driver metapackage from `nvidia-driver-580` (proprietary)"
- (optional) Go to "Settings" -> "Power", select  the "Performance" "Power Mode" and disable all "Power Saving Options"

```sh
sudo apt update && sudo apt upgrade

nvidia-smi                          # Should report something like "Driver Version: 580.65.06, CUDA Version: 13.0"

sudo apt install -y mesa-utils
glxinfo -B                          # (optional) Check OpenGL renderer, to force GPU rendering, use $ sudo prime-select nvidia
```

## Install Docker Engine

```sh
# Based on https://docs.docker.com/engine/install/ubuntu/ and https://docs.docker.com/engine/install/linux-postinstall/

for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install Docker Engine
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo docker run hello-world         # Test Docker is working
sudo docker version                 # (optional) Check version

# Remove the need to sudo the docker command
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker                       # Reboot

docker run hello-world              # Test Docker is working without sudo
```

## Install NVIDIA Container Toolkit

```sh
# Based on https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

sudo apt-get update && sudo apt-get install -y --no-install-recommends curl gnupg2
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.19.0-1
sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

docker info | grep -i runtime       # Check the `nvidia` runtime is available

docker run --rm --gpus all nvcr.io/nvidia/cuda:12.9.1-cudnn-runtime-ubuntu22.04 nvidia-smi # Test nvidia-smi works in a container with CUDA
```

## Optimize Memory Usage

**Optionally**, increase the swap size and, if you have an SSD, decrease swappiness

```sh
# Increase Ubuntu's default 2GB swap memory to 8GB
sudo swapon --show
sudo swapoff /swapfile
sudo fallocate -l 8G /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon --show

# Decrease Ubuntu's default swappiness of 60 to 10 (to reduce SSD wear)
cat /proc/sys/vm/swappiness
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
cat /proc/sys/vm/swappiness
```

## Troubleshoot

To be able to pull the base Docker images frequently, you might have to log in to the NVIDIA Registry:

- Go to https://ngc.nvidia.com and login/create an account.
- Click on your account the top right, go to Setup -> Get API Key.
- Click "Generate API Key" -> "+ Generate Personal Key" for the "NCG Catalog" service, confirm, and copy the key.

```sh
docker login nvcr.io                # To be able to reliably pull NVIDIA base images
Username:                           # type $oauthtoken
Password:                           # copy and paste the API key and press enter to pull base images from nvcr.io/
```
