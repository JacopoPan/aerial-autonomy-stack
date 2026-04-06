# Pre-installation Steps for AAS on Windows 11

> These instructions are tested using Windows 11 Enterprise 23H2 (OS build 22631.6199) on an i7-11 with 32GB RAM and RTX A2000

## Setup WSL for Ubuntu 22

> The latest Windows Subsystem for Linux (WSL2) and WSLg (WSL2 extension with GUI capabilities) are included in Windows 11

- If WSL is not enabled, use "Turn Windows features on or off" to checkbox [Hyper-V, Windows Hypervisor Platform, and Windows Subsystem for Linux](https://github.com/microsoft/WSL/issues/9521#issuecomment-2385289848)
- From PowerShell, update and default to WSL2 `wsl --update`, `wsl --set-default-version 2`
- Check the available Linux distributions `wsl --list --online`
- Install "Ubuntu-22.04" `wsl --install -d Ubuntu-22.04`
- Setup an account when prompted `Enter new UNIX username:` and `New password:`
- Check you have Ubuntu-22.04 on VERSION 2 with `wsl --list --verbose`

```sh
wsl ~                                                 # Access WSL from Windows PowerShell

sudo apt update && sudo apt upgrade

sudo apt install -y x11-apps x11-xserver-utils        # Install X11 apps and xserver
xclock                                                # Test: a new window with a clock should appear

free -h                                               # (optional) Check the memory and swap made available to WSL
```

> [!WARNING]
> When building and running large Docker images, WSL can easily consume available system resources: to prevent crashes, hangs, or 100% disk usage, configure WSL’s resource limits using a `.wslconfig` file
> 
> - Create (or edit) file `C:\Users\<YourWindowsUsername>\.wslconfig` (make sure it has no extension)
> - Add the following lines to it (change `YourWindowsUsername`; increase the amount of resources, if available)
> 
> ```sh
> [wsl2]
> memory=24GB
> processors=8
> swap=8GB
> swapfile=C:\\Users\\<YourWindowsUsername>\\AppData\\Local\\Temp\\wsl-swap.vhdx
> localhostForwarding=true
> ```
>
> After editing `.wslconfig`, restart WSL from PowerShell for the new settings to take effect:
>
> ```sh
> exit                              # If you are still running WSL in Windows PowerShell, exit
> wsl --shutdown 
> wsl ~
> free -h                           # Check the available memory and swap reflect .wslconfig
> ```

---

> [!TIP]
> Run [`check_requirements.sh`](/scripts/check_requirements.sh) **inside WSL** to verify whether you need to follow the steps below

## Install the NVIDIA Driver on Windows 11

Download and install the **NVIDIA driver 580 on Windows** using the [NVIDIA App](https://www.nvidia.com/en-us/software/nvidia-app/) 

> [!WARNING] 
> The latest NVIDIA Windows drivers fully support WSL2, enabling existing CUDA applications compiled on Linux to run unmodified in WSL, once the Windows NVIDIA driver is installed, CUDA is available in WSL2 *via* a stubbed `libcuda.so`
>
> **Do NOT install a separate NVIDIA GPU Linux driver inside WSL2**

```sh
wsl ~                               # Access WSL from Windows PowerShell

nvidia-smi                          # From WSL, check NVIDIA driver (these instructions are tested on Driver Version: 581.80, CUDA Version:13.0)

sudo apt update && sudo apt install -y mesa-utils
glxinfo -B                          # (optional) Check OpenGL renderer, to force GPU rendering, use $ echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc && source ~/.bashrc

```

## Install Docker Engine inside WSL

```sh
wsl ~                               # Access WSL from Windows PowerShell

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
newgrp docker                       # Reboot (exit; wsl --shutdown; wsl ~)

docker run hello-world              # Test Docker is working without sudo
```

## Install NVIDIA Container Toolkit inside WSL

```sh
wsl ~                               # Access WSL from Windows PowerShell

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

![wsl](https://github.com/user-attachments/assets/e58d039c-b23b-43a0-9957-d18579e652ec)
