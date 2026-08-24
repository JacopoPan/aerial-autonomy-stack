# Setup Avionics

> These instructions are tested on a [Holybro Jetson Baseboard](https://holybro.com/products/pixhawk-jetson-baseboard) with Pixhawk 6X and NVIDIA Orin NX 16GB on an [X650](https://holybro.com/collections/multicopter-kit/products/x650-development-kit)
>
> Alternative hardware options include: [ARK's Jetson PAB Orin NX NDAA Bundle](https://arkelectron.com/product/ark-jetson-orin-nx-ndaa-bundle/),  [ARK's Jetson PAB V3 Orin NX NDAA Bundle](https://arkelectron.com/product/ark-jetson-pab-v3-orin-nx-ndaa-bundle/), and a [Holybro's 6X Pro](https://holybro.com/collections/flight-controllers/products/pixhawk-6x-pro) or [CubePilot Cube Orange+](https://www.cubepilot.org/#/cube/features) paired with a [Seeed Studio's A603](https://www.seeedstudio.com/A603-Carrier-Board-for-Jetson-Orin-NX-Nano-p-5635.html) or [A608](https://www.seeedstudio.com/Jetson-A608-Carrier-Board-for-Orin-NX-Orin-Nano-Series-p-5853.html) Jetson carrier
>
> For the complete bill of materials of an `aerial-autonomy-stack`-enabled quadcopter, read [`BOM.md`](/tools_and_docs/docs/BOM.md)

## Flash JetPack 7 to Jetson Orin

Holybro Jetson baseboards normally ship with JetPack 5

To upgrade to JetPack 7, download NVIDIA SDK Manager on an Ubuntu 24 (see [compatibility matrix](https://developer.nvidia.com/sdk-manager#host_os_comp_matrix)) host computer from [here](https://developer.nvidia.com/sdk-manager#installation_get_started)

```sh
cd ~/Downloads
sudo apt install ./sdkmanager_[version]-[build#]_amd64.deb # Currently version 2.4.1, build 13536
sdkmanager                          # Log in with your https://developer.nvidia.com account
```

- Put the Holybro Jetson baseboard in recovery mode with the dedicated switch
- Connect the REC-USB-C port to the computer running `sdkmanager` and power on the board
- On Step 1, fields "Jetson", "Host Machine Ubuntu 2x x86_64", "Target Hardware Jetson Orin NX" are auto-detected
  - Select "SDK Version JetPack 7.2.1" and "Direct Flash", no additional SDKs
- On Step 2, under "Target Components", select all "Jetson Linux" (uncheck all others: Host Components, Jetons Runtime, SDK Components)
- Accept the "terms and conditions" and click "CONTINUE" (if prompted, click "Create" folder and/or input the password to `sudo`)
- Wait for `sdkmanager` to download the necessary software
- On the flash dialog after the download, choose "OEM Pre-config", username, password, and "Storage NVMe", click "Flash"
- On `sdkmanager` click "FINISH AND EXIT" once the process is completed
- Power-off, put the board out of recovery mode, disconnect the USB-C cable, and power-on again
- With a screen, mouse, and keyboard connected to the Jetson basedboad, log in, finish the configuration
- Select an appropriate "Power Mode" (e.g. MAXN or 25W)
- Under "Settings" -> "System" -> "Users", unlock and enable "Automatic Login"
- If connected to the internet, e.g. with a USB Ethernet adapter, Firefox can be installed from the "Software" application
- Prioritize SSD over Network Boot
```sh
sudo efibootmgr -v                  # Check what Boot000x entry is the SSD, e.g. 0001
sudo efibootmgr -o 0001,000B, etc.  # Copy line BootOrder from the previous command, moving the desired entry to the front
```

<!--
- [PX4 documentation](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/companion_computer/holybro_pixhawk_jetson_baseboard.md#flashing-the-jetson-board)
-->

## Configure Jetson-IO for the CSI IMX219-200 Camera

```sh
sudo /opt/nvidia/jetson-io/jetson-io.py

# Follow these steps:
#   "Configure Jetson 24pin CSI Connector"
#   -> "Configure for compatible hardware"
#   -> "Camera IMX219 Dual" (even if only using one)
#   -> "Save pin changes"
#   -> "Save and reboot to reconfigure pins"
#   -> Press any key to reboot

sudo dmesg | grep -i imx219         # After reboot, this will show at least one imx219 successfully bound

# Inspect device (e.g. /dev/video0) resolution and frame rate
sudo apt update && sudo apt install -y v4l-utils
v4l2-ctl --list-formats-ext -d /dev/video0
```

## Install Docker Engine on Jetson Orin

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

## Install NVIDIA Container Toolkit on Jetson Orin

```sh
# Based on https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

sudo apt-get update && sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg2
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.20.0-1
sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

docker info | grep -i runtime       # Check the `nvidia` runtime is available

docker run --rm --runtime=nvidia -e NVIDIA_DISABLE_REQUIRE=true nvcr.io/nvidia/cuda:13.3.1-tensorrt-devel-ubuntu24.04 nvidia-smi # Test nvidia-smi works in a container with CUDA and TensorRT
```

## Build and Flash PX4 or ArduPilot Firmware

Build the PX4 firmware in `simulation-image` for Pixhawk 6X

```sh
# Check available PX4 targets
docker run -it --rm --entrypoint bash simulation-image -c "cd /aas/github_apps/PX4-Autopilot && make list_config_targets"
```

```sh
# Build PX4 for Pixhawk 6X (saved in the ~/Downloads folder)
docker run -it --rm --entrypoint bash -v ~/Downloads:/temp simulation-image -c \
  "cd /aas/github_apps/PX4-Autopilot && make px4_fmu-v6x_default && cp build/px4_fmu-v6x_default/*.px4 /temp/"
```

Build the ArduPilot firmware in `simulation-image` for Pixhawk 6X

```sh
# Check available ArduPilot targets
docker run -it --rm --entrypoint bash simulation-image -c "cd /aas/github_apps/ardupilot && ./waf list_boards"
```

```sh
# Build ArduCopter (quads) for Pixhawk 6X (saved in the ~/Downloads folder)
docker run -it --rm --entrypoint bash -v ~/Downloads:/temp simulation-image -c \
  "cd /aas/github_apps/ardupilot && ./waf configure --board Pixhawk6X && ./waf copter && cp build/Pixhawk6X/bin/*.apj /temp/"
```

```sh
# Build ArduPlane (VTOLs) for Pixhawk 6X (saved in the ~/Downloads folder)
docker run -it --rm --entrypoint bash -v ~/Downloads:/temp simulation-image -c \
  "cd /aas/github_apps/ardupilot && ./waf configure --board Pixhawk6X && ./waf plane && cp build/Pixhawk6X/bin/*.apj /temp/"
```

To flash the newly created `.px4` or `.apj` file to your autopilot board, follow [QGC's User Guide](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/setup_view/firmware.html)

## PX4: Configure 6X's Network and DDS Client

On the Jetson Baseboard's Orin NX, under "Settings" -> "Network", configure the "PCI Ethernet" connection to "Manual" with IPv4 with address 10.10.1.44 and netmask 255.255.255.0

Connect the Pixhawk 6X to the ground station with the USB-C port next to the RJ-45 port

- Access QGroundControl -> "Analyze Tools" -> "MAVLink console"
- Copy-and-paste the following commands (these will assign an IP to the PX4 autopilot (e.g., 10.10.1.33) and let the `uxrce_dds_client` connect to the Orin NX (e.g., on IP 10.10.1.44) using namespace `Drone1`)
- Re-start the autopilot

```sh
# Configure DDS Client connection to the NX
mkdir /fs/microsd/etc
cd /fs/microsd/etc
echo "uxrce_dds_client stop" > extras.txt
echo "sleep 3" >> extras.txt
echo -n "uxrce_dds_client start -p 8888" >> extras.txt
echo " -h 10.10.1.44 -n Drone1" >> extras.txt

# Check the content of the file (remember to set the proper "DroneX" namespace)
cat /fs/microsd/etc/extras.txt

# Configure Autopilot network settings
echo DEVICE=eth0 > /fs/microsd/net.cfg
echo BOOTPROTO=static >> /fs/microsd/net.cfg
echo IPADDR=10.10.1.33 >> /fs/microsd/net.cfg
echo NETMASK=255.255.255.0 >> /fs/microsd/net.cfg
echo ROUTER=10.10.1.254 >> /fs/microsd/net.cfg
echo DNS=10.10.1.254 >> /fs/microsd/net.cfg

# Check the content of the file
cat /fs/microsd/net.cfg
# Check the current network configuration
ifconfig

# Apply the new configuration
netman update
# Reboot and check new network configuration
ifconfig

# Set vehicle/MAV_SYS_ID (for multiple QGC connections) and UXRCE_DDS_DOM_ID/ROS_DOMAIN_ID
param set MAV_SYS_ID 1
param set UXRCE_DDS_DOM_ID 1

# Optionally
param set MAV_2_CONFIG 0            # Disable MAVLINK on Ethernet (so Ethernet is used for XRCE-DDS only), if needed, also check params MAV_0_CONFIG, MAV_1_CONFIG
param set UXRCE_DDS_CFG 1000        # Use DDS over Ethernet
```

> [!CAUTION]
> Match `MAV_SYS_ID`, `UXRCE_DDS_DOM_ID`, and `uxrce_dds_client`'s namespace `-n Drone1`, with the `DRONE_ID` used to launch `./deploy_run.sh`: this is the `ROS_DOMAIN_ID` of the aircraft container

One should be able to `ping 10.10.1.44` (the Orin NX) from MAVLink Console on QGC; and `ping 10.10.1.33` (the autopilot) from a terminal on the Orin NX

<!--
- [PX4 documentation](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/companion_computer/holybro_pixhawk_jetson_baseboard.md#ethernet-setup-using-netplan)
-->

## ArduPilot: Configure 6X's Serial for MAVROS

MAVROS can be connected to MAVLink using the Pixhawk 6X's `TELEM2` serial port (this is ArduPilot's `SERIAL2` and Orin's `/dev/ttyTHS1`)

In QGroundControl -> "Vehicle Configuration" -> "Parameters" set:

```sh
SERIAL2_BAUD 921600
SERIAL2_OPTIONS 0
SERIAL2_PROTOCOL MAVLink2

# Stream rates for MAVROS
SR2_ADSB         5
SR2_EXTRA1       50
SR2_EXTRA2       10
SR2_EXTRA3       2
SR2_EXT_STAT     1
SR2_PARAMS       10
SR2_POSITION     10
SR2_RAW_CTRL     1
SR2_RAW_SENS     50
SR2_RC_CHAN      1
```

> [!CAUTION]
> Match ArduPilot parameter `SYSID_THISMAV` with the `DRONE_ID` used to launch `./deploy_run.sh`: this is the `ROS_DOMAIN_ID` of the aircraft container

<!--
- [ArduPilot serial configuration](https://ardupilot.org/copter/docs/common-serial-options.html)
- [Jetson baseboard serial configuration](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/companion_computer/holybro_pixhawk_jetson_baseboard.md#serial-connection)
- [MAVROS connection](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md)
- [Requesting MAVLink data from ArduPilot](https://ardupilot.org/dev/docs/mavlink-requesting-data.html)
-->

## Setup TELEM1 for the Radio Telemetry Link with the Ground Station

TELEM1 is the [bottom-right 6-pin port on the Jetson Baseboard](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-jetson-baseboard/ports-pinout#tel1-tel3-ports)

To use it to connect a ground station (e.g. QGC) with a telemetry radio (e.g., [Holybro 1W SiK telemetry](https://holybro.com/collections/telemetry-radios/products/sik-telemetry-radio-1w) for point-to-point or [Holybro Microhard telemetry](https://holybro.com/collections/telemetry-radios/products/microhard-telemetry-radio-v2) for point-to-multipoint), use the following parameters

### PX4 Configuration

- Access QGroundControl -> "Analyze Tools" -> "MAVLink console"
- Copy-and-paste the following commands to set the GLOBAL_POSITION_INT stream rate to 10Hz on TELEM1
- Re-start the autopilot

```sh
mkdir /fs/microsd/etc
cd /fs/microsd/etc
echo "mavlink stream -r 10 -s GLOBAL_POSITION_INT -d /dev/ttyS6" >> extras.txt
# Verify serial port name: https://docs.px4.io/main/en/flight_controller/pixhawk6x#serial-port-mapping

# Check the content of the file
cat /fs/microsd/etc/extras.txt
```

```sh
MAV_0_CONFIG        TELEM 1
MAV_0_FLOW_CTRL     Auto-detected
MAV_0_FORWARD       Enabled
MAV_0_MODE          Normal
MAV_0_RATE          2400B/s

SER_TEL1_BAUD       57600 8N1
# All these are default values and tested with "Holybro SiK Telemetry Radio - Long Range; SKU: 17031"
# Note: on a point-to-multipoint configuration with multiple drones the serial baud rate between the ground radio and GCS computer should be greater than SER_TEL1_BAUD, scaling with the number of drones yet capped by the air link (e.g., GND_TELEM_BAUD=230400 in ./deploy_run.sh)
```

### ArduPilot Configuration

```sh
BRD_SER1_RTSCTS     Auto

SERIAL1_BAUD        57600
SERIAL1_OPTIONS     0
SERIAL1_PROTOCOL    MAVLink2

# Stream rates for the telemetry radio
# Note: for the values below to be authoritative, one MUST change QGroundControl default settings
# In QGroundControl -> Application Settings -> Telemetry
# Make sure the "ArduPilot Only Stream Rates" are "set by the vehicle", NOT "requested"
# This is enforced by "apmStartMavlinkStreams=false" in QGroundControl.ini
SR1_ADSB         0
SR1_EXTRA1       4
SR1_EXTRA2       4
SR1_EXTRA3       2
SR1_EXT_STAT     2
SR1_PARAMS       10
SR1_POSITION     10
SR1_RAW_CTRL     0
SR1_RAW_SENS     0
SR1_RC_CHAN      1
# Tested with "Holybro SiK Telemetry Radio - Long Range; SKU: 17031"
# Note: on a point-to-multipoint configuration with multiple drones the serial baud rate between the ground radio and GCS computer should be greater than SERIAL1_BAUD, scaling with the number of drones yet capped by the air link (e.g., GND_TELEM_BAUD=230400 in ./deploy_run.sh)
```

### SiK (point-to-point) and Microhard (point-to-multipoint) Radio Configuration

```sh
groups                              # Check 'dialout' is in the list otherwise run 'sudo usermod -aG dialout $USER' and reboot
sudo apt install -y picocom         # Install picocom
```

For the [Holybro 1W SiK telemetry](https://holybro.com/collections/telemetry-radios/products/sik-telemetry-radio-1w), set `NETID` to use multiple pairs in point-to-point configuration simultaneously

```sh
ls /dev/ttyUSB*                     # List devices
# If necessary, close QGroundControl or anything else using the serial ports on which the radios are connected

# To review the configuration of a connected point-to-point pair of Sik radios
picocom -b 57600 /dev/ttyUSB0       # Connect to one of the devices listed above (in this case, ttyUSB0)
# If the '+++' command below never returns "OK", the radio's SERIAL_SPEED may differ, retry, e.g., with '-b 115200'
# If the radio is paired and connected to an autopilot, it will print gibberish, just ignore it
+++                                 # Do NOT press Enter, wait for OK, it will enter Command Mode
ATI5                                # Press Enter, to list the local radio parameters
RTI5                                # Press Enter, to list the paired drone radio parameters, over the air, retry if necessary
ATI7                                # Press Enter, shows link info, RSSI, etc
ATI                                 # Press Enter, shows firmware info
ATO                                 # Press Enter, exits Command Mode/returns to Data Mode ('O' not zero)
# Ctrl+a, Ctrl+x to finally exit picocom

# Similarly, to edit the NETID of a Sik radio (do this on each of the two radios in a pair, do not update NETID over the air)
picocom -b 57600 /dev/ttyUSB0
+++                                 # Do NOT press Enter, wait for OK, it will enter Command Mode
ATS3=31                             # Press Enter, set the NETID of the radio connected via USB-C to, e.g., 31 (radios in the same pair must use the same NETID, choose a different value for each pair)
AT&W                                # Press Enter, write the change
ATZ                                 # Press Enter, reboot and apply (also exits Command Mode)
# Wait a few seconds for the reboot
+++                                 # Do NOT press Enter, wait for OK, it will enter Command Mode
ATI5                                # Press Enter, to list the local radio parameters, verify the change
ATO                                 # Press Enter, exits Command Mode/returns to Data Mode ('O' not zero)
# Ctrl+a, Ctrl+x to finally exit picocom

# Note, the following parameters MUST match on the two ends of each pair (factory defaults are ok):
# - S2:AIR_SPEED - over-the-air data rate in kbps
# - S3:NETID - network ID, isolates a pair from other SiK radios nearby
# - S8:MIN_FREQ/S9:MAX_FREQ - frequency band edges in kHz
# - S10:NUM_CHANNELS - channels in the frequency band
# - S13:MANCHESTER - encoding (off by default)
# Optional:
# - S4:TXPOWER - set to 30 for 1W if supported/allowed
```

For the [Holybro Microhard V2 P900 telemetry](https://holybro.com/collections/telemetry-radios/products/microhard-telemetry-radio-v2), use the point-to-multipoint configuration

```sh
ls /dev/ttyUSB*                     # List devices
# If necessary, close QGroundControl or anything else using the serial ports on which the radios are connected

# To review the configuration of a Microhard V2 P900 radio
picocom -b 9600 /dev/ttyUSB0        # Connect to one of the devices listed above (in this case, ttyUSB0), Command Mode on Microhard runs at 9600
# With picocom open, enter Command Mode:
#   1. Remove power from the XT30 port
#   2. Press and HOLD the CONFIG button
#   3. Re-apply power to the XT30 port, wait for boot and release the CONFIG button
# Session prints "NO CARRIER / OK" to confirm entering Command Mode
AT&V                                # Press Enter, to list the local radio parameters
ATA                                 # Press Enter, exits Command Mode/returns to Data Mode
# Ctrl+a, Ctrl+x to exit picocom

# Holybro Microhard V2 telemetries factory settings are point-to-point configurations
# The following instructions apply the point-to-multipoint configuration

# Master radio (GCS)
# Connect picocom and restart the radio holding the CONFIG button as above
AT&F7                               # Press Enter, set PMP Master profile: S133=0 (PMP), S101=0 (Master), S140=65535 (broadcast to all remotes), S105=1 (master unit address, do not change)
ATS102=0                            # Press Enter, increase the serial baud to 230400 to receive multiple drone telemetries (this is the data baud rate, not the 9600 Command Mode nor the air link baud rate)
ATS103=2                            # Press Enter, increase the air link rate to 276480bps
ATS104=1337                         # Press Enter, set Network ID, e.g., 1337 (radios in the point-to-multipoint network must use the same Network ID)
ATS108=27                           # Press Enter, optional: 20dBm=100mW, 27dBm=500mW, 30dBm=1W
AT&V                                # Press Enter, verify: S133=0, S101=0, S140=65535, S102=0, S103=2, S104=1337, S105=1
AT&W                                # Press Enter, write the changes
ATA                                 # Press Enter, return to data mode

# Remote radio #1 (Drone 1)
# Connect picocom and restart the radio holding the CONFIG button as above
AT&F8                               # Press Enter, set PMP Slave profile: S133=0 (PMP), S101=2 (Remote), S140=1 (send data to master, do not change)
ATS102=2                            # Press Enter, set a 57600 serial baud for each drone radio (this is the data baud rate, not the 9600 Command Mode nor the air link baud rate)
ATS103=2                            # Press Enter, increase the air link rate to 276480bps
ATS104=1337                         # Press Enter, set Network ID, e.g., 1337 (radios in the point-to-multipoint network must use the same Network ID)
ATS108=27                           # Press Enter, optional: 20dBm=100mW, 27dBm=500mW, 30dBm=1W
ATS105=2                            # Press Enter, set the unit address, MUST be unique per remote
AT&K3                               # Press Enter, enable RTS/CTS handshaking towards the FCU
AT&V                                # Press Enter, verify: S133=0, S101=2, S140=1, S102=2, S103=2, S104=1337, S105=2, and "Handshaking &K3" in the header line
AT&W                                # Press Enter, write the changes
ATA                                 # Press Enter, return to data mode

# Remote radio #2 (Drone 2)
# Connect picocom and restart the radio holding the CONFIG button as above
# Follow the same steps as for Remote 1, except ATS105
...
ATS105=3                            # Press Enter, set the unit address, MUST be unique per remote
...

# Verify the PMP network formed:
#   1. Power on the master and all remotes
#   2. On each remote: the orange RX LED lights tells the radio is synchronized and receiving valid packets from the master
#   3. The 3 blue RSSI LEDs show link strength
```

<!--
- [Sik NET ID configuration](https://docs.px4.io/main/en/data_links/sik_radio#configuration-instructions)
- [Microhard Point-to-Multipoint](https://docs.holybro.com/radio/microhard-radio/point-to-multipoint-setup-with-microhard-radio)
-->

## RC Input

RC IN is the [bottom-left 5-pin port on the Jetson Baseboard](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-jetson-baseboard/ports-pinout#rc-in-port)

Use it to connect an RC receiver (e.g., [Radiomaster R86C V2](https://radiomasterrc.com/products/r86c-receiver), [[user manual](https://cdn.shopify.com/s/files/1/0609/8324/7079/files/R86C.pdf)]) and bind it to an RC (e.g., [Radiomaster Boxer 4in1](https://radiomasterrc.com/products/boxer-radio-controller-m2?variant=46486352232640), [[user manual](https://cdn.shopify.com/s/files/1/0701/8066/7584/files/BOXER_A1.9.pdf)])

On the Radiomaster Boxer, press `MDL`, then `PAGE>`, on the `SETUP` page scroll to the `Internal RF` configuration:

```sh
Mode        MULTI
Type        FrSky X
Subtype     D16
Ch. Range   CH1-16
...
Failsafe    No pulses
```

Power on R86C receiver in [bind mode](https://cdn.shopify.com/s/files/1/0609/8324/7079/files/R86C.pdf) by holding the KEY button, on the RC, select `Receiver   [Bnd]` on the `SETUP` page
