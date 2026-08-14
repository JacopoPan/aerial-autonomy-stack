# Setup Chrony

> Follow these steps to let the Jetsons synchronize their time to the `ground-image` computer over `AIR_SUBNET` when without internet connectivity (for Zenoh, etc.)

## On the Ground Station Computer

Install `chrony` (on the host operating system):
```sh
sudo apt update && sudo apt install chrony -y
sudo nano /etc/chrony/chrony.conf
```

Modify `chrony.conf`:
```sh
# ... [Standard Ubuntu defaults, add at the end of the file] ...

# Allow the Jetsons on the Doodle Labs AIR_SUBNET (10.223) to query this computer for time
allow 10.223.0.0/16
# If this computer loses internet connectivity, declare itself as a valid master clock
local stratum 10
```

Restart `chrony`:
```sh
sudo systemctl restart chrony
```

## On each Jetson

Install `chrony`  (on the host operating system):
```sh
sudo apt update && sudo apt install chrony -y
sudo gedit /etc/chrony/chrony.conf
```

Modify `chrony.conf`:
```sh
# ... [Standard Ubuntu defaults, find the line below] ...
# Add `prefer` to favor the internet when available
pool ntp.ubuntu.com iburst maxsources 4 prefer
# ... [More Ubuntu defaults, add at the end of the file] ...

# Use the ground laptop on the Doodle Labs AIR_SUBNET (10.223.90.101) as a time source, if the internet is not available
server 10.223.90.101 iburst
```

Restart `chrony`:
```sh
sudo systemctl restart chrony
sudo chronyc makestep
```

On Jetson, check with:
```sh
chronyc -n sources # Note it might take some time for the Jetson to switch source
```

If `[AIR_SUBNET].90.101` has a `^*` next to it, the Jetson is syncing to the ground computer

If an internet server has a `^*` next to it, the Jetson is syncing to the internet

If `[AIR_SUBNET].90.101` has a + or - next to it, the Jetson sees the ground computer as backup for timesync
