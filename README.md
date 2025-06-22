# HUSKY ROS2 SETUP Documentation
## Installation and Initialisation of Ubuntu 22.04
1) Download the official server image of Ubuntu 22.04 [https://releases.ubuntu.com/jammy/]
2) Install Rufus/balenaEtcher to flash it to a bootable drive [https://etcher.balena.io/]
	1) Once you download the zip file, run `unzip balenaEtcher-linux-x64-x.x.x.zip`. 
	2) After extracting open the folder in terminal and run `balena-etcher`
	3) Flash the downloaded .iso into the pendrive/desired drive
3) Insert the bootable drive, start the Husky system and press Esc/F2/F9 to boot into the external drive. Go with `Try/Install Ubuntu server`. If it doesn't work open with the other option suggested (some kernel option)
4) Follow installation steps, and once it finishes, it boots into the freshly installed Ubuntu 22.04 server
5) To configure network, most importantly static IP addressing, create and edit a file by running `vim netplan.yaml`. Refer the attached netplan.yaml for reference. Since you are inside server edition, gedit or other GUI text editors will not work. Hence be careful with the indentation in vim.
		-> Important Note: In **Vim**, to start editing a file press `i` and once the edit is done, to save and quit the file first press `Esc` and then type `:wq`.
6) Now move this file into `/etc/netplan` by running `sudo mv netplan.yaml /etc/netplan/`
7) Now use `netplan` to check if the .yaml file is formatted right by running `sudo netplan try`. Resolve all the issues shown and then move to the next step.
8) Now run `sudo netplan apply` for the network configuration to take place.
9) If instructed, you might have to reboot the system using `sudo reboot`.
#### netplan.yaml
```
network:
  version: 2
  renderer: networkd
  ethernets:
    enp3s0:
      dhcp4: false
      addresses: [192.168.1.200/24]
#      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

  wifis:
    wlo1:
      dhcp4: false
      optional: true
      access-points:
        "wifi-username":
          password: "wifi-password"
      addresses: [192.168.0.200/24]
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      gateway4: 192.168.0.1
```

## ROS2 Installation
- Best resource, blindly trust and follow [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html]


## Husky A200 Physical Setup & Keyboard Teleop (ROS 2 Humble)
### 1. Prerequisites

Ensure you have:

- Ubuntu 22.04 (x86_64)
- ROS 2 Humble installed (desktop or base)
- `colcon`, `git`, and `rosdep` tools available

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

Also make sure to add the following lines in `~/.bashrc` file,
```bash
# Sourcing ROS2 Humble Hawksbill
source /opt/ros/humble/setup.bash

# Setting up colcon_cd
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/

# Setting up colcon autocomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# ROS2 Domain ID
export ROS_DOMAIN_ID=200    # 200 because we can match with the IP too, easy to remember
```

---

### 2. Install Clearpath Robot Software

Clearpath provides two key repositories for Husky:

#### 2.1 `clearpath_common`
High-level ROS 2 meta-packages (e.g., controllers, description, simulation).

```bash
mkdir -p ~/husky_ws/src
cd ~/husky_ws/src
git clone -b humble https://github.com/clearpathrobotics/clearpath_common.git
```

#### 2.2 `clearpath_robot`
Hardware-level drivers and bringup support.

```bash
cd ~/husky_ws/src
git clone -b humble https://github.com/clearpathrobotics/clearpath_robot.git
```

---

### 3. Build the Workspace

```bash
cd ~/husky_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Once built:

```bash
source ~/husky_ws/install/setup.bash
```

---

### 4. (Optional) Install Clearpath Debians

If you prefer Debian packages instead of building from source:

```bash
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main"   | sudo tee /etc/apt/sources.list.d/clearpath-latest.list
sudo apt update
sudo apt install ros-humble-husky-desktop ros-humble-husky-robot
```

---

### 5. Robot Bringup & Launch

The bringup of robot and launch involves a bit-more of steps as we do not have the config files loaded at `/etc` like we had earlier in the ROS1 system. Follow the below steps to achieve the desired configuration:
#### Clearpath Husky Setup — Required files at /etc/clearpath along with the directory structure

```
# Expected /etc/clearpath/ directory structure for ROS 2 Humble Husky

/etc/clearpath/
├── robot.yaml                          ← Required: Core robot configuration
├── setup.bash                          ← Generated environment helper

├── platform/
│   ├── config/
│   │   ├── control.yaml               ← Controller parameters
│   │   ├── imu_filter.yaml           ← EKF/IMU filter params
│   │   ├── localization.yaml         ← EKF/localization params
│   │   └── twist_mux.yaml            ← /cmd_vel multiplexer params
│   └── launch/
│       └── platform-service.launch.py ← Launches base platform nodes

├── sensors/
│   ├── config/
│   │   ├── camera_0.yaml             ← Sensor 0 parameters
│   │   └── camera_1.yaml             ← Sensor 1 parameters (if any)
│   └── launch/
│       ├── sensors-service.launch.py ← Launches all sensors
│       ├── camera_0.launch.py        ← Launch sensor_0 driver
│       └── camera_1.launch.py        ← Launch sensor_1 driver

├── manipulators/                      ← If a manipulator (arm) is defined
│   ├── config/
│   │   └── moveit.yaml              ← MoveIt-specific parameters
│   └── launch/
│       └── manipulators-service.launch.py ← Launches MoveIt and controllers

└── platform-extras/                   ← Optional, for extra platform nodes
    └── launch/
        └── platform-extras-service.launch.py ← Extra platform features

# Required files summary
- robot.yaml               → core configuration (serial, sensors, ROS2 settings)
- platform-service.launch.py    → launches base platform nodes/controllers
- sensors-service.launch.py     → launches all sensor nodes per robot.yaml
- manipulators-service.launch.py → launches MoveIt arm stack if defined
- config/*.yaml           → ROS parameter files generated from robot.yaml
- setup.bash              → (optional) environment setup script
```

But the catch here is that, all we need to create a `/etc/clearpath/robot.yaml` file and then run 

```
# Clearpath Generator Commands for ROS 2 Humble

# Make sure /etc/clearpath/robot.yaml exists before running these:

ros2 run clearpath_generator_common generate_bash /etc/clearpath/robot.yaml
ros2 run clearpath_generator_common generate_description /etc/clearpath/robot.yaml
ros2 run clearpath_generator_common generate_semantic_description /etc/clearpath/robot.yaml
ros2 run clearpath_generator_common generate_discovery_server /etc/clearpath/robot.yaml

ros2 run clearpath_generator_robot generate_launch /etc/clearpath/robot.yaml
ros2 run clearpath_generator_robot generate_param /etc/clearpath/robot.yaml

# After generation, bring up the robot:

source /etc/clearpath/setup.bash
ros2 launch clearpath_common bringup.launch.py
```

This populates `/etc/clearpath` with the necessary files and folders required in the directory.

```bash
source ~/husky_ws/install/setup.bash
ros2 launch clearpath_common bringup.launch.py
```

---
### 6. Integration with systemd services for launch on startup

Save these files under `/etc/systemd/system`

```
# clearpath-platform.service
[Unit]
Description=Clearpath Platform Service
After=network.target

[Service]
ExecStart=/bin/bash -c "source /etc/clearpath/setup.bash && ros2 launch /etc/clearpath/platform/launch/platform-service.launch.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target

# clearpath-sensors.service
[Unit]
Description=Clearpath Sensors Service
After=clearpath-platform.service

[Service]
ExecStart=/bin/bash -c "source /etc/clearpath/setup.bash && ros2 launch /etc/clearpath/sensors/launch/sensors-service.launch.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target

# clearpath-manipulators.service (if needed)
[Unit]
Description=Clearpath Manipulators Service
After=clearpath-sensors.service

[Service]
ExecStart=/bin/bash -c "source /etc/clearpath/setup.bash && ros2 launch /etc/clearpath/manipulators/launch/manipulators-service.launch.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target

# clearpath-robot.service (optional)
[Unit]
Description=Master orchestrator for all robot services
Requires=clearpath-platform.service clearpath-sensors.service clearpath-manipulators.service
After=network.target

[Service]
Type=oneshot
ExecStart=/bin/true
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target

```

You can break the above to individual `.service` files, and then run

```bash
sudo cp *.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable clearpath-robot.service
sudo systemctl start clearpath-robot.service
```

---

### 6. Keyboard Teleoperation

Install the teleop tool:

```bash
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
```

Run teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You’ll control the robot using keyboard keys that publish to `/cmd_vel`. Watch it move!

---

### 7. Diagnostics & Troubleshooting

- Check node status and topics:

```bash
ros2 node list
ros2 topic list
```

- Monitor logs:

```bash
sudo journalctl -u clearpath-robot -f
```

- If controllers aren't moving:
  - Verify the MCU is connected (USB câble, comms light green)
  - Ensure `/cmd_vel` is published and heard by the driver

---

