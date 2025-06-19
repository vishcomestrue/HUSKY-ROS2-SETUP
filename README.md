# HUSKY A200 ROS2 Humble Setup
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
