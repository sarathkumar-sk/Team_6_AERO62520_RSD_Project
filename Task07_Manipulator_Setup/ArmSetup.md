# myCobot 280 Pi Documentation

## Table of Contents
1. [Key Specifications](#key-specifications)
2. [Performance & Structural Parameters](#performance--structural-parameters)
3. [Electronic Parameters & Interfaces](#electronic-parameters--interfaces)
4. [Kinematic Modeling (DH Parameters)](#kinematic-modeling-dh-parameters)
5. [Technical Diagram](#technical-diagram)
6. [Setup Guide](#setup-guide)
   - [Unboxing & Assembly](#unboxing--assembly)
   - [First Boot Steps](#first-boot-steps)
   - [Setting up a Service for Consequent Boots](#setting-up-a-service-for-consequent-boots)
     - [Create a Headless Launch Script](#1-create-a-headless-launch-script)
     - [Create a systemd Service File](#2-create-a-systemd-service-file)
     - [Enable and Start the Service](#3-enable-and-start-the-service)
     - [Reboot and Test](#4-reboot-and-test)
7. [Cloning and Building the Elephant Arm Packages](#cloning-and-building-the-elephant-arm-packages)
   - [Create the Workspace and Clone the Repository](#1-create-the-workspace-and-clone-the-repository-in-your-main-machine-intel-nuc)
   - [Install Dependencies](#2-install-dependencies)
   - [Build the Workspace](#3-build-the-workspace)
   - [Source the Workspace](#4-source-the-workspace)
   - [Set the ROS_DOMAIN_ID on your system](#5-set-the-ros_domain_id-on-your-system)
   - [Launch MoveIt2 Demo](#6-launch-moveit2-demo)
8. [Installing mycobot 280pi with ubuntu 24.04 and ros jazzy](#installing-mycobot-280pi-with-ubuntu-2404-and-ros-jazzy)
---

## Key Specifications

The **myCobot 280 Pi 2023** is a lightweight 6DOF manipulator with an in-built Raspberry Pi for communication and running ROS2 nodes.

<p align="center">
  <a href="http://www.youtube.com/watch?v=sY7ScSSkyfU" title="Video Title">
    <img src="http://img.youtube.com/vi/sY7ScSSkyfU/0.jpg" alt="Newest Addition To Our Tech Stack! MyCobot 280 Pi From @ElephantRobotics"/>
  </a>
</p>


## Performance & Structural Parameters

- **Model:** myCobot 280 Raspberry Pi
- **Operating System:** Ubuntu Mate 20.04
- **Microprocessor:** Raspberry Pi 4B (1.5GHz quad-core)
- **Degrees of Freedom:** 6
- **Payload:** 250g
- **Effective Working Radius:** 280mm
- **Net Weight:** 860g
- **Repeated Positioning Precision:** ±0.5mm
- **Communication:** Type-C
- **Joint Angles:** Joints J1-J5: -165° to +165°, J6: -175° to +175°
- **Power:** 12V, 5A DC charger

---

## Electronic Parameters & Interfaces

- **System on Chip (SOC):** Broadcom BCM2711
- **CPU:** 64-bit 1.5GHz quad-core
- **USB:** 2x USB3.0, 2x USB2.0
- **HDMI:** 2x microHDMI interfaces (port 2 recommended)
- **Connectivity:** Bluetooth, wireless, Ethernet
- **Pedestal Ports:**
  - **Front:** Power Switch, GPIO Pins, USB ports, DC power, Ethernet
  - **Side:** SD card slot, Type C, HDMI
- **End-effector Interfaces:**
  - Servo Interface for grippers
  - Grove Interface (GND, 5V, G26, G32)
  - Functional Interface Group 2 (5V, GND, 3.3V, G22, G19, G23, G33)
  - Type C for PC communication/firmware updates
  - **Atom:** 5x5 RGB LED (G27), key function (G39)

---

## Kinematic Modeling (DH Parameters)

The robot arm uses standard DH parameters for kinematic modeling:

| Joint (j) | theta | d (mm) | a (mm) | alpha (rad) | offset (rad) |
| :-------- | :---- | :----- | :----- | :---------- | :----------- |
| 1         | q1    | 131.22 | 0      | 1.5708      | 0            |
| 2         | q2    | 0      | -110.4 | 0           | -1.5708      |
| 3         | q3    | 0      | -96    | 0           | 0            |
| 4         | q4    | 63.4   | 0      | 1.5708      | -1.5708      |
| 5         | q5    | 75.05  | 0      | -1.5708     | 1.5708       |
| 6         | q6    | 45.6   | 0      | 0           | 0            |

---

## Technical Diagram

<p align="center">
  <img src="../Images/Manipulator/image.png" alt="myCobot 280 Pi Technical Diagram" width="500"/>
</p>

---

## Setup Guide

### Unboxing & Assembly

1. **Base Plate Preparation:**  
   Place four clips in the corners of the base plate as shown.  
   <p align="center">
     <img src="../Images/Manipulator/Image1.jpeg" alt="Base Plate Clips" width="350"/>
   </p>

2. **Mount Manipulator:**  
   Mount the manipulator on the base plate. Hold the manipulator during this process; the base plate alone cannot support its weight. Listen for a "click" when mounted.  
   <p align="center">
     <img src="../Images/Manipulator/Image2.jpeg" alt="Mount Manipulator" width="350"/>
     <img src="../Images/Manipulator/Image3.jpeg" alt="Mount Manipulator 2" width="350"/>
   </p>

3. **Fix to Surface:**  
   Clamp or use the suction pads, to steady the manipulator to the table before operation.  
   <p align="center">
     <img src="../Images/Manipulator/Image4.jpeg" alt="Clamp" width="350"/>
   </p>

4. **Connect Peripherals:**  
   For the first boot, connect a keyboard, mouse, and microHDMI cable.  
   <p align="center">
     <img src="../Images/Manipulator/Image5_1.jpeg" alt="Connect Peripherals" width="350"/>
   </p>

5. **Power On:**  
   Ensure markings are aligned, then power on the robot. The servos will activate after a few seconds.  
   <p align="center">
     <img src="../Images/Manipulator/Image6.jpeg" alt="Power On" width="350"/>
   </p>

---

### First Boot Steps
> [!WARNING]
> Users are experiencing mixed success when issues connecting to the network, causes by routing issues.
> If you are unable to ping, please the 'Set up static IPs' section as an alternative. 

1. **Connect to Network:**  
   Connect the arm's Raspberry Pi to Wi-Fi or Ethernet.  
   <p align="center">
     <img src="../Images/Manipulator/Image7.png" alt="Network" width="500"/>
   </p>

2. **Get IP Address:**  
   Check `wlan0` (Wi-Fi) or `eth0` (Ethernet) for the IP address (e.g., 192.168.1.193).  
   <p align="center">
     <img src="../Images/Manipulator/Image8.png" alt="IP Address" width="500"/>
   </p>

3. **SSH Access:**  
   SSH from your Intel NUC to the arm:  
   ```bash
   ssh er@IP_ADDRESS_OF_ARM
   ```
   Password: `Elephant` (case sensitive)  
   <p align="center">
     <img src="../Images/Manipulator/image-1.png" alt="SSH" width="700"/>
   </p>

4. **ROS2 Environment Setup:**  
   We can automate sourcing our ROS workspace by appending instructions to **the end of** the `.bashrc` script.

   ```
   nano ~/.bashrc
   ```
   Scorll down to the bottom of the file and add:
   Ensure you replace `YOUR_GROUP_NUMBER' with an int value.
   ```bash
   # Source ROS Galactic setup with error checking
   if source /opt/ros/galactic/setup.bash; then
     echo "Sourced /opt/ros/galactic/setup.bash successfully"
   else
     echo "Failed to source /opt/ros/galactic/setup.bash"
   fi
   
   # Source your workspace setup with error checking
   if source ~/colcon_ws/install/setup.bash; then
     echo "Sourced ~/colcon_ws/install/setup.bash successfully"
   else
     echo "Failed to source ~/colcon_ws/install/setup.bash"
   fi
   
   # Export and print ROS_DOMAIN_ID
   export ROS_DOMAIN_ID=YOUR_GROUP_NUMBER
   echo "ROS_DOMAIN_ID is set to $ROS_DOMAIN_ID"
   echo "To change this automation, use nano to edit ~/.bashrc and the source ~/.bashrc to apply."
   ```
   Exit nano (CTRL+X) and save.

   After saving, apply the changes by running:
   ```bash
   source ~/.bashrc
   ```
   This ensures your environment variables.
6. **Test Servos:**  
   Launch the slider test:
   ```bash
   ros2 launch mycobot_280pi slider_control.launch.py
   ```
> [!WARNING]
> Avoid using the `Randomize` button. This interface does not contrain the manipulator. Random joint configurations may cause the arm to attempt to move through the table or other objects.
   <p align="center">
     <img src="../Images/Manipulator/image-2.png" alt="Slider Test" width="700"/>
   </p>

---

> [!CAUTION]
> The section is a currently under development.
### Set up static IPs (Alternative)
The aim of this sectiois to set up the laptop/NUC with 192.168.12.1/24 as it's IP address and the Manipulator's raspberry pi with 192.168.12.2/24 as it IP address on a local network.

   Physically connect your manipulator arm and intel NUC/laptop by ethernet cable
   On both machines run:
   ```
   ip link
   ```
   On the NUC/Laptop we typically expect `enp0s31f6` and on the Manipulator's pi typically we expect `eth0`. This guide assumes this, if your devices differ, ammend as required. 

**Configure static IPs**
In Laptop/NUC terminal
```
sudo nano /etc/netplan/99-wired-static.yaml
```
> [!NOTE]
> Context: Named 99 so doesn't override any defaults and is applied last so guarantees priority 
   
   Paste the below into the nano editor (note the ip address)
   ```
   network:
     version: 2
     renderer: NetworkManager
     ethernets:
       enp0s31f6:
         dhcp4: false
         addresses:
           - 192.168.12.1/24
   ```
   Exit (CTRL+X) and save.
   
   Apply the config
   ```
   sudo chmod 600 /etc/netplan/99-wired-static.yaml
   sudo netplan apply
   ```
   > [!NOTE]
   > Without the chmod 600 command, netplan with throw a warning that the config is too open. On an isolated network this isn't a worry, but useful to store somewhere in your brain. chmod 600 specifies that only the owner can read and write the file, but suppresses the warning.
   
   Verify  
   ```
   ip addr show enp0s31f6
   ```
   
   On the Manipulator
   ```
   sudo nano /etc/netplan/99-wired-static.yaml
   ```
  Paste the below into the nano editor (note the ip address)
   ```
   network:
     version: 2
     ethernets:
       eth0:
         dhcp4: false
         addresses:
           - 192.168.12.2/24

   ```
   Exit (CTRL+X) and save.
   
   Apply the config
   ```
   sudo chmod 600 /etc/netplan/99-wired-static.yaml
   sudo netplan apply
   ```
   Verify
   ```
   ip addr show eth0
   ```
   **Check connection**
   On NUC/Laptop ping the Pi
   ```
   ping -c 3 192.168.12.2
   ```
   On the Pi ping the NUC/Laptop
   ```
   ping -c 3 192.168.12.1
   ```
   **You can now SSH**
   On the NUC:
   ```
   ssh -X er@192.168.12.2
   ```
   Password for the Manipulator's arm is `Elephant`, case sensative.
   The `-X` flag enables X11 forwarding when connecting, useful if you want to visualise the `rqt_graph`.
> [!CAUTION]
> The contruction zone has ended.

---

### Setting up a Service for Consequent Boots

#### 1. Create a Headless Launch Script

SSH into the Pi and create the script:
```bash
nano ~/start_mycobot.sh
```
Paste:
```bash
#!/bin/bash
source /opt/ros/galactic/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 run mycobot_280pi slider_control
```
Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).  
Make it executable:
```bash
chmod +x ~/start_mycobot.sh
```

#### 2. Create a systemd Service File

Create `/etc/systemd/system/mycobot.service`:
`sudo nano /etc/systemd/system/mycobot.service`
```
[Unit]
Description=MyCobot ROS 2 Launch
After=network.target

[Service]
Type=simple
User=er
WorkingDirectory=/home/er
ExecStart=/home/er/start_mycobot.sh
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

#### 3. Enable and Start the Service

```bash
sudo systemctl daemon-reload
sudo systemctl enable mycobot.service
sudo systemctl start mycobot.service
```

#### 4. Reboot and Test

Reboot the Pi:
```bash
sudo reboot
```
Check service status:
```bash
systemctl status mycobot.service
```
<p align="center">
  <img src="../Images/Manipulator/image-3.png" alt="Slider Test" width="700"/>
</p>

> [!TIP]
> You could also add a check to the `.bashrc` script on the Manipulator's pi
>```
>echo "Checking MyCobot service status..."
>systemctl is-active --quiet mycobot.service && echo "MyCobot service is running." || echo "MyCobot service is NOT running."
>```
---
## Cloning and Building the Elephant Arm Packages

Follow these steps to clone the required packages and set up your workspace:

### 1. Create the Workspace and Clone the Repository in your main machine (Intel NUC)

```bash
mkdir -p ~/elephant_arm_ws
cd ~/elephant_arm_ws
git clone https://github.com/UoMMScRobotics/MSc-manipulator-task.git
mv MSc-manipulator-task src
```

### 2. Install Dependencies

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-resources-panda-moveit-config
sudo apt install ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller
```

### 3. Build the Workspace

```bash
colcon build
```

### 4. Source the Workspace

```bash
source install/setup.bash
```
### 5. Set the ROS_DOMAIN_ID on your system
Repeat the ROS_DOMAIN_ID setup on your own machine to ensure network isolation between teams.  
Add the following line to your `~/.bashrc`, replacing `YOUR_GROUP_NUMBER` with your assigned number:

```bash
export ROS_DOMAIN_ID=6
```

After saving, run `source ~/.bashrc` or open a new terminal to apply the change. This ensures only your team can communicate with your robot arm.

### 6. Change runscript.sh

```bash
#!/bin/bash
sshpass -p "Elephant" ssh -X -tt er@192.168.12.2 << EOF
source /opt/ros/jazzy/setup.bash
source ~/elephant_arm_ws/install/setup.bash
ros2 launch mycobot_280pi slider_control.launch.py gui:=false
EOF
```

### 7. Launch MoveIt2 Demo

```bash
ros2 launch mycobot_280_moveit2 demo.launch.py
```
<p align="center">
  <img src="../Images/Manipulator/image-4.png" alt="MoveIt2 Demo" width="700"/>
</p>

Familiarise yourself with the moveit2 simulation before it is tested on the actual robot. This can be done by varying the sliders to see how the robot moves and then testing it with the actual robot.

---
## Installing mycobot 280pi with ubuntu 24.04 and ros jazzy
## Setup Guide

### Flash the SD Card

As shared on MS Teams, download the `.img` file. Remove the SD card from the base of the manipulator.

We will use the build in Disks utility to flash the image onto the SD card.

<p align="center">
    <img title="disk1" src="https://github.com/UoMMScRobotics/maniupulator-jazzy/blob/4d775245b05734a16eab930f339125fcebc825c5/Images/disks.png" width="60%"/>
</p>
<p align="center">
    <img title="disk2" src="https://github.com/UoMMScRobotics/maniupulator-jazzy/blob/4d775245b05734a16eab930f339125fcebc825c5/Images/disks_flash.png" width="60%"/>
</p>
<p align="center">
    <img title="disk3" src="https://github.com/UoMMScRobotics/maniupulator-jazzy/blob/4d775245b05734a16eab930f339125fcebc825c5/Images/disks_flash_2.png" width="60%"/>
</p>

Once the SD card is flashed re-insert back into the manipulator. 

> [!IMPORTANT]
> Downloading and flashing the image can take a long time. Why not reach the Understanding and [Troubleshooting ROS 2 Networking and Communication Guide](https://github.com/UoMMScRobotics/UOMDocumentationForLeoRover/blob/main/Further_reading/Networking.md) while you wait?

### First Boot Steps

This tutorial sets up a basic peer to peer wired network between the pi and another device (such as your NUC or laptop). This is to get you going, you will need to revise this set up when you consider your system architecture.

> [!WARNING]
> If previously you've set up up static IP addresses using `99-wired-static.yaml` then remove it using `sudo rm /etc/netplan/99-wired-static.yaml`, then `sudo netplan apply` to apply the change.

The pi will boot into the default user `elephant` which uses the password `trunk`. The pi has a static IP address, `10.3.14.59` (a little π humour).

Check the network on the pi, it should be set up as below.
<p align="center">
    <img title="Network" src="https://github.com/UoMMScRobotics/maniupulator-jazzy/blob/4d775245b05734a16eab930f339125fcebc825c5/Images/Network.png" width="60%"/>
</p>
<p align="center">
    <img title="Network" src="https://github.com/UoMMScRobotics/maniupulator-jazzy/blob/4d775245b05734a16eab930f339125fcebc825c5/Images/Network_mani_pi.png" width="60%"/>
</p>

**ROS2 Environment Setup on the pi:**  
We can automate sourcing our ROS workspace by appending instructions to **the end of** the `.bashrc` script.

 ```
 nano ~/.bashrc
 ```
 Scorll down to the bottom of the file and add:
 Ensure you replace `YOUR_GROUP_NUMBER' with an int value.
 ```bash
 # Source ROS Jazzy setup with error checking
if source /opt/ros/jazzy/setup.bash; then
  echo "Sourced /opt/ros/jazzy/setup.bash successfully"
else
  echo "Failed to source /opt/ros/jazzy/setup.bash"
fi

# Explicit setting of DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "DDS set to $RMW_IMPLEMENTATION"

# Source your workspace setup with error checking
if source ~/colcon_ws/install/setup.bash; then
  echo "Sourced ~/colcon_ws/install/setup.bash successfully"
else
  echo "Failed to source ~/colcon_ws/install/setup.bash"
fi

# Export and print ROS_DOMAIN_ID
export ROS_DOMAIN_ID=6
echo "ROS_DOMAIN_ID is set to $ROS_DOMAIN_ID"

# ROS_LOCALHOST_ONLY is being fazed out but is good for a quick set up
export ROS_LOCALHOST_ONLY=0
echo "ROS_LOCALHOST_ONLY is set to $ROS_LOCALHOST_ONLY"

echo "To change this automation, use nano to edit ~/.bashrc and the source ~/.bashrc to apply."
 ```
 Exit nano (CTRL+X) and save.

 After saving, apply the changes by running:
 ```bash
 source ~/.bashrc
 ```

**Test Servos:**  
 Launch the slider test:
 ```bash
 ros2 launch mycobot_280pi slider_control.launch.py
 ```
> [!WARNING]
> Avoid using the `Randomize` button. This interface does not contrain the manipulator. Random joint configurations may cause the arm to attempt to move through the table or other objects.
---


**Laptop/NUC setup**  

> [!WARNING]
> If previously you've set up up static IP addresses using `99-wired-static.yaml` then remove it using `sudo rm /etc/netplan/99-wired-static.yaml`, then `sudo netplan apply` to apply the change.
> 
Now let's set up your laptop/NUC for our peer to peer wired network. Ensure you have ROS Jazzy installed on the device and you have the ethernet cable plugged into both your device and the raspberry pi. 
Again we can automate sourcing our ROS workspace by appending instructions to **the end of** the `.bashrc` script.
Ensure you replace `YOUR_GROUP_NUMBER' with an int value.
```
# Source ROS Jazzy setup with error checking
if source /opt/ros/jazzy/setup.bash; then
  echo "Sourced /opt/ros/jazzy/setup.bash successfully"
else
  echo "Failed to source /opt/ros/jazzy/setup.bash"
fi

# Delcare the DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "Explicilty set fast DDS"


# Export and print ROS_DOMAIN_ID
export ROS_DOMAIN_ID=6
echo "ROS_DOMAIN_ID is set to $ROS_DOMAIN_ID"
echo "To change this automation, use nano to edit ~/.bashrc and the source ~/.bashrc to apply."

# For quick setup use ROS_LOCALHOST_ONLY, do revise later
export ROS_LOCALHOST_ONLY=0
echo "ROS_LOCALHOST_ONLY set to $ROS_LOCALHOST_ONLY"
```
 Exit nano (CTRL+X) and save.

 After saving, apply the changes by running:
 ```bash
 source ~/.bashrc
 ```

**Test the set up**

On the raspberry pi run: `ros2 launch mycobot_280pi slider_control.launch.py`. Leave that running and turn your attention to your other device and run: `ros2 topic list`. You should be able to see the topics running on your arm from your NUC/Laptop.

**Remote Access**

You are not required to use a monitor and pherierals everytime you want to use your raspberry pi. You can remove access the device, in this set up we use the static IP and can SSH.
```
ssh elephant@10.3.14.59
```
The password being `trunk`.
Any issues with SSH please see troubleshooting.

### Troubleshooting

* General check
   * In a multi system setup ensure the DDS is being used, check by runing `ros2 doctor --report` on all machines

* Find out more with verbose 
   ```
   ssh -vvv elephant@10.3.14.59
   ```

* Issues with `known_hosts` try:
   ```
   ssh-keygen -R 10.3.14.59
   ```
* Check the device you're trying to SSH from has the SSH client
   ```
   ssh -V
   #sudo apt install openssh-client #if needed
   ```

* Check the device you're trying to SSH into has the SSH server
```
# Check if the SSH server (sshd) service is running
sudo systemctl status ssh

# Start the SSH server immediately (if it's installed but not running)
sudo systemctl start ssh

# Enable the SSH server to start automatically at boot
sudo systemctl enable ssh

# Install the OpenSSH server package (if it's not already installed)
sudo apt update && sudo apt install -y openssh-server
```

* Issues with hanging when trying to SSH, or Black GUI with Gazebo.
   * Please the [Troubleshooting ROS 2 Networking and Communication Guide](https://github.com/UoMMScRobotics/UOMDocumentationForLeoRover/blob/main/Further_reading/Networking.md), where it will explain the background context and provide a fix.


