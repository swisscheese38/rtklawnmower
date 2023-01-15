# rtklawnmower

Work in progress. Inspired by https://openmower.de/

## Hardware

* Lawn Mower: ZSA Zuchetti Ambrogio L15 Deluxe
* Computer: Raspberry Pi 4 (8GB)
* Microcontroller: Arduino Nano
* Motor drivers: ZS-X11H V2
* GPS Devices: Ardusimple Simplertk2b
* IMU: MPU9250
* Battery: Einhell 18V 4Ah LI-ION Battery Pack
* Voltage Convertor: KIS3R33S DC-DC 7-24V to 5V USB Step-Down
* Some resistors (1 kOhm, 4.7 kOhm), transistors (2N2222) and cables

## Software

* Ubuntu 20.04 Server
* ROS1
* Rover Packages: `git unzip ros-noetic-ros-base gpsd gpsd–clients python-gps rtklib build-essential ros-noetic-controller-manager ros-noetic-joint-state-controller ros-noetic-serial ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-diff-drive-controller ros-noetic-teleop-twist-keyboard i2c-tools`
* Development Node Packages: `git ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential ros-noetic-serial`

## Installation

### Motor drivers

We are using some cheap ZS-X11H V2 motor drivers to interact with the BLDC motors from the mower. The HAL encoders need to be pulled up high for the motor driver to be able to read the position values. Refer to [this guide](https://www.digikey.no/no/blog/using-bldc-hall-sensors-as-position-encoders-part-3) to get more information. I used 4.7 kOhm resistors. The direction has to be controlled by connecting a pin to ground. As this cannot be achieved with an Arduino out of the box, I used a 2N2222 transistor and a 1 kOhm resistor as suggested in [this forum entry](https://forums.raspberrypi.com/viewtopic.php?t=335218).

### IMU
We are using a MPU9250 for getting heading and acceleration information. The needed ROS packages are already added as Git submodules and will be built as part of the catkin_make build. The IMU is connected over I2C and the GIPO pins. Unfortunately Debian 20.04 in its packages currently supplies a version of pigpio that does not include `pigpio.h`. Therefore we have to download and build the library ourselves. For this you can follow the steps described [here](http://abyz.me.uk/rpi/pigpio/download.html).

### GPS

The Simplertk2b is going to be connected to the Raspberry Pi through a serial connection (using the provided Arduino Serial Shield by Ardusimple). For this to work, serial needs to be enabled by adding the line `enable_uart=1` to `/boot/firmware/syscfg.txt`. As we don't want to login through serial we remove the part `console=serial0,115200` from `/boot/firmware/cmdline.txt`. Disable serial Getty by `sudo systemctl stop serial-getty@ttyS0.service` and `sudo systemctl disable serial-getty@ttyS0.service`.

Furthermore Bluetooth needs to be disabled. This can be done by adding another line `dtoverlay=disable-bt` to the file `/boot/firmware/usercfg.txt`.

Unfortuunately, as soon as the Simplertk2b is connected, the Pi will no longer boot. When hooking it up to a screen through HDMI you see that it is stuck before booting into Ubuntu. This is because the messages from the GPS module coming thrugh serial now interrupt the autoboot countdown. Therefore you need to adjust uboot as described here https://raspberrypi.stackexchange.com/questions/116074/how-can-i-disable-the-serial-console-on-distributions-that-use-u-boot/117950#117950.

After rebooting you should now see (scrambled) messages coming in from the GPS device when you look at `less -f /dev/ttyAMA0`.

We are going to configure `gpsd` to provide the GPS data. For this to work `/etc/default/gpsd` has to be adjusted for `DEVICES="/dev/ttyAMA0"` and `GPSD_OPTIONS="-s 38400"` (select the serial speed baud rate that your ublox is currently configured). Afterwards restart the deamon with `sudo systemctl restart gpsd.socket`. Then finally you should see some GPS data when you open up `cgps`.

In order for us to provide correction data to the rover, we are using a free NTRIP caster. Later we will add our own base station. The connection is done from the Raspberry Pi through USB to the second USB port of the Simplertk2b board that connects to the XBee socket, where we connect the RX/TX-pins according to https://youtu.be/qlkN70bBfFQ. Additionally, the Baudrate for UART1 is also increased to 115200 in Ucenter under UBX->CFG->PRT. The F9P configuration can be found in [gnss-rover.txt](gnss-rover.txt).

Afterwards, the configuration in `/etc/default/gpsd` has to be ajusted to reflect the changed baudrate `GPSD_OPTIONS="-s 115200"`. Restart the deamon with `sudo systemctl restart gpsd.socket` and you should again be able to see position data when you look at `cgps`. You will also notice that the rate of data coming in is much higher now as we increased it from 1Hz (factory setting) to 10Hz.

For sending the correction data, we use str2str, which doesn't have to be compiled ourselves but can simply be installed as an Ubuntu package with `sudo apt install rtklib`. As we want to have the data streamed to the Simplertk2b board at all times, we will install `str2str` as a systemd servie. Have a look at [str2str.service](str2str.service) and then install it as follows:
```
sudo cp str2str.service /lib/systemd/system/.
sudo systemctl start str2str.service
sudo systemctl enable str2str.service
```

### ROS

Install ROS according to [http://wiki.ros.org/Installation/Ubuntu]. Make sure you have sourced your ros installation with `source /opt/ros/noetic/setup.bash` and then Catkin make your cloned workspace
```
cd /home/vboxuser/git/rtklawnmower/catkin_ws/
catkin_make
```
Then also source your workspace `source ~/git/rtklawnmower/catkin_ws/devel/setup.bash`.
