# rtklawnmower

Work in progress. Inspired by https://openmower.de/

## Hardware

* Lawn Mower: ZSA Zuchetti Ambrogio L15 Deluxe
* Computer: Raspberry Pi 4 (8GB)
* Microcontroller: Arduino Nano
* Motor drivers: ZS-X11H V2
* GPS Devices: Ardusimple Simplertk2b
* IMU: BNO055
* Battery: Einhell 18V 4Ah LI-ION Battery Pack
* Voltage Convertor: KIS3R33S DC-DC 7-24V to 5V USB Step-Down
* Some resistors (1 kOhm, 4.7 kOhm), transistors (2N2222) and cables

## Software

* Ubuntu 20.04 Server
* ROS1
* Installed Packages: `git unzip ros-noetic-ros-base gpsd gpsd–clients python-gps rtklib build-essential ros-noetic-controller-manager ros-noetic-joint-state-controller ros-noetic-serial ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-diff-drive-controller ros-noetic-teleop-twist-keyboard i2c-tools ros-noetic-gpsd-client ros-noetic-gps-common ros-noetic-robot-localization ros-noetic-move-base python3-pip`

## Installation

### udev rules for USB devices

To ensure that we have constant device names that we can use to access the different devices over USB, we create some udev rules. First we plugin the different devices one by one and identify vendor and product IDs for each of them by typing `lsusb` in between:

```
...
Bus 001 Device 007: ID 0403:6015 Future Technology Devices International, Ltd Bridge(I2C/SPI/UART/FIFO)
Bus 001 Device 008: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter
...
```

Then we can create a rule file `sudo vi /etc/udev/rules.d/91-usbdevices.rules` with the following content (adjust your idVendor and idProduct if necessary):

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="usb-ardusimple"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="usb-nano"
```

Have the rules applied by typing `sudo udevadm trigger` and then check that the symlinks were actually created by issuing `ls -l /dev/ttyUSB* /dev/usb-*`:

```
crw-rw---- 1 root dialout 188, 0 Apr  2 07:07 /dev/ttyUSB0
crw-rw---- 1 root dialout 188, 1 Apr  2 07:07 /dev/ttyUSB1
lrwxrwxrwx 1 root root         7 Apr  2 07:07 /dev/usb-ardusimple -> ttyUSB0
lrwxrwxrwx 1 root root         7 Apr  2 07:07 /dev/usb-nano -> ttyUSB1
```

### Motor drivers

We are using some cheap ZS-X11H V2 motor drivers to interact with the BLDC motors from the mower. The HAL encoders need to be pulled up high for the motor driver to be able to read the position values. Refer to [this guide](https://www.digikey.no/no/blog/using-bldc-hall-sensors-as-position-encoders-part-3) to get more information. I used 4.7 kOhm resistors. The direction has to be controlled by connecting a pin to ground. As this cannot be achieved with an Arduino out of the box, I used a 2N2222 transistor and a 1 kOhm resistor as suggested in [this forum entry](https://forums.raspberrypi.com/viewtopic.php?t=335218).

### GPS Base

If you live close enough to a reliable publicly available base station, you don't necessarily need to do run your own GPS base station.

But for our setup we are going to install one ourselves. The Simplertk2B is transmitting its data to the free NTRIP Caster rtk2go.com so it can not only be used by myself but also by others that live close enough. You can use any ESP32 and flash it with [the same firmware](https://github.com/nebkat/esp32-xbee) that Ardusimple is also using for their WiFi NTRIP Master ESP32.

If you want to achieve sub-centimeter accuracy also for absolute positions you can use one of the free PPP services for post processing your exact location. For this you first upload the `RAW data (PPK) over UART1 & USB at 1Hz` configuration from [Ardusimple's provided configuration files](https://www.ardusimple.com/configuration-files/). You start up u-center and record raw data to a file for 24 hours. Then you convert the `.ubx` file to a RINEX `.obs` file and upload it to [Canadian Spatial Reference System Precise Point Positioning (CSRS-PPP)](https://webapp.csrs-scrs.nrcan-rncan.gc.ca/geod/tools-outils/ppp.php). You will then receive a report with the very precise position of your base station's antenna. You can now upload the `Base` configuration from [Ardusimple's provided configuration files](https://www.ardusimple.com/configuration-files/) and then adjust the survey in method from `survey in` to `fixed` in u-center under `UBX->CFG->TMODE3` together with your precise coordinates. Don't forget to persist your changes under `UBX->CFG->CFG`. There's a great [tutorial from Sparkfun](
https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station/) that helped me a lot. The F9P configuration (with my specific location) can be found in [gnss-base.txt](gnss-base.txt).

### GPS Rover

The Simplertk2b is going to be connected to the Raspberry Pi through a serial connection (using the provided Arduino Serial Shield by Ardusimple). For this to work, serial needs to be enabled by adding the line `enable_uart=1` to `/boot/firmware/syscfg.txt`. As we don't want to login through serial we remove the part `console=serial0,115200` from `/boot/firmware/cmdline.txt`. Disable serial Getty by `sudo systemctl stop serial-getty@ttyS0.service` and `sudo systemctl disable serial-getty@ttyS0.service`.

Furthermore Bluetooth needs to be disabled. This can be done by adding another line `dtoverlay=disable-bt` to the file `/boot/firmware/usercfg.txt`.

Unfortuunately, as soon as the Simplertk2b is connected, the Pi will no longer boot. When hooking it up to a screen through HDMI you see that it is stuck before booting into Ubuntu. This is because the messages from the GPS module coming through serial now interrupt the autoboot countdown. Therefore we don't want to use u-boot anymore for bootloader but start into Ubuntu directly. As suggested in [Ubuntu's wiki for the Raspberry Pi](https://wiki.ubuntu.com/ARM/RaspberryPi#Change_the_bootloader) we do so by commenting out the `device_tree_address` section in `/boot/firmware/config.txt` and by exchanging the line `kernel=uboot_rpi_4.bin` for `kernel=vmlinuz` and by adding the line `initramfs initrd.img followkernel` just below.

After rebooting you should now see (scrambled) messages coming in from the GPS device when you look at `less -f /dev/ttyAMA0`.

We are going to configure `gpsd` to provide the GPS data. For this to work `/etc/default/gpsd` has to be adjusted for `DEVICES="/dev/ttyAMA0"` and `GPSD_OPTIONS="-s 38400"` (select the serial speed baud rate that your ublox is currently configured). Afterwards restart the daemon with `sudo systemctl restart gpsd.socket`. Then finally you should see some GPS data when you open up `cgps`.

The connection is done from the Raspberry Pi through USB to the second USB port of the Simplertk2b board that connects to the XBee socket, where we connect the RX/TX-pins according to [this guide on Youtube](https://youtu.be/qlkN70bBfFQ). Additionally, the Baudrate for UART1 is also increased to 115200 in u-center under UBX->CFG->PRT. The F9P configuration can be found in [gnss-rover.txt](gnss-rover.txt).

Afterwards, the configuration in `/etc/default/gpsd` has to be adjusted to reflect the changed baudrate `GPSD_OPTIONS="-s 115200"`. Restart the daemon with `sudo systemctl restart gpsd.socket` and you should again be able to see position data when you look at `cgps`. You will also notice that the rate of data coming in is much higher now as we increased it from 1Hz (factory setting) to 10Hz.

For sending the correction data, we use str2str, which doesn't have to be compiled ourselves but can simply be installed as an Ubuntu package with `sudo apt install rtklib`. As we want to have the data streamed to the Simplertk2b board at all times, we will install `str2str` as a systemd service. Create a file `sudo vi /lib/systemd/system/str2str.service` with the following content:

```
[Unit]
Description=Stream RTCM correction messages from our base station to serial USB
After=multi-user.target
StartLimitIntervalSec=300
StartLimitBurst=5

[Service]
ExecStart=str2str -in ntrip://firstname.lastname-at-gmail.com@rtk2go.com:2101/CHE-Bern-Krauchthal#rtcm3 -out serial://usb-ardusimple:115200:8:n:1:
User=pi
RestartSec=10
Restart=always

[Install]
WantedBy=multi-user.target
```

Then install the service as follows:
```
sudo systemctl daemon-reload
sudo systemctl start str2str.service
sudo systemctl enable str2str.service
```

The two blue LEDs labelled `XBEE>GPS` and `GPS>XBEE` should now begin to flash, indicating that the Simplertk2b board is receiving correction messages. And after you position the rover with clear sight of the sky then the blue LED `NO RTK` should turn off after a while.

### Python and PIP

According to [this article](https://people.eng.unimelb.edu.au/pbeuchat/asclinic/software/workflow_i2c.html#important-notes-on-i2c-usages-in-ros-nodes) you can access one I2C bus only exclusively from one ROS node. Because we want to communicate with several sensors over I2C we need to create our own ROS node that then communicates with all the sensors we want to have (IMU, TOF, BMS, ...). We will make use of some existing packages which we will install using PIP.

```
pip3 install smbus
```

### ROS

Install ROS according to [the official guide](http://wiki.ros.org/Installation/Ubuntu). Make sure you have sourced your ros installation with `source /opt/ros/noetic/setup.bash` and then Catkin make your cloned workspace
```
cd /home/vboxuser/git/rtklawnmower/catkin_ws/
catkin_make
```
Then also source your workspace `source ~/git/rtklawnmower/catkin_ws/devel/setup.bash`.
