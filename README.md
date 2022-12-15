# rtklawnmower

Work in progress. Inspired by https://openmower.de/

## Hardware

* Lawn Mower: ZSA Zuchetti Ambrogio L15 Deluxe
* Computer: Raspberry Pi 4 (8GB)
* Microcontroller: Arduino Nano
* Motor driver: ZS-X11H V2
* GPS Device: Ardusimple Simplertk2b
* Some resistors and cables

## Software

* Ubuntu 20.04 Server
* ROS1
* Packages: gpsd gpsd–clients python-gps rtklib

## Installation

The Simplertk2b is going to be connected to the Raspberry Pi through a serial connection (using the provided Arduino Serial Shield by Ardusimple). For this to work, serial needs to be enabled by adding the line `enable_uart=1` to `/boot/firmware/syscfg.txt`. As we don't want to login through serial we remove the part `console=serial0,115200` from `/boot/firmware/cmdline.txt`. Disable serial Getty by `sudo systemctl stop serial-getty@ttyS0.service` and `sudo systemctl disable serial-getty@ttyS0.service`.

Furthermore Bluetooth needs to be disabled. This can be done by adding another line `dtoverlay=disable-bt` to the file `/boot/firmware/usercfg.txt`.

Unfortuunately, as soon as the Simplertk2b is connected, the Pi will no longer boot. When hooking it up to a screen through HDMI you see that it is stuck before booting into Ubuntu. This is because the messages from the GPS module coming thrugh serial now interrupt the autoboot countdown. Therefore you need to adjust uboot as described here https://raspberrypi.stackexchange.com/questions/116074/how-can-i-disable-the-serial-console-on-distributions-that-use-u-boot/117950#117950.

After rebooting you should now see (scrambled) messages coming in from the GPS device when you look at `less -f /dev/ttyAMA0`.

We are going to configure `gpsd` to provide the GPS data. For this to work `/etc/default/gpsd` has to be adjusted for `DEVICES="/dev/ttyAMA0"` and `GPSD_OPTIONS="-s 38400"` (select the serial speed baud rate that your ublox is currently configured). Afterwards restart the deamon with `sudo systemctl restart gpsd.socket`. Then finally you should see some GPS data when you open up `cgps`.

In order for us to provide correction data to the rover, we are using a free NTRIP caster. Later we will add our own base station. The connection is done from the Raspberry Pi through USB to the second USB port of the Simplertk2b board that connects to the XBee socket, where we connect the RX/TX-pins according to https://youtu.be/qlkN70bBfFQ. The F9P configuration can be found in [gnss-rover.txt](gnss-rover.txt).

For sending the correction data, we also use str2str, which can be added as a package with `sudo apt install rtklib`. As we want to have the data streamed to the Simplertk2b board at all times we will install `str2str` as a systemd servie. Have a look at [str2str.service](str2str.service) and then install it as follows:
```
sudo cp str2str.service /lib/systemd/system/.
sudo systemctl start str2str.service
sudo systemctl enable str2str.service
```
