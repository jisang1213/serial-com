![cpp-version](https://img.shields.io/badge/standard-C++17-black.svg?logo=c%2B%2B) ![ubuntu-version](https://img.shields.io/badge/ubuntu-20.04-e95420?logo=ubuntu) ![ros-version](https://img.shields.io/badge/ROS2-foxy-blue?logo=ros) [![raisin-ci](https://github.com/HuboLabKaist/raisin/workflows/raisin-ci/badge.svg)](https://github.com/HuboLabKaist/raisin/actions/workflows/ci.yml)


# Raisin_RF

RF_SBUS_serial reading & decoding package for Rasin.


It maps SBUS Signal into ROS Joy msg.

SBUS_Reading Code is sourced from : <https://github.com/uzh-rpg/rpg_quadrotor_control/tree/22b55b085546a3322c302a68c2b67108a092a298/bridges/sbus_bridge>

***

### Three Hardware Component
Taranis X9 (Controller)

FrSky X8R (Receiver)

FT232 Module(Serial Converter)

***
### Prerequisites(done in raibo)
You need to make an .rules file in your robot to set USB Port name and to get R/W permission on serial port.


Check the idVendor/idProduct/serial number of FTDI device by the follwing command
```bash
$ udevadm info --name=/dev/ttyUSBx --attribute-walk
```

Modify the rule file base on the information above.
example
```bash
$ vim 10-usb-serial.rules
$ cp vim 10-usb-serial.rules  /etc/udev/rules.d
```

10-usb-serial.rules (for Port name)

```
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001",ATTRS{serial}=="A18866V6", SYMLINK+="ftdi"
```
50-usb-serial.rules (for R/W permission)
```
     SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0777:",ATTRS{serial}="A18866V6"
```

***


***
### Running procedure of RF_Node
#### 1.  Turn on the controller and check the connection(green LED on reciever)
#### 2.  Run Up script in remote PC 
Note) Do not close the terminal to monitor error like USB disconnection.
```bash
$ sh $raisin_src/raisin_rf/scripts/remote_rf_joy_up.sh
```
#### 3. Check topic echo 

```bash
$ ros2  topic echo joy
```

#### 4. After experiment finished, run Down script to shutdown

```bash
$ sh $raisin_src/raisin_rf/scripts/remote_rf_joy_down.sh
```
***
### Key Bindings
Left Right Stick 

SA= StandUP/Neutral/SitDown

SD= Emergency Stop/Neutral/Neutral


***
## Warning
## 1. When you first boot the robot, be sure to turn on the controller before running the RF_node.

After booting the robot, turn on the controller to check the connection with the receiver before running the node. 

Check the signal level of the connection in the upper left  corner of the controller display. 

To generate signal, SBUS receiver must be connected to the controller for the first time after power on.

If it is connected once, the signal will continue as long as the robot is powered.


## 2. At every moment, check that all buttons are neutral.
SA/SD + OFFSET knobs.



***
## Error Handle
## Case 1. USB connection lost
If the connection between PC and FTDI converter is unstable, the node will be shutdown and left 0 command topic.
Check the USB connection, and run the node again.

## Case 2. Wireless connection lost(Controller battery down)
If the connection between the controller and the receiver is unstable, the receiver will enter fail-safe mode automatically. In this mode, receiver generates 0 command until the connection is restored.


There is no need to rerun the node here.

If the controller is turned off after the initial connection, output 0 until the controller is turned on.




