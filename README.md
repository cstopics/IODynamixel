# IODynamixel

Library to control Dynamixel motors vía *creature* scheme.

## Setting up the USB port

This library was tested with the *U2D2* controller (*Robotis*). In order to reach good update frequencies, it is necesary to reduce the latency delay, as follows:

``` bash
$ sudo su
# echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# exit
```
