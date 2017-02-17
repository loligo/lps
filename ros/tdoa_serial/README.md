In linux, to set the serial device names to values that are easy to use add the following to:

/etc/udev/rules.d/99-usb-serial.rules
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="AK04SUSO", SYMLINK+="ttyUSB.tdoa_anchor00"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="AK04SUS2", SYMLINK+="ttyUSB.tdoa_anchor01"
# and more rows for each anchor you have
```

where you will have to change the serial to what's correct for your anchors. To check you can use:
```bash
$ udevadm info -a -n /dev/ttyUSB0 |grep serial
    SUBSYSTEMS=="usb-serial"
    ATTRS{serial}=="AK04SUSO"    <---- This is the value used above
    ATTRS{serial}=="0000:00:14.0"
```
After making changes to the udev rules file you need to disconnect and reconnect the LPS.