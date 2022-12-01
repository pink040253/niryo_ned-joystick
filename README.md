# niryo_ned-joystick
2022/08/20 智造咖展覽

-

### robot_ip setting in codes
**hotspot mode** : `10.10.10.10`

**wifi mode** : `10.10.10.110`

---

## niryo_joystick_raspi4.py
Joystick usb-hub connect with Raspberry Pi.

### Install packages
```bash
pip3 install pyniryo
pip3 install udev
pip3 install evdev
```

### Add udev rule
確認 joystick ID
```bash
lsusb   # Check ID of the joystick, ex.Bus 001 Device 012: ID 045e:02ea Microsoft Corp. 
```

修改 `.rules`
```bash
cd /etc/udev/rules.d
nano .rules
# SUBSYSTEM=="usb", ATTR{idVendor}=="<<front 4 words>>", ATTR{idProduct}=="<<last 4 words>>", MODE="0666"
```

完成後重開機
```bash
sudo reboot
```

### Operate 
1. Remote niryo ned.
2. Cmd `python3 niryo_joystick_raspi4.py`

---

## niryo_joystick_ubuntu1804.py
Joystick usb-hub connect with PC.

### Install packages
```bash
pip3 install pyniryo
pip3 install pygame
```

### Operate
```bash
python3 niryo_joystick_ubuntu1804.py
```

---

## niryo_joystick_win10.py
Joystick usb-hub connect with PC.

### Install packages
```bash
pip3 install pyniryo
pip3 install pygame
```

### Operate
```bash
python3 niryo_joystick_win10.py
```

---
## niryo_joystick_mqtt
沒有成功，待有朝一日完成

## Sources
* [udev (To get joystick information on raspi4)](https://www.reddit.com/r/VFIO/comments/ae8jb8/evdev_permission_denied_on_devinputevent13/)
* [Check joystick information on raspi4 in terminal](https://linuxhint.com/connect-xbox-controller-raspberry-pi/)
* [evdev (To get joystick information on raspi4)](https://python-evdev.readthedocs.io/en/latest/)
* [Niryo Docs. - Mouse control application](https://docs.niryo.com/applications/ned/v1.0.3/en/source/examples/control_ned_mouse.html)
* [Niryo Docs. - Control Ned/Ned2 with MoveIt and ROS Multi-Machines](https://docs.niryo.com/applications/ned/v1.0.3/en/source/tutorials/moveit_multimachines.html)





