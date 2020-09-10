# common_gerobot_application

```
common_gerobot_application
├── CMakeLists.txt
├── include
│   └── common_gerobot_application
├── launch
│   ├── accelerometer_control.launch
│   ├── bringup.launch
│   ├── camera_control.launch
│   ├── dc_motor_control.launch
│   ├── encoder_control.launch
│   └── hokuyo_control.launch
├── msg
│   └── objCenter.msg
├── package.xml
├── README.md
├── script
│   ├── camera_converter.py
│   ├── camera_preview.py
│   ├── laser_obstacle.py
│   ├── laser_preview.py
│   └── teleop_key.py
├── setup.py
└── src
    └── common_gerobot_application
        ├── __init__.py
        ├── makesimpleprofile.py
        └── pid.py

```

# About
A project named as **"Capacity Buildingfor Caregivers and Older Persons in**
**Elderly Living Institution for Contactless Deliveries using Indoor Autonomous**
**Platform”**

![GitHub Logo](https://github.com/KhairulIzwan/common_gerobot_application/blob/master/img/WhatsApp%20Image%202020-09-10%20at%208.00.21%20PM.jpeg)

# Required Package
1. imutils
2. cv_camera
3. rosserial
4. common_arduino_application
5. urg_node

# How it works?
### camera_preview.py
Previewing a stream of image from the camera [webcam, usbcam, raspicam, etc]

1. roslaunch common_gerobot_application camera_control.launch
2. rosrun common_gerobot_application camera_converter.py [re-correct the image orientation]
3. rosrun common_gerobot_application camera_preview.py

### teleop_key.py
Controllig the robot with tele-operation [keyboard]

1. roslaunch common_gerobot_application dc_motor_control.launch
2. rosrun common_gerobot_application teleop_key.py

### laser_preview.py
Testing for the LIDAR reading

1. roslaunch common_gerobot_application hokuyo_control.launch
2. rosrun common_gerobot_application laser_preview.py

### laser_obstacle.py
Run robot avoiding the obstacle; stop

1. roslaunch common_gerobot_application hokuyo_control.launch
2. rosrun common_gerobot_application laser_obstacle.py
