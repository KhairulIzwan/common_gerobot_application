# common_gerobot_application

```
common_gerobot_application
├── CMakeLists.txt
├── img
│   ├── WhatsApp Image 2020-09-10 at 8.00.07 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.07 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.08 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.08 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.09 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.09 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.10 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.10 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.11 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.11 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.12 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.16 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.16 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.17 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.18 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.18 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.19 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.19 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.20 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.20 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.21 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.21 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.44 PM (1).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.44 PM (2).jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.44 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.45 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.46 PM.jpeg
│   ├── WhatsApp Image 2020-09-10 at 8.00.47 PM.jpeg
│   ├── WhatsApp Video 2020-09-10 at 8.00.12 PM.mp4
│   ├── WhatsApp Video 2020-09-10 at 8.00.13 PM (1).mp4
│   ├── WhatsApp Video 2020-09-10 at 8.00.13 PM.mp4
│   ├── WhatsApp Video 2020-09-10 at 8.00.14 PM (1).mp4
│   ├── WhatsApp Video 2020-09-10 at 8.00.14 PM.mp4
│   ├── WhatsApp Video 2020-09-10 at 8.00.15 PM (1).mp4
│   ├── WhatsApp Video 2020-09-10 at 8.00.15 PM.mp4
│   ├── WhatsApp Video 2020-09-10 at 8.00.17 PM.mp4
│   ├── WhatsApp Video 2020-09-10 at 8.00.46 PM.mp4
│   ├── WhatsApp Video 2020-09-10 at 8.00.47 PM.mp4
│   └── WhatsApp Video 2020-09-10 at 8.00.48 PM.mp4
├── include
│   └── common_gerobot_application
├── launch
│   ├── accelerometer_control.launch
│   ├── bringup.launch
│   ├── camera_control.launch
│   ├── camera_converter.launch
│   ├── dc_motor_control.launch
│   ├── encoder_control.launch
│   └── hokuyo_control.launch
├── msg
│   └── objCenter.msg
├── package.xml
├── README.md
├── script
│   ├── camera_apriltag_center.py
│   ├── camera_apriltag.py
│   ├── camera_apriltag_tracking_izwan.py
│   ├── camera_apriltag_tracking.py
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
        ├── makesimpleprofile.pyc
        ├── pid.py
        └── pid.pyc
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

### camera_apriltag.py
Detect and determine the apriltag

1. roslaunch common_gerobot_application camera_control.launch
2. rosrun common_gerobot_application camera_converter.py [re-correct the image orientation]
3. rosrun common_gerobot_application camera_apriltag.py

### camera_apriltag_center.py
Detect and determine the apriltag, thus publish the center of apriltag

1. roslaunch common_gerobot_application camera_control.launch
2. rosrun common_gerobot_application camera_converter.py [re-correct the image orientation]
3. rosrun common_gerobot_application camera_apriltag_center.py

### camera_apriltag_tracking.py
### camera_apriltag_tracking_izwan.py
Detect and determine the apriltag, thus publish the center of apriltag

1. roslaunch common_gerobot_application camera_control.launch
2. rosrun common_gerobot_application camera_converter.py [re-correct the image orientation]
3. rosrun common_gerobot_application camera_apriltag_center.py
4. rosrun common_gerobot_application camera_apriltag_tracking.py

# Notes:
To simplify, in order to activating all the sensors and actuators on the robot, 
we can simply execute:

1. roslaunch common_gerobot_application bringup.launch

which is equivalent to:

1. roslaunch common_gerobot_application camera_control.launch
2. roslaunch common_gerobot_application dc_motor_control.launch
3. roslaunch common_gerobot_application encoder_control.launch
4. roslaunch common_gerobot_application hokuyo_control.launch
5. roslaunch common_gerobot_application accelerometer_control.launch
6. roslaunch common_gerobot_application camera_converter.launch
