rostensorflow
=====================

- Install tensorflow (see https://www.tensorflow.org/versions/r0.9/get_started/os_setup.html)
- Install ROS (see http://wiki.ros.org)
- Install cv-bridge, and camera driver (for example, cv_camera)

```bash
$ sudo apt-get install ros-indigo-cv-bridge ros-indigo-cv-camera
```

image_recognition.py
--------------------------------

* publish: /result (std_msgs/String)
* subscribe: /image (sensor_msgs/Image)

How to try

```bash
$ roscore
$ rosrun cv_camera cv_camera_node
$ python image_recognition.py image:=/cv_camera/image_raw
$ rostopic echo /result
```
