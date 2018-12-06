# success_baxter_camera_calibration
License - MIT  
Maintainer - zhi.tan@ri.cmu.edu


This package contains two parts (1) An interface to allow us to calibrate baxter's hand and head camera, and (2) a node that publishes those information independent of the original camera.

This is to mainly work around the fact that Baxter's camera's initial calibration file seemed to be incorrect.

# Steps
1. Run the following code to start calibration
```
roslaunch success_baxter_camera_calibration calibrate.launch camera:="<name of the camera you want>" image_topic:="'/cameras/left_hand_camera/image'"
```
You will then see a calibration window with the image shown. [Follow the steps here]([http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). Once you press commit, it will save a .yaml configuration file in `${PACKAGE_ROOT}/calibration_files/<camera_name>.yaml`
2. To publish the camera information, run
```
roslaunch success_baxter_camera_calibration cam_info_pub.launch pub_topic:="<name of the topic you want to publish the camera info>" file_path:=<path that you specify before. If you just put <camere_name>, it will automatically try resolve to ${PACKAGE_ROOT}/calibration_files/<camera_name>.yaml>
```
This will then publish the camera_info at a rate of 10Hz