#!/usr/bin/env python

"""
This is copied and modified from https://gist.github.com/rossbar/ebb282c3b73c41c1404123de6cea4771
"""
"""
pointgrey_camera_driver (at least the version installed with apt-get) doesn't
properly handle camera info in indigo.
This node is a work-around that will read in a camera calibration .yaml
file (as created by the cameracalibrator.py in the camera_calibration pkg),
convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.
The yaml parsing is courtesy ROS-user Stephan:
    http://answers.ros.org/question/33929/camera-calibration-parser-in-python/
This file just extends that parser into a rosnode.
"""
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
import rospkg
import os

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

if __name__ == "__main__":
    # Initialize publisher node
    rospy.init_node("camera_calibration_pub_node", anonymous=True)
    yaml_path_info = rospy.get_param('~calibration_file_path',"")
    cam_topic = rospy.get_param('~calibration_topic_name')

    if not yaml_path_info.endswith('.yaml'):
        yaml_path_info = yaml_path_info + '.yaml'

    if not os.path.exists(yaml_path_info):
        rospack = rospkg.RosPack()
        yaml_path_info = os.path.join(rospack.get_path("success_baxter_camera_calibration"),"calibration_files", yaml_path_info)
        if not os.path.exists(yaml_path_info):
            raise RuntimeError("Cannot find calibration file")

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(yaml_path_info)


    publisher = rospy.Publisher(cam_topic, CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()