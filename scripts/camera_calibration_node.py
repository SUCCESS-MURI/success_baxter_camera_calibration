#!/usr/bin/env python

from sensor_msgs.msg import CameraInfo
from camera_info_manager import CameraInfoManager
import rospy
import rospkg
import os

if __name__ == "__main__":
    rospy.init_node('camera_calibration_node')
    #get the parameters for the node
    camera_name = rospy.get_param('camera_name')
    rospack = rospkg.RosPack()
    camera_info_path = rospy.get_param('info_path',
        os.path.join(rospack.get_path("success_baxter_camera_calibration"),"calibration_files","{}.yaml".format(camera_name)))
    #make sure the directory exist
    came_info_dir = os.path.dirname(camera_info_path)
    if not os.path.exists(came_info_dir):
        os.makedirs(came_info_dir)
    
    #covert the path to URL
    camera_info_path = "file://" + camera_info_path
    
    #start the camera calibration
    CameraInfoManager(cname=camera_name, url=camera_info_path)

    rospy.loginfo("starting fake camera calibration node")
    rospy.spin()