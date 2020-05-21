#!/usr/bin/env python
import roslib
import rospy
import rosbag
import json
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import numpy as np
f = open('/home/mkz/Downloads/can_bus/scene-0107_ms_imu.json',"r")
imuData=json.load(f)
print(imuData[0]["utime"]) 
#print(len(poseData))
f.close()
imuMsg= Imu()
bagFile=rosbag.Bag('/home/mkz/Downloads/nuscenes_bags/blob02/withCAN/107.bag','a')
for idx in range(0,len(imuData)):
    Utime=imuData[idx]["utime"]
    imuMsg.header.stamp.nsecs=(Utime%1e6)*1e3
    imuMsg.header.stamp.secs=np.floor(Utime/1e6)

    imuMsg.orientation.x=imuData[idx]["q"][0]
    imuMsg.orientation.y=imuData[idx]["q"][1]
    imuMsg.orientation.z=imuData[idx]["q"][2]
    imuMsg.orientation.w=imuData[idx]["q"][3]

    imuMsg.angular_velocity.x=imuData[idx]["rotation_rate"][0]
    imuMsg.angular_velocity.y=imuData[idx]["rotation_rate"][1]
    imuMsg.angular_velocity.z=imuData[idx]["rotation_rate"][2]

    imuMsg.linear_acceleration.x=imuData[idx]["linear_accel"][0]
    imuMsg.linear_acceleration.y=imuData[idx]["linear_accel"][1]
    imuMsg.linear_acceleration.z=imuData[idx]["linear_accel"][2]
    bagFile.write('/imu',imuMsg,imuMsg.header.stamp)

imuData=[]
# Pose Data:
f = open('/home/mkz/Downloads/can_bus/scene-0107_pose.json',"r")
poseData=json.load(f)
f.close()
poseMsg=Pose()
tmpHdr=Header()
for idx in range(0,len(poseData)):
    Utime=poseData[idx]["utime"]
    tmpHdr.stamp.nsecs=(Utime%1e6)*1e3
    tmpHdr.stamp.secs=np.floor(Utime/1e6)

    poseMsg.orientation.x=poseData[idx]["orientation"][0]
    poseMsg.orientation.y=poseData[idx]["orientation"][1]
    poseMsg.orientation.z=poseData[idx]["orientation"][2]
    poseMsg.orientation.w=poseData[idx]["orientation"][3]

    poseMsg.position.x=poseData[idx]["pos"][0]
    poseMsg.position.y=poseData[idx]["pos"][1]
    poseMsg.position.z=poseData[idx]["pos"][2]

    
    bagFile.write('/pose',poseMsg,tmpHdr.stamp)
bagFile.close()
