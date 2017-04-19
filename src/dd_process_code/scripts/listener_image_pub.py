#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import struct
import numpy as np
import matplotlib.pyplot as plt
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

global generateEstimates
generateEstimates=False

def callback(data):
###    rospy.loginfo(rospy.get_caller_id() )
###    print('Header: ', data.header)
###    print('seq: ', data.header.seq)
###    print('height: ', data.height)
###    print('width: ', data.width)
###    print('fields: ', data.fields)
###    print('point_step: ', data.point_step)
###    print('row_step: ', data.row_step)
###    print('bigendian: ', data.is_bigendian)

#
# Setup call to publish
#
###    pub = rospy.Publisher('/objects/obs1_e/rear/gps/rtkfix', Odometry, queue_size=10)
    pub = rospy.Publisher('/objects/obs1_diag/rear/gps/rtkfix', Odometry, queue_size=10)
#
# Convert data streams into floats etc.
#

    if generateEstimates:  # obstacle estimation processing
        pointStep=data.point_step
        rowLength=data.width
        xSize=500 #pix
        ySize=500 #pix
        xScale=5  #pix/meter
        yScale=5  #pix/meter
        xOffset=50 #meters
        yOffset=50 #meters
        ringRangeAvg=np.zeros([1,32])
        ringRangeInit=np.zeros([1,32])
        
        imageGrid = np.zeros([xSize,ySize])
        for iData in range(rowLength):
            baseIdx = iData*pointStep
            x = struct.unpack('f', data.data[baseIdx:baseIdx+4])
            y = struct.unpack('f', data.data[baseIdx+4:baseIdx+8])
            z = struct.unpack('f', data.data[baseIdx+8:baseIdx+12])
            lidarInt = struct.unpack('f', data.data[baseIdx+16:baseIdx+20])
            ringNum = struct.unpack('i', data.data[baseIdx+20:baseIdx+24])

#
# Assume a grid centered on capture vehicle for image purposes
#

            xIdx = int((x[0]+xOffset)*xScale)
            yIdx = int((y[0]+yOffset)*yScale)
            if xIdx < xSize and yIdx < ySize and x[0]>0 and ringNum[0]>11 and ringNum[0]<20:
###                imageGrid[xIdx,yIdx] = 1.0*ringNum[0]
                if ringRangeInit[0,ringNum[0]] < 1.0:
                    ringRangeAvg[0,ringNum[0]] = x[0]**2+y[0]**2
                    ringRangeInit[0,ringNum[0]] = 1.0
                elif  x[0]**2+y[0]**2 < 0.75 * ringRangeAvg[0,ringNum[0]]:
                    imageGrid[xIdx,yIdx] = 1.0

#
# Publish message
#

    if generateEstimates:  # regular processing mode
        print('regular processing mode:')
        x = 0.69  + np.random.normal()  # RTKFix values approximately equal to obs1 in dataset 1-10 in release 2
        y = -76.9 + np.random.normal()
        z = 2.18 
    else:
        print('diagnostic processing mode:')
        x = data.pose.pose.position.x  # diagnostic testing only
        y = data.pose.pose.position.y # + 0.7239  # diagnostic testing only
        z = data.pose.pose.position.z  # diagnostic testing only

    msg = Odometry()
    msg.header.stamp=data.header.stamp
    msg.pose.pose.position = Point(x, y, z) 
    pub.publish(msg)        
                
###        print('Timestamp (sec) = ', data.header.stamp.secs)
###        print('ringNum = ', ringNum)
###        cv2.imshow('image',imageGrid)
###        cv2.waitKey(0)
###        cv2.destroyAllWindows()

#
# End of callback
#

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    if generateEstimates:  # regular processing
        rospy.Subscriber('velodyne_points', PointCloud2, callback)
    else:  # diagnostic processing
        rospy.Subscriber('objects/obs1/rear/gps/rtkfix', Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
