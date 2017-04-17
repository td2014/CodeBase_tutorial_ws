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

def callback(data):
###    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.)
    rospy.loginfo(rospy.get_caller_id() )
    print('Header: ', data.header)
    print('seq: ', data.header.seq)
    print('height: ', data.height)
    print('width: ', data.width)
    print('fields: ', data.fields)
    print('point_step: ', data.point_step)
    print('row_step: ', data.row_step)
    print('bigendian: ', data.is_bigendian)
#
# Convert data streams into floats etc.
#
    xOffset=0
    yOffset=4
    zOffset=8
    intOffset=12
    pointStep=data.point_step
    rowLength=data.width
    frameNum=0
    targFrame=0

    if frameNum==targFrame:  #create birds-eye snapshot (assume x-y plane)
        xSize=500 #pix
        ySize=500 #pix
        xScale=5  #pix/meter
        yScale=5  #pix/meter
        xOffset=50 #meters
        yOffset=50 #meters
        imageGrid = np.zeros([xSize,ySize])
        for iData in range(rowLength):
            baseIdx = iData*pointStep
            x = struct.unpack('f', data.data[baseIdx:baseIdx+4])
            y = struct.unpack('f', data.data[baseIdx+4:baseIdx+8])
            z = struct.unpack('f', data.data[baseIdx+8:baseIdx+12])

#
# Assume a 100x100 meter grid centered on vehicle, resolution 1 meter (for now)
#

            xIdx = int((x[0]+xOffset)*xScale)
            yIdx = int((y[0]+yOffset)*yScale)
            if xIdx < xSize and yIdx < ySize :
                imageGrid[xIdx,yIdx] = 1.0

        cv2.imshow('image',imageGrid)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
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

    rospy.Subscriber('velodyne_points', PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
