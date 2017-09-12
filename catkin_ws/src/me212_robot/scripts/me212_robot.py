#!/usr/bin/python

# 2.12 Lab 2 me212_robot: ROS driver running on the pc side to read and send messages to arduino
# Peter Yu Sept 2016

import rospy
import tf
import numpy as np
import threading
import serial
import pdb

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose

port = '/dev/ttyACM0'

class Arduino():
    def __init__(self, port = '/dev/ttyACM0'):
        self.comm = serial.Serial(port, 115200, timeout=5)
        self.br = tf.TransformBroadcaster()
        self.sendbuff = []
        self.readbuff = ''
        
        self.thread = threading.Thread(target=self._loop)
        self.thread.start()
        self.prevtime = rospy.Time.now()

    def _loop(self):
        while not rospy.is_shutdown():
            # 1. get the current data from serial
            serialDatas = self.comm.read(80)
            self.readbuff += serialDatas
            #print self.readbuff
            #print serialDatas
            currentpos_str = ''
            
            # 2. find the last complete data
            chunks = serialDatas.split('\n')
            if len(chunks) >= 1 and len(chunks[-1]) >= 2 and chunks[-1].startswith( '#' ) and chunks[-1].endswith( '#' ):
                self.readbuff = ""
                currentpos_str = chunks[-1]
            elif len(chunks) >= 2 and len(chunks[-2]) >= 2 and chunks[-2].startswith( '#' ) and chunks[-2].endswith( '#' ):
                self.readbuff = chunks[-1]  # incomplete data
                currentpos_str = chunks[-2]
            else:
                continue # not enough readings in readbuff
            
            # 3. parse the string to double
            splitData = currentpos_str.split(',');
            #print splitData
            try:
                x = float(splitData[1]);
                y = float(splitData[2]);
                phi = float(splitData[3]);
                
                print 'x=', x, ' y=', y, ' theta =', phi, ' hz =', 1.0/(rospy.Time.now().to_sec() - self.prevtime.to_sec())
                self.prevtime = rospy.Time.now()
                
                self.br.sendTransform((x, y, 0),                                                # to 3d translation (x, y, z)
                                     tf.transformations.quaternion_from_euler(0, 0, phi),       # to 3d rotation    (qx, qy, qz, qw)
                                     rospy.Time.now(), # timestamp
                                     "base_link",      # robot frame
                                     "map")            # base frame
            except:
                print 'cannot parse', splitData

def poselist2pose(poselist):
    pose = Pose()
    pose.position.x = poselist[0]
    pose.position.y = poselist[1]
    pose.position.z = poselist[2]
    pose.orientation.x = poselist[3]
    pose.orientation.y = poselist[4]
    pose.orientation.z = poselist[5]
    pose.orientation.w = poselist[6]
    return pose

def createPointMarker2(points, marker_id, rgba = None, pose=[0,0,0,0,0,0,1], frame_id = '/map'):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.POINTS
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.id = marker_id
    
    n = len(points)
    sub = 1
    
    if rgba is not None:
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = tuple(rgba)
        
    for i in xrange(0,n,sub):
        p = Point()
        p.x = points[i][0]
        p.y = points[i][1]
        p.z = points[i][2]
        marker.points.append(p)
        
        
    if rgba is None:
        for i in xrange(0,n,sub):
            p = ColorRGBA()
            p.r = points[i][3]
            p.g = points[i][4]
            p.b = points[i][5]
            p.a = points[i][6]
            marker.colors.append(p)
        
    marker.pose = poselist2pose(pose)
    
        
    return marker
    


def main():
    rospy.init_node('me212_robot', anonymous=True)
    arduino = Arduino()
    
    # visualize U
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    points = []
    for i in np.linspace(0, 1):
        points.append([i, 0, 0])
    for i in np.linspace(-np.pi/2, np.pi/2):
        points.append([np.cos(i)*0.25 + 1, np.sin(i)*0.25 + 0.25, 0])
    for i in np.linspace(0, 1):
        points.append([i, 0.5, 0])
    
    for i in xrange(0,9):
        vis_pub.publish(createPointMarker2(points, 1, [0.6, 0.6, 0, 1]))
        rospy.sleep(0.1)
    
    rospy.spin()
    
    
    
if __name__=='__main__':
    main()
    
