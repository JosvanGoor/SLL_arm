#!/usr/bin/env python
import roslib
roslib.load_manifest('borg_pioneer')
import rospy
import sys
import cjson

from sensor_msgs.msg import LaserScan
from borg_pioneer.srv import MemorySrv


class LaserToMemory:

    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.newLaserScanCB)

        print "Wait for write memory"
        self.service_write = rospy.ServiceProxy('memory', MemorySrv)
        rospy.wait_for_service('memory')
        print "Write memory found"

    def newLaserScanCB(self, data):
        #print (data.angle_max + abs(data.angle_min)) / data.angle_increment
        avg_distance = sum(data.ranges[0:len(data.ranges)]) / len(data.ranges)

        try:
            self.service_write(rospy.Time.now(), "LaserData", cjson.encode(avg_distance))
        except:
            print "Write memory not available"

def main(args):
    Laser = LaserToMemory()
    rospy.init_node('LaserToMemory')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
