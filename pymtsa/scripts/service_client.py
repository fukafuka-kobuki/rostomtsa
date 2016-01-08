#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def call_service():
    rospy.loginfo('waiting service')
    rospy.wait_for_service('call_me')
    try:
        service = rospy.ServiceProxy('call_me', Empty)
        response = service()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == "__main__":
    call_service()
