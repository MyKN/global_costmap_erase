#!/usr/bin/env python

import rospy
import subprocess

def clear_costmap_timer(event):
    try:
        subprocess.call(['rosservice', 'call', '/move_base/clear_costmaps'])
    except Exception as e:
        rospy.logerr("Failed to call clear_costmaps service: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('clear_costmap_timer_node')
    
    # 2 saniyede bir
    rospy.Timer(rospy.Duration(2.0), clear_costmap_timer)
        
    rospy.spin()
