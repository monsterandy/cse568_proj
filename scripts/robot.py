#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class Robot():
    def __init__(self):
        rospy.init_node('robot', anonymous=False)
        # self.map_subscriber = rospy.Subscriber('map', OccupancyGrid, self.update_occ_grid)
        self.update_occ_grid()
        # rospy.spin()

    def update_occ_grid(self):
        data = rospy.wait_for_message('map', OccupancyGrid)
        print(data)

if __name__ == "__main__":
    try:
        robot = Robot()
    except rospy.ROSInterruptException:
        pass