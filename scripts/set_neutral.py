#!/usr/bin/env python

import rospy
from baxter_artist import BaxterController

def main():
    rospy.loginfo("Initializing node... ")
    rospy.init_node("play_xylophone")
    controller = BaxterController()

    controller.set_neutral()

    rospy.signal_shutdown("Finished control")

if __name__ == '__main__':
    main()