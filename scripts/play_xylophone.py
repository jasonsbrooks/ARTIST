#!/usr/bin/env python

import json, copy, rospy

from baxter_artist import Performer

from sensor_msgs.msg import (
    Image,
)

def main():
    print("Initializing node... ")
    rospy.init_node("play_xylophone")
    performer = Performer()

    print("Performing!...")
    performer.perform()

    rospy.signal_shutdown("Finished perform control")
    print("Done with the performance. A+")

if __name__ == '__main__':
    main()