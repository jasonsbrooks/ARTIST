#!/usr/bin/env python

import rospy
from artist_performer import Performer

def main():
    rospy.init_node("performer")
    rospy.loginfo("Initializing node... ")

    performer = Performer()
    performer.subscribe()

if __name__ == '__main__':
    main()