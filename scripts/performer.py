#!/usr/bin/env python

import rospy,signal,sys
from artist_performer import Performer

def main():
    rospy.init_node("performer")
    rospy.loginfo("Initializing node... ")

    performer = Performer()
    performer.subscribe()
    performer.set_neutral()

def signal_handler(signum, frame):
    rospy.loginfo('Signal handler called with signal ' + str(signum))
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()