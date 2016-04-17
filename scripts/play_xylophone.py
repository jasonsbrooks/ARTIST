#!/usr/bin/env python

import json, copy, rospy

from baxter_artist import Performer
import baxter_artist.msg

from sensor_msgs.msg import (
    Image,
)

class NotePublisher(object):
    def __init__(self):
        self.publisher = rospy.Publisher('baxter_artist_notes', baxter_artist.msg.Note, queue_size=10)
    
    def pub_note(self,note):
        # construct the note message
        msg = baxter_artist.msg.Note()
        msg.starttime = rospy.Time.now()
        msg.pitch = note 

        # publish the message
        self.publisher.publish(msg)


def main():
    print("Initializing node... ")
    rospy.init_node("play_xylophone")
    # performer = Performer()

    # print("Performing!...")
    # performer.perform()

    # rospy.signal_shutdown("Finished perform control")
    # print("Done with the performance. A+")

    notes = {"B5":63,
        "C6":64,
        "D6":66,
        "E6":68,
        "F6":69,
        "G6":71,
        "A6":73,
        "B6":75,
        "C7":76}

    noteNameArray = ["C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "A6", "G6", "F6", "E6", "D6", "G6", "F6", "E6", "D6", "C6", "B5", "C6", "D6", "B5", "D6"]
    # noteNameArray = ["C7", "B5", "C7","B5", "C7","B5", "C7","B5", "C7"]
    # noteNameArray = ["C7", "E6"]
    noteValArray = [notes[x] for x in noteNameArray]

    publisher = NotePublisher()
    for note in noteValArray:
        publisher.pub_note(note)
        rospy.sleep(1)


if __name__ == '__main__':
    main()