#!/usr/bin/env python

import json, copy, rospy, pdb

from artist_performer import Performer
import baxter_artist.msg

from sensor_msgs.msg import (
    Image,
)

QUEUE_SIZE = 100

class NotePublisher(object):
    def __init__(self):
        self.publisher = rospy.Publisher('baxter_artist_notes', baxter_artist.msg.Note, queue_size=QUEUE_SIZE)
    
    def pub_note(self,note):
                
        # construct the note message
        msg = baxter_artist.msg.Note()
        msg.starttime = rospy.Time.now() + rospy.Duration.from_sec(1)
        msg.pitch = note 

        rospy.loginfo("publish - pitch: " + str(msg.pitch))
        
        # publish the message
        self.publisher.publish(msg)


def main():
    rospy.loginfo("Initializing node... ")
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
    # pdb.set_trace()

    # noteNameArray = ["C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "A6", "G6", "F6", "E6", "D6", "G6", "F6", "E6", "D6", "C6", "B5", "C6", "D6", "B5", "D6"]
    # noteNameArray = ["C6", "D6", "E6", "F6", "G6", "A6", "B6", "C7"]
    # noteNameArray = ["B5", "C6", "D6", "C6", "B5", "C6", "D6", "C6", "B5", "C6", "D6", "C6"]
    # noteNameArray = ["B5", "C7", "B5", "C7", "B5", "C7", "B5", "C7", "B5", "C7"]
    noteNameArray = ["C7", "B5", "C7","B5", "C7","B5", "C7","B5", "C7"]
    # noteNameArray = ["C7", "E6"]
    noteValArray = [notes[x] for x in noteNameArray]

    publisher = NotePublisher()
    for note in noteValArray:
        publisher.pub_note(note)
        rospy.sleep(2)


if __name__ == '__main__':
    main()