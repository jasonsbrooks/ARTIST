#!/usr/bin/env python

import json, copy, rospy, pdb, sys, signal

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
        msg.starttime = rospy.Time.now() + rospy.Duration.from_sec(2.5)
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

    notes = {"G4": 47,
        "A4": 49,
        "B4": 51,
        "C5": 52,
        "D5": 54,
        "E5": 56,
        "F5": 57,
        "G5": 59,
        "A5": 61,
        "B5":63,
        "C6":64,
        "D6":66,
        "E6":68,
        "F6":69,
        "F#6":70,
        "G6":71,
        "G#6":72,
        "A6":73,
        "A#6":74,
        "B6":75,
        "C7":76,
        "R": -1}

    noteNameArray = ["C6", "C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "A6", "G6", "F6", "E6", "D6", "G6", "F6", "E6", "D6", "C6", "B5", "C6", "D6", "B5", "D6"]
    # noteNameArray = ["C6", "C6", "D6", "E6", "F6", "G6", "A6", "B6", "C7", "B6", "A6", "G6", "F6", "E6", "D6", "C6"]
    # noteNameArray = ["B5", "C6", "D6", "C6", "B5", "C6", "D6", "C6", "B5", "C6", "D6", "C6"]
    # noteNameArray = ["B5", "C7", "B5", "C7", "B5", "C7", "B5", "C7", "B5", "C7"]
    # noteNameArray = ["C7", "B5", "C7","B5", "C7","B5", "C7","B5", "C7"]
    # noteNameArray = ["C7", "E6"]
    # noteNameArray = ["C5", "C5", "D5", "E5", "C6", "D6", "E6", "C6", "F6", "G6", "E6"]
    noteNameArray = ["C5", "C5", "D5", "E5", "F5", "G5", "A5", "B5", "C6", "D6", "E6", "F6", "G6", "A6", "B6", "C7"]
    # noteNameArray = ["C6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6"] 
    noteNameArray = ["C5", "C5", "B5", "C5", "R", "E6", "A5", "G5", "R", "E5", "B4", "C6", "R", "F5", "G5", "R", "C5", "R", "C6", "B4", "C5", "R", "F5", "B5", "C6", "R", "A4", "E5", "G4", "G5", "R", "E5", "R", "R", "F5", "R", "D5", "A4", "R", "C6", "R", "E5", "B4", "C6", "R", "F5", "G5", "R", "C5", "R", "C6", "B4", "C5", "R", "A5", "G5", "R", "E5", "B4", "C6", "R", "F5", "G5", "R", "C5", "R", "C6", "B4", "C5", "R", "F5", "B5", "C6", "R", "A4", "E5", "F5", "R", "E5", "B5", "C5", "R", "E6", "A5", "G5", "R", "B4", "C6", "R", "F5", "G5", "R", "C5", "R", "C6", "B4", "C5"]
    noteValArray = [notes[x] for x in noteNameArray]

    # pdb.set_trace()
    publisher = NotePublisher()
    for note in noteValArray:
        publisher.pub_note(note)
        rospy.sleep(2)

def signal_handler(signum, frame):
    rospy.loginfo('Signal handler called with signal ' + str(signum))
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()