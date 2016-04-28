#!/usr/bin/env python

import json, copy, rospy, pdb, sys, signal

from artist_performer import Performer
import baxter_artist.msg

from sensor_msgs.msg import (
    Image,
)

QUEUE_SIZE = 100
BPM = 60

# convert BPM to DURKS_PER_SECOND
DURKS_PER_SECOND = 8.0 * float(BPM) / 60.0
SECONDS_PER_DURK = 1.0 / float(DURKS_PER_SECOND)

class NotePublisher(object):
    def __init__(self):
        rospy.loginfo("Initializing NotePublisher")
        self.publisher = rospy.Publisher('baxter_artist_notes', baxter_artist.msg.Note, queue_size=QUEUE_SIZE)
        
        # need to sleep between creating Publisher and publishing to it.
        rospy.loginfo("Sleeping...")
        rospy.sleep(5)

    def pub_note(self,pitch,starttime):
        
        # construct the note message
        msg = baxter_artist.msg.Note()
        msg.starttime = starttime
        msg.pitch = pitch 

        rospy.loginfo("publish - pitch: " + str(msg.pitch))
        
        # publish the message
        self.publisher.publish(msg)

    def pub_notes(self,notes):
        # wait 2 seconds before starting the piece
        piece_starttime = rospy.Time.now() + rospy.Duration.from_sec(2)

        # when the next note should start
        note_starttime = piece_starttime

        for note in notes:
            pitch = note[0]
            dur = note[1]

            # send the note message
            self.pub_note(pitch,note_starttime)
            note_starttime += rospy.Duration.from_sec(dur * SECONDS_PER_DURK)

def main():
    rospy.loginfo("Initializing node... ")
    rospy.init_node("play_xylophone")
    # performer = Performer()

    # print("Performing!...")
    # performer.perform()

    # rospy.signal_shutdown("Finished perform control")
    # print("Done with the performance. A+")

    noteToNum = {"G4": 47,
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

    # noteNameArray = ["C6", "C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "A6", "G6", "F6", "E6", "D6", "G6", "F6", "E6", "D6", "C6", "B5", "C6", "D6", "B5", "D6"]
    # noteNameArray = ["C6", "C6", "D6", "E6", "F6", "G6", "A6", "B6", "C7", "B6", "A6", "G6", "F6", "E6", "D6", "C6"]
    # noteNameArray = ["B5", "C6", "D6", "C6", "B5", "C6", "D6", "C6", "B5", "C6", "D6", "C6"]
    # noteNameArray = ["B5", "C7", "B5", "C7", "B5", "C7", "B5", "C7", "B5", "C7"]
    # noteNameArray = ["C7", "B5", "C7","B5", "C7","B5", "C7","B5", "C7"]
    # noteNameArray = ["C7", "E6"]
    # noteNameArray = ["C5", "C5", "D5", "E5", "C6", "D6", "E6", "C6", "F6", "G6", "E6"]
    # noteNameArray = ["C5", "C5", "D5", "E5", "F5", "G5", "A5", "B5", "C6", "D6", "E6", "F6", "G6", "A6", "B6", "C7"]
    # noteNameArray = ["C6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6", "C6", "E6", "F6", "F#6", "G6", "A6", "B6", "G6", "C7", "A#6", "A6", "G#6", "G6", "F6", "E6", "D6"] 
    # noteNameArray = ["C5", "C5", "B5", "C5", "R", "E6", "A5", "G5", "R", "E5", "B4", "C6", "R", "F5", "G5", "R", "C5", "R", "C6", "B4", "C5", "R", "F5", "B5", "C6", "R", "A4", "E5", "G4", "G5", "R", "E5", "R", "R", "F5", "R", "D5", "A4", "R", "C6", "R", "E5", "B4", "C6", "R", "F5", "G5", "R", "C5", "R", "C6", "B4", "C5", "R", "A5", "G5", "R", "E5", "B4", "C6", "R", "F5", "G5", "R", "C5", "R", "C6", "B4", "C5", "R", "F5", "B5", "C6", "R", "A4", "E5", "F5", "R", "E5", "B5", "C5", "R", "E6", "A5", "G5", "R", "B4", "C6", "R", "F5", "G5", "R", "C5", "R", "C6", "B4", "C5"]
    # noteNameArray = ["C6","C6","B5","A5","G5","F5"]
    noteNameArray = ["B5","B5","C6","D6","E6","F6","G6","A6","B6","C7"]
    # noteNameArray = ["C6","C6","C6","C6","C6","C6"]
    # notes = [(notes[x],4) for x in noteNameArray]


    # notes = [("B5",8),("B5",4),("C6",8),("D6",4),("E6",8),("F6",4),("G6",8),("A6",4),("B6",8),("C7",4)]
    notes = [("B5",8),("E6",8),("B5",8),("E6",8),("C7",8),("G6",8),("C7",8),("G6",8),("B6",8),("C7",8)]

    notes = map(lambda (pitch,dur): (noteToNum[pitch],dur),notes)

    rospy.loginfo(str(notes))

    # pdb.set_trace()
    publisher = NotePublisher()
    publisher.pub_notes(notes)

def signal_handler(signum, frame):
    rospy.loginfo('Signal handler called with signal ' + str(signum))
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()