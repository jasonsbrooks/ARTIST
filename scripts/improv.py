#!/usr/bin/env python

from optparse import OptionParser

from artist_generator import ngram,ga
from artist_performer import NotePublisher,ARTIST_SHARE_DIR

import rospy

TWELVE_BAR_BLUES = [1,1,1,1,4,4,1,1,5,4,1,1]

noteToNum = {1: 52,
    2: 54,
    3: 56,
    4: 57,
    5: 59,
    6: 61,
    7:63,
    8:64,
    9:66,
    10:68,
    11:69,
    12:71,
    13:73,
    14:75,
    15:76,
    16:64,
    17:66,
    18:68,
    19:69,
    20:71,
    21:73,
    -1: -1}

def main():

    rospy.loginfo("Initializing node... ")
    rospy.init_node("improv_xylophone")

    ngram_dir = ARTIST_SHARE_DIR + "/ngram"
    notes = ga.run(ngram_generate=(lambda: ngram.generate(ngram_dir,TWELVE_BAR_BLUES)),num_iter=800)

    notes = map(lambda (pitch,dur): (noteToNum[pitch],dur), notes)
    publisher = NotePublisher()
    publisher.pub_notes(notes)
    
if __name__ == '__main__':
    main()
