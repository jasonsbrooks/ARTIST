#!/usr/bin/python

'''
Stores midi files in a directory into DB

>> ./midi_store.py [optional-folder]

if no folder specified, midi_store.py will store all midi files in current directory
'''

import sys
import fnmatch
import os
from helpers import midi_to_song


def main():
    midiDirectory = 'data/'
    if len(sys.argv) >= 2:
        midiDirectory = sys.argv[1]

    listPoo = []
    for root, dirnames, filenames in os.walk(midiDirectory):
        for filename in fnmatch.filter(filenames, '*.mid'):
            midiPath = os.path.abspath(os.path.join(root, filename))
            print "Analyzing " + midiPath.split('/')[-1]
            song_obj = midi_to_song(midiPath)
            print "FINISHED...moving on"
    print "DONE"



if __name__ == "__main__":
    main()
