#!/usr/bin/python

'''
Stores midi files in a directory into DB

>> ./midi_store.py [optional-folder]

if no folder specified, midi_store.py will store all midi files in current directory
'''

import sys
import os
import glob
from helpers import midi_to_song

path = ''


def main():
    if len(sys.argv) >= 2:
        path = sys.argv[1]
    for filename in glob.glob(os.path.join(path, '*.txt')):
        song_obj = midi_to_song(filename)
        #  store song_obj to db


if __name__ == "__main__":
    main()
    pass
