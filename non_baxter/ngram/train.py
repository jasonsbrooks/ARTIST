#!/usr/bin/env python

"""
Train an ngram model
USAGE: ./ngram.py OUTFILE
"""

'''
Todo:
1) key transposition (multiple ngram models or transpose into one?)
2) skip bass/rhythm/strings/piano?


Complete:
1) Skip all drums

'''
import numpy as np
import sys, os
import re

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

NUM_NOTES = 128
matrix_size = (NUM_NOTES, NUM_NOTES, NUM_NOTES)

counts = np.zeros(matrix_size, dtype=np.int16)

from collections import deque

from sqlalchemy import desc, asc

from db import Session, Song, Track, Note

session = Session()

# iterate through all the tracks
test_counter = 0
MAX_TRACKS = 1000

for trk in session.query(Track).all():
    print os.path.basename(trk.song.title), ":", trk.instr_name

    # skip percurssion tracks
    regexp = re.compile(r'drum|cymbal', re.IGNORECASE)
    if trk.channel == 9 or regexp.search(trk.instr_name) is not None:
        print 'skipped percussion track'
        continue

    # skip bass tracks
    regexp = re.compile(r'bass', re.IGNORECASE)
    if (trk.channel >= 32 and trk.channel <= 39) or regexp.search(trk.instr_name) is not None:
        print 'skipped bass track'
        continue

    triple = deque()

    # and through all the notes in a track
    for note in trk.notes:
        if note.pitch < 0 or note.pitch >= NUM_NOTES:
            pass

        triple.append(note.pitch)

        # update our counts matrix
        if len(triple) > 3:
            triple.popleft()
            np.add.at(counts, tuple(triple), 1)

    test_counter += 1
    if (test_counter > MAX_TRACKS):
        break

with open(sys.argv[1], 'w') as outfile:
    np.save(outfile, counts)
