#!/usr/bin/env python

"""
Train an ngram model
USAGE: ./ngram.py OUTFILE
"""
import numpy as np
import sys, os

if len(sys.argv) < 2:
	print(__doc__)
	sys.exit(1)

NUM_NOTES = 128
matrix_size = (NUM_NOTES,NUM_NOTES,NUM_NOTES)

counts = np.zeros(matrix_size,dtype=np.int16)

from collections import deque

from sqlalchemy import desc, asc

from db import Session, Song, Track, Note

session = Session()

# iterate through all the tracks
for trk in session.query(Track).all():
	print os.path.basename(trk.song.title), ":", trk.instr_name

	triple = deque()

	# and through all the notes in a track
	for note in trk.notes:		
		if note.pitch < 0 or note.pitch >= NUM_NOTES:
			pass

		triple.append(note.pitch)

		# update our counts matrix
		if len(triple) > 3:
			triple.popleft()
			np.add.at(counts,tuple(triple),1)

with open(sys.argv[1],'w') as outfile:
	np.save(outfile,counts)