#!/usr/bin/env python

"""
Train an ngram model
USAGE: ./ngram.py OUTFILE
"""
import numpy as np
import sys

if len(sys.argv) < 2:
	print(__doc__)
	sys.exit(1)

NUM_NOTES = 88
matrix_size = (NUM_NOTES,NUM_NOTES,NUM_NOTES)

counts = np.zeros(matrix_size,dtype=np.int16)

from collections import deque

### DB STUFF
from db_reset import Song, Track, Note

from sqlalchemy import create_engine, desc, asc
from sqlalchemy.orm import sessionmaker

engine = create_engine('sqlite:////tmp/artist.db')
Session = sessionmaker(bind=engine)
session = Session()

# iterate through all the tracks
for trk in session.query(Track.query.join(Song)):
	triple = deque()

	# and through all the notes in a track
	for note in session.query(trk):
		triple.popleft()
		triple.append(note.pitch)

		# update our counts matrix
		if len(triple) == 3:
			np.add.at(counts,tuple(triple),1)

sums = counts.sum(axis=2)

probs = counts.astype(np.float)

# iterate over thru dimensions and calculate probabilities
for d_one in xrange(matrix_size[0]):
	for d_two in xrange(matrix_size[1]):
		for d_three in xrange(matrix_size[2]):
			probs[d_one,d_two,d_three] = probs[d_one,d_two,d_three] / counts[d_one,d_two]

with open(sys.argv[1],'w') as outfile:
	np.save(outfile,probs)