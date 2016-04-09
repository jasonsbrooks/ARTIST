#!/usr/bin/env python

"""
Train an ngram model
USAGE: ./ngram.py OUTFILE
"""
import numpy as np
import sys, os, threading

if len(sys.argv) < 2:
	print(__doc__)
	sys.exit(1)

THREAD_POOL_SIZE = 12

NUM_NOTES = 128
matrix_size = (NUM_NOTES,NUM_NOTES,NUM_NOTES)

counts = np.zeros(matrix_size,dtype=np.int16)

from collections import deque
from Queue import Queue

from sqlalchemy import desc, asc

from db import Session, Song, Track, Note

class TrackTrainer(threading.Thread):
	def __init__(self,q,counts):
		threading.Thread.__init__(self)
		self.session = Session()
		self.q = q
		self.counts = counts

	def run(self):
		while True:
			trk_id = self.q.get()
			self.train(trk_id)
			self.q.task_done()

	def train(self,trk_id):
		trk = self.session.query(Track).get(trk_id)
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
				np.add.at(self.counts,tuple(triple),1)

session = Session()
q = Queue()

# iterate through all the tracks
for trk in session.query(Track).all():
	q.put(trk.id)

for i in xrange(THREAD_POOL_SIZE):
	thrd = TrackTrainer(q,counts)
	thrd.daemon = True
	thrd.start()

with open(sys.argv[1],'w') as outfile:
	np.save(outfile,counts)