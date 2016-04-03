#!/usr/bin/env python

"""
Generate notes from an ngram model
USAGE: ./ngram.py MODELFILE NUM_NOTES
"""

import numpy as np 
from collections import deque
import sys,random,pdb
from ngram_helper import pitch_to_str

def generate(counts):

	last_two = deque([60,67])

	generated_notes = []

	for note in xrange(int(sys.argv[2])):
		row = counts[tuple(last_two)]

		total = np.sum(row)

		rand = (random.randint(0,total-1) if total > 0 else 0)

		i = 0
		rand -= row[i]

		while rand > 0:
			i += 1
			rand -= row[i]

		if i > 127:
			pdb.set_trace()

		last_two.append(i)
		last_two.popleft()

		generated_notes.append(i)

	return generated_notes


if __name__ == '__main__':
	
	if len(sys.argv) < 3:
		print(__doc__)
		sys.exit(1)
	with open(sys.argv[1]) as f:
		counts = np.load(f)

	print map(pitch_to_str,generate(counts))