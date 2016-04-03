#!/usr/bin/env python

"""
Generate notes from an ngram model
USAGE: ./ngram.py MODELFILE NUM_NOTES
"""

import numpy as np 
from collections import deque
import sys,random,pdb
from ngram_helper import pitch_to_str

class NgramGenerator():
    def __init__(self, first_two, num_notes):
        self.num_generated = 0
        self.num_notes = num_notes

        self.last_two = deque(first_two)

    def __iter__(self):
        return self

    def next(self):

        if self.num_generated < self.num_notes:

            row = counts[tuple(self.last_two)]
            total = np.sum(row)

            rand = (random.randint(0,total-1) if total > 0 else 0)

            i = 0
            rand -= row[i]

            while rand > 0:
                i += 1
                rand -= row[i]

            self.last_two.append(i)
            self.last_two.popleft()

            self.num_generated += 1

            return i
        else:
            raise StopIteration()

def generate(counts, num_notes):

    first_two = [60,67]

    return [note for note in NgramGenerator(first_two,num_notes)]


if __name__ == '__main__':
    
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    with open(sys.argv[1]) as f:
        counts = np.load(f)

    print map(pitch_to_str,generate(counts, int(sys.argv[2])))