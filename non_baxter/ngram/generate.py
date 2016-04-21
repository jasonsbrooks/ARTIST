#!/usr/bin/env python

"""
Generate notes from an ngram model
USAGE: ./ngram.py MODELFILE NUM_NOTES
"""

import numpy as np 
from collections import deque
import sys,random,pdb,os,music21
from ngram_helper import pitch_to_str

TWELVE_BAR_BLUES = [1,1,1,1,4,4,1,1,5,4,1,1]
DURKS_PER_MEASURE = 32

class NgramGenerator():
    def __init__(self,key,chord_progression,model_dir):
        self.durk = 0
        self.key = key
        self.chord_progression = chord_progression
        self.last_two = deque()
        self.rn = self.chord_progression[self.durk]

        # load the various rn models
        self.counts = []
        for i in xrange(7):
            with open(os.path.join(model_dir,str(i+1) + ".npy")) as f:
                self.counts.append(np.load(f))

    def __iter__(self):
        return self

    def choose_note(self,rn,last_two):
        mat = counts[rn-1]

        row = counts[tuple(last_two)]
        total = np.sum(row)

        rand = (random.randint(0,total-1) if total > 0 else 0)

        i = 0
        rand -= row[i]

        while rand > 0:
            i += 1
            rand -= row[i]

        return i

    def calc_note_val(self,note):
        pitches = self.key.getScale().getChord().pitches

        for i in xrange(7):
            if pitches[i].name == note.name:
                return i+1

    def next(self):

        if self.durk < self.chord_progression * DURKS_PER_MEASURE:

            rn = self.chord_progression[self.durk % 32]

            if rn != self.rn:
                self.last_two = deque()

                # append the first, third
                scale = self.key.getScale()
                deque.append(scale.chord.root())
                deque.append(scale.chord.third)

            self.rn = rn
            note = self.choose_note(rn,self.last_two)

            self.last_two.append(note)
            self.last_two.popleft()

            self.durk += 4

            note_name = # CALCULATE NOTE NAMME
            self.calc_note_val(note_name)
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