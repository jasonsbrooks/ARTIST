#!/usr/bin/env python

"""
Generate notes from an ngram model
USAGE: ./ngram.py MODELFILE NUM_NOTES
"""

import numpy as np 
from collections import deque
import sys,random,pdb,os,music21
from ngram_helper import pitch_to_str
from audiolazy import midi2str
from optparse import OptionParser
import ga

DURKS_PER_MEASURE = 32

class NgramGenerator():
    def __init__(self,key,chord_progression,model_dir):
        """
        @param key: music21 key. equal to key model was trained in
        @param chord_progression: series of music21 roman numerals
        @param model_dir: directory containing the 7 separate trained models
        @return:
        """
        self.durk = 0
        self.key = key
        self.chord_progression = chord_progression
        self.last_two = deque()
        self.rn = None

        # load the various rn models
        self.counts = []
        for i in xrange(7):
            with open(os.path.join(model_dir,str(i+1) + ".npy")) as f:
                self.counts.append(np.load(f))

    def __iter__(self):
        return self

    def _choose_note_once(self,rn,last_two):
        mat = self.counts[rn.scaleDegree-1]

        row = mat[tuple(last_two)]
        total = np.sum(row)

        rand = (random.randint(0,total-1) if total > 0 else 0)

        i = 0
        rand -= row[i]

        while rand > 0:
            i += 1
            rand -= row[i]

        return music21.pitch.Pitch(midi2str(i))

    def choose_note(self,rn,last_two,depth=0):
        # failed too many times. just play the root of the scale
        # if depth >= sys.getrecursionlimit() * 0.75:
        #     return self.key.getScale().chord.root()

        # try generating a pitch.
        gen = self._choose_note_once(rn,last_two)
        return gen
        # for pitch in rn.pitches:
        #     # ensure the pitch is in the rn
        #     if gen.pitchClass == pitch.pitchClass:
        #         return gen
        #
        # # failed. try again to generate
        # return self.choose_note(rn,last_two,depth+1)

    # convert note to 1->7 for GA
    def calc_note_val(self,pitch_class):
        pitches = self.key.getScale().getChord().pitches

        for i in xrange(7):
            if pitches[i].pitchClass == pitch_class:
                return i+1

        # should never happen!
        return 1

    def next(self):

        if self.durk < len(self.chord_progression) * DURKS_PER_MEASURE:

            rn = self.chord_progression[self.durk / DURKS_PER_MEASURE]

            if rn != self.rn:
                self.last_two = deque()

                # append the first, third
                self.last_two.append(rn.pitches[0].midi)
                # TODO: this is not the THIRD!
                self.last_two.append(rn.pitches[2].midi)

            self.rn = rn
            pitch = self.choose_note(rn,self.last_two)

            # update last_two
            self.last_two.append(pitch.midi)
            self.last_two.popleft()

            self.durk += 4

            # calculate 1->7 value
            val = self.calc_note_val(pitch.pitchClass)
            print self.key, self.rn.figure, pitch, val

            return val
        else:
            raise StopIteration()

def construct_roman_numerals(key,int_seq):
    rns = []
    for i in int_seq:
        rns.append(music21.roman.RomanNumeral(i,key))
    return rns

def _generate(o_key,o_model_dir,cp):
    key = music21.key.Key(o_key)
    cp = construct_roman_numerals(key,cp)
    generator = NgramGenerator(key,cp,o_model_dir)

    return [(note,4) for note in generator]

def generate(model_dir,chord_progression):
    return _generate("C",model_dir,chord_progression)

if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("-m", "--model-dir", dest="model_dir")
    parser.add_option("-k", "--key", dest="key", default="C")

    (options, args) = parser.parse_args()

    TWELVE_BAR_BLUES = [1,1,1,1,4,4,1,1,5,4,1,1]

    ngram_output = generate(options.model_dir,TWELVE_BAR_BLUES)
    ga.create_midi_file((0,ngram_output), ga.create_chord_progression())
    print ngram_output

