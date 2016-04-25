#!/usr/bin/env python

"""
Generate notes from an ngram model

    $ python -m ngream.generate [-m modeldir] [-k key]

where:
    - `modeldir` is the directory containing the model files
    - `key` is the key the models are recorded in
"""

import numpy as np 
from collections import deque
import sys,random,pdb,os,music21
from ngram_helper import pitch_to_str
from audiolazy import midi2str
from optparse import OptionParser
import ga

DURKS_PER_MEASURE = 32
"""int: Number of durks per measure."""

class NgramGenerator():
    """
    Generate notes using a trigram model. Built as a Python iterator.
    """
    def __init__(self,key,chord_progression,model_dir):
        """
        Initialize our generator
        Args:
            key (music21.key.Key): equal to key model was trained in
            chord_progression (music21.roman.RomanNumeral[]): a series of music21 roman numerals
            model_dir: directory containing the 7 separate trained models
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

    def choose_note(self,rn,last_two):
        """
        Choose the next note to output, given the current Roman Numeral and the last two notes played
        Args:
            rn (music21.roman.RomanNumeral): the current roman numeral
            last_two (int[]): array of last midi pitches played

        Returns:
            music21.pitch.Pitch: the next note to play
        """
        # find the appropriate counts matrix
        mat = self.counts[rn.scaleDegree-1]

        # find the row in our matrix, and sum it.
        row = mat[tuple(last_two)]
        total = np.sum(row)

        # choose a random number between [0,total)
        rand = (random.randint(0,total-1) if total > 0 else 0)

        i = 0
        rand -= row[i]

        # and select the corresponding row in the matrix
        while rand > 0:
            i += 1
            rand -= row[i]

        return music21.pitch.Pitch(midi2str(i))

    def calc_note_val(self,pitch_class):
        """
        Convert note to 1->7 for input into genetic algorithm
        Args:
            pitch_class: pitch class of the note to output

        Returns:
            int: number to input into GA
        """
        # grab the notes in this key's scale
        pitches = self.key.getScale().getChord().pitches

        # and iterate until we find the appropriate pitch class.
        for i in xrange(7):
            if pitches[i].pitchClass == pitch_class:
                return i+1

        # should never happen!
        return 1

    def next(self):
        """
        Generate the next note in the trigram model
        Returns:
            int: the next note to play :)
        """

        # while we still have notes to generate
        if self.durk < len(self.chord_progression) * DURKS_PER_MEASURE:

            # calculate the roman numeral
            rn = self.chord_progression[self.durk / DURKS_PER_MEASURE]

            if rn != self.rn:
                # start of a new chord progression

                self.last_two = deque()

                # append the first, third
                self.last_two.append(rn.pitches[0].midi)
                # TODO: this is not the THIRD!
                self.last_two.append(rn.pitches[2].midi)

            # choose the note, given the last_two
            self.rn = rn
            pitch = self.choose_note(rn,self.last_two)

            # update last_two
            self.last_two.append(pitch.midi)
            self.last_two.popleft()

            # and move forward in time
            self.durk += 4

            # calculate 1->7 value
            val = self.calc_note_val(pitch.pitchClass)
            print self.key, self.rn.figure, pitch, val

            return val
        else:
            raise StopIteration()

def construct_roman_numerals(key,int_seq):
    """
    Given a key and sequence of ints, construct music21.roman.RomanNumeral objects.
    Args:
        key: the key in which the roman numerals belong
        int_seq: sequences of I,II, ... roman numerals

    Returns:
        music21.roman.RomanNumeral[]: the roman numeral objects.
    """
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
    """
    Generate notes for the chord_progression, using the ngram model
    Args:
        model_dir: directory where the ngram models live
        chord_progression: chord progression over which to generate

    Returns:
        int[]: generated midi notes
    """
    return _generate("C",model_dir,chord_progression)

if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("-m", "--model-dir", dest="model_dir")
    parser.add_option("-k", "--key", dest="key", default="C")

    (options, args) = parser.parse_args()

    # default to twelve bar blues
    TWELVE_BAR_BLUES = [1,1,1,1,4,4,1,1,5,4,1,1]

    # calculate ngram output and save to midi file
    ngram_output = generate(options.model_dir,TWELVE_BAR_BLUES)
    ga.create_midi_file((0,ngram_output), ga.create_chord_progression())
    print ngram_output

