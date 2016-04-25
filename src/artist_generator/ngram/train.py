#!/usr/bin/env python

"""
Train an ngram model

    $ python -m ngram.train -o outdir [-t poolsize] [-k key] [-u username] [-p password]

where:
    - outdir is where the trained models will be saved
    - poolsize is the number of databases
    - key is the key to save the models in
    - username is the database username
    - password is the database password
"""

import numpy as np
import sys, os, re, music21
from optparse import OptionParser
from multiprocessing import Process

from collections import deque

from sqlalchemy import desc, asc

from db import Song, Track, Note, get_sessions, InvalidKeySignature
from ngram_helper import key_transpose_pitch

NUM_NOTES = 128

class RomanTrainer(object):
    """
    A RomanTrainer is the model trainer

    1. for a given process / database, and
    2. for a given roman numeral.
    """
    def __init__(self,p_id,rt_id,counts,options):
        """
        Initialize the RomanTrainer
        Args:
            p_id: process id
            rt_id: roman numeral id
            counts: counts matrix
            options: options passed into script
        """
        self.p_id = p_id
        self.rt_id = rt_id

        self.counts = counts
        self.triple = deque()
        self.options = options

        # assume the user has specified a major key
        self.dest_key = (music21.key.Key(options.key).sharps,0)

    def transposed_triple(self):
        """
        Transpose a triple into the appropriate key
        Returns:
            int[]: the transposed triple
        """
        res = []
        notes = list(self.triple)
        for note in notes:
            src_key = (note.track.key_sig_top,note.track.key_sig_bottom)
            res.append(key_transpose_pitch(note.pitch,src_key,self.dest_key))
	return res

    def train(self,note):
        """
        Train this RomanTrained on a given note
        Args:
            note: the note to train on
        """
        self.triple.append(note)

        if len(self.triple) > 3:
            # remove the old note
            old_note = self.triple.popleft()
            try:
                # increment the matrix, where appropriate
                np.add.at(self.counts, tuple(self.transposed_triple()), 1)
            except InvalidKeySignature, e:
                # remove the bad note, append the old note.
                self.triple.pop()
                self.triple.appendleft(old_note)

    def write(self):
        """
        Write the numpy counts matrix out to file.
        """
        with open(os.path.join(self.options.outdir,str(self.p_id),str(self.rt_id) + ".npy"), 'w') as outfile:
            np.save(outfile, self.counts)

class TrackTrainer(Process):
    """
    Separate process to train ngram models, all music sourcing from one database
    """
    def __init__(self,p_id,session,options):
        """
        Initialize the TrackTrainer process
        Args:
            p_id: process id
            session: the database session to load from
            options (dict): options passed to script
        """
        Process.__init__(self)
        self.session = session
        self.options = options

        self.rts = []
        matrix_size = (NUM_NOTES, NUM_NOTES, NUM_NOTES)

        # construct the roman trainers
        for i in xrange(7):
            rt = RomanTrainer(p_id,i + 1,np.zeros(matrix_size, dtype=np.int16),options)
            self.rts.append(rt)

    def run(self):
        """
        Start the process, training on each track separately
        """
        # iterate through all the tracks
        for trk in self.session.query(Track).all():
            self.train(trk)

        # write all the rts
        for rt in self.rts:
            rt.write()

    def train(self,trk):
        """
        Train the ngram model on a specific track
        Args:
            trk: the track on which to train
        """
        print os.path.basename(trk.song.title), ":", trk.instr_name

        # skip percurssion tracks
        regexp = re.compile(r'drum|cymbal', re.IGNORECASE)
        if trk.channel == 9 or regexp.search(trk.instr_name) is not None:
            # print 'skipped percussion track'
            return

        # skip bass tracks
        regexp = re.compile(r'bass', re.IGNORECASE)
        if (trk.channel >= 32 and trk.channel <= 39) or regexp.search(trk.instr_name) is not None:
            # print 'skipped bass track'
            return

        # and through all the notes in a track
        for note in trk.notes:
            if note.pitch < 0 or note.pitch >= NUM_NOTES:
                pass

            # train using the appropriate rt
            if note.roman:
                self.rts[note.roman-1].train(note)

def main():
    parser = OptionParser()

    parser.add_option("-o", "--outdir", dest="outdir")
    parser.add_option("-t", "--poolsize", dest="pool_size", default=8, type="int")
    parser.add_option("-k", "--key", dest="key", default="C")
    parser.add_option("-u", "--username", dest="db_username", default="postgres")
    parser.add_option("-p", "--password", dest="db_password", default="postgres")

    (options, args) = parser.parse_args()

    # make the process output directory if not there already
    for p_id in xrange(options.pool_size):
	print options.outdir
    	pt = os.path.join(options.outdir,str(p_id) + "/")
	print pt
        if not os.path.exists(pt):
            os.mkdir(pt)

    sessions = get_sessions(options.pool_size,options.db_username,options.db_password)
    processes = []

    # construct and start the threads
    for i in xrange(options.pool_size):
        p = TrackTrainer(str(i),sessions[i],options)
        processes.append(p)
        p.start()

    # wait for processes to complete
    for p in processes:
        p.join()

    # construct cumulative counts matrices
    matrix_size = (NUM_NOTES, NUM_NOTES, NUM_NOTES)
    cumulative_counts = []

    for i in xrange(7):
        cumulative_counts.append(np.zeros(matrix_size, dtype=np.int16))

    for p_id in xrange(options.pool_size):
        for rt_id in xrange(7):
            with open(os.path.join(options.outdir + "/",str(p_id) + "/",str(rt_id + 1) + ".npy")) as f:
                counts = np.load(f)
                cumulative_counts[rt_id] = np.add(cumulative_counts[rt_id],counts)

    for i in xrange(7):
        with open(os.path.join(options.outdir + "/",str(i+1) + ".npy"), "w") as f:
            np.save(f,cumulative_counts[i])

if __name__ == '__main__':
    main()
