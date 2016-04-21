#!/usr/bin/env python

"""
Train an ngram model
USAGE: ./ngram.py THREAD_POOL_SIZE OUTFILE_NAME
"""

'''
Todo:
1) key transposition (multiple ngram models or transpose into one?)
2) skip bass/rhythm/strings/piano?


Complete:
1) Skip all drums

'''
import numpy as np
import sys, os, threading, re
from optparse import OptionParser

from collections import deque
from Queue import Queue

from sqlalchemy import desc, asc

from db import Session, Song, Track, Note

NUM_NOTES = 128

class RomanTrainer(object):
    def __init__(self,name,counts,options):
        self.name = name
        self.counts = counts
        self.triple = deque()
        self.options = options

    def train(self,note):
        self.triple.append(note.pitch)

        if len(self.triple) > 3:
            self.triple.popleft()
            np.add.at(self.counts, tuple(self.triple), 1)

    def write(self):
        with open(os.path.join(self.options.outdir,str(self.name) + ".npy"), 'w') as outfile:
            np.save(outfile, self.counts)

class TrackTrainer(threading.Thread):
    def __init__(self,q,rts):
        threading.Thread.__init__(self)
        self.session = Session()
        self.q = q
        self.rts = rts

    def run(self):
        while True:
            trk_id = self.q.get()
            self.train(trk_id)
            self.q.task_done()

    def train(self,trk_id):
        trk = self.session.query(Track).get(trk_id)
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
    parser.add_option("-t", "--poolsize", dest="thread_pool_size", default=8, type="int")

    (options, args) = parser.parse_args()

    print "outdir", options.outdir
    print "thread_pool_size", options.thread_pool_size

    matrix_size = (NUM_NOTES, NUM_NOTES, NUM_NOTES)

    session = Session()
    q = Queue()

    rts = []
    # construct the roman trainers
    for i in xrange(7):
        rt = RomanTrainer(i + 1,np.zeros(matrix_size, dtype=np.int16),options)
        rts.append(rt)

    # iterate through all the tracks
    for trk in session.query(Track).all():
        q.put(trk.id)

    # construct and start the threads
    for i in xrange(options.thread_pool_size):
        thrd = TrackTrainer(q,rts)
        thrd.daemon = True
        thrd.start()

    q.join()

    for rt in rts:
        rt.write()

if __name__ == '__main__':
    main()
