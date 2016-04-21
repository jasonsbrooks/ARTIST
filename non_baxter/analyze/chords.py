from db import Session,Song,Track,Note
from iter import TimeIterator

from preference_rules import *

import music21,threading
from Queue import Queue
from optparse import OptionParser

class ChordSpan(object):
    def __init__(self,initial_ts,prev_cs):
        self.tss = [initial_ts]
        self.root = None

        # a back-pointer to the previous best chord-span
        self.prev_cs = prev_cs

    def __repr__(self):
        return "<ChordSpan: root=%r>" % (self.root)

    def last_ts(self):
        return max(self.tss,key=lambda ts: ts.time)

    def add(self,ts):
        self.tss.append(ts)

    def remove(self,ts):
        self.tss.remove(ts)

    def notes(self):
        res = []
        # iterate through all chords
        for ts in self.tss:
            # all notes in this time instance
            for note in ts.notes():
                res.append(note)
        return res

    def label(self):
        # label all the notes in this chord span
        for note in self.notes():
            if self.root:
                note.root = self.root.name
                note.iso_root = self.root.midi

        # label the previous chord span
        if self.prev_cs:
            self.prev_cs.label()

    def pr_score(self,m_root):
        last_ts = self.last_ts()
        ts_notes = last_ts.notes()

        # calculate the beat strength
        stren = beat_strength(ts_notes)

        # compatibility scores
        comp_score = compatibility(ts_notes,m_root)

        # difference from previous chord root on line of fifths
        lof = (lof_difference(self.prev_cs.root,m_root) if self.prev_cs else 0)

        return STRENGTH_MULTIPLIER * stren + COMPATIBILITY_MULTIPLIER * comp_score + LOF_MULTIPLIER * lof

    def calc_best_root(self):

        # start with C, weight of 0
        best_root,best_weight = music21.note.Note('C'),-len(line_of_fifths)

        # try all possible roots
        for m_root in music21.scale.ChromaticScale('C').pitches:

            val = self.pr_score(m_root)

            if val > best_weight:
                best_root,best_weight = m_root,val

        # use this as the chord-span root
        self.root = best_root

        prev_cs_score = (self.prev_cs.score if self.prev_cs else 0)
        return prev_cs_score + best_weight

class HarmonicAnalyzer(threading.Thread):
    def __init__(self,q,durk_step):
        threading.Thread.__init__(self)
        self.q = q
        self.durk_step = durk_step
        self.session = Session()

    def run(self):
        while True:
            song_id = self.q.get()

            # grab and analyze the song
            song = self.session.query(Song).get(song_id)
            self.analyze(song)

            # this task is complete
            self.q.task_done()

    def analyze(self,song):
        cs,idx = None,0
        print "#"*80, "\n", song ,"\n", "#"*80, "\n"

        for ts in TimeIterator(song,self.durk_step):
            cs = self.consider_ts(cs,ts)
            idx += 1
            print idx, ts, "--", cs.score, ":", cs

        cs.label()
        self.session.commit()


    def consider_ts(self,cs,ts):
        if not cs:
            res = ChordSpan(ts,None)
            score = res.calc_best_root()
        else:
            # option 1: start a new chord-span
            opt1_cs = ChordSpan(ts,cs)
            opt1_score = cs.calc_best_root()

            # option 2: add to prior segment
            cs.add(ts)
            opt2_score = cs.calc_best_root()

            if opt1_score > opt2_score:
                cs.remove(ts)
                res = opt1_cs
                score = opt1_score
            else:
                res = cs
                score = opt2_score

        # set the score on this cs
        res.score = score

        return res

def main():
    parser = OptionParser()

    parser.add_option("-d", "--durk-step", dest="durk_step", default=4, type="int")
    parser.add_option("-t", "--pool-size", dest="thread_pool_size", default=8, type="int")

    (options, args) = parser.parse_args()

    print "thread_pool_size", options.thread_pool_size
    print "durk_step", options.durk_step

    session = Session()
    q = Queue()

    for song in session.query(Song).all():
        q.put(song.id)

    for i in xrange(options.thread_pool_size):
        thrd = HarmonicAnalyzer(q,options.durk_step)
        thrd.daemon = True
        thrd.start()

    q.join()

if __name__ == '__main__':
    main()