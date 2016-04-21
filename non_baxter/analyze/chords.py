from db import Session,Song,Track,Note
from preference_rules import *
from time_iterator import TimeIterator

session = Session()

import music21

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
            note.root = self.root
            note.iso_root = music21.note.Note(self.root).midi

        # label the previous chord span
        if self.prev_cs:
            self.prev_cs.label()

    def pr_score(self,m_root):
        last_ts = self.last_ts()
        ts_notes = last_ts.notes()

        # calculate the beat strength
        stren = beat_strength(ts_notes)

        # compatibility scores
        comp = []
        for note in ts_notes:
            m_note = music21.note.Note(note.iso_pitch)
            comp.append(compatibility(m_root,m_note))
        comp_score = sum(comp) / float(len(ts_notes))

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

def consider_ts(cs,ts):
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

DURK_STEP = 4

def main():
    session = Session()

    cs,idx = None,0

    for song in session.query(Song).all():
        for ts in TimeIterator(song,DURK_STEP):
            cs = consider_ts(cs,ts)
            idx += 1
            print idx, cs.score, ":", cs

    cs.label()
    session.commit()

if __name__ == '__main__':
    main()