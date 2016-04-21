from db import Session,Song,Track,Note
from preference_rules import pr_score,line_of_fifths
from time_iterator import TimeIterator

session = Session()

import music21

class ChordSpan(object):
    def __init__(self,tss,prev_cs):
        self.tss = tss

        # a back-pointer to the previous best chord-span
        self.prev_cs = prev_cs
        # TODO: how to calculate next value?
        self.val = self.prev_cs.val +

    def calc_min_max(self):
        self.start = min(self.tss,key=lambda ts: ts.time)
        self.end = max(self.tss,key=lambda ts: ts.time)

    def add(self,ts):
        self.tss.append(ts)
        self.calc_min_max()

    def remove(self,ts):
        self.tss.remove(ts)
        self.calc_min_max()

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

    def calc_best_root(self):

        # start with C, weight of 0
        best_root,best_weight = music21.note.Note('C'),-len(line_of_fifths)

        # try all possible roots
        for m_root in music21.scale.ChromaticScale('C').pitches:

            val = pr_score(self,m_root)

            if val > best_weight:
                best_root,best_weight = m_root,val

        # use this as the chord-span root
        self.root = best_root
        return best_weight

DURK_STEP = 4

def main():
    session = Session()

    cs = None

    for song in session.query(Song).all():
        for ts in TimeIterator(song,DURK_STEP):

            if not cs:
                cs = ChordSpan(ts,None)
                score = cs.calc_best_root()
            else:
                # a new segment
                cs = ChordSpan(ts,cs)

                # try other segments.



if __name__ == '__main__':
    main()