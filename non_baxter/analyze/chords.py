from db import Session,Song,Track,Note

session = Session()

import music21

class ChordSpan(object):
    def __init__(self,tss,root,prev_cs):
        self.start = min(tss,key=lambda ts: ts.time)
        self.end = max(tss,key=lambda ts: ts.time)
        self.tss = tss
        self.root = root

        # a back-pointer to the previous best chord-span
        self.prev_cs = prev_cs
        # TODO: how to calculate next value?
        self.val = self.prev_cs.val +

    def notes(self):
        res = []
        # iterate through all chords
        for ts in self.tss:
            # all notes in this time instance
            for note in ts.notes():
                res.append(note)
        return res

    def label(self):
        for note in self.notes():
            note.root = self.root
            note.iso_root = music21.note.Note(self.root).midi

def best_next_root(m_prev,chord):

    # calculate the beat strength
    stren = beat_strength(chord[0])

    # start with C, weight of 0
    best_root,best_weight = music21.note.Note('C'),-len(line_of_fifths)

    # try all possible roots
    for m_root in music21.scale.ChromaticScale('C').pitches:

        # compatibility scores
        comp = []
        for note in chord:
            m_note = music21.note.Note(note.iso_pitch)
            comp.append(compatibility(m_root,m_note))
        comp_score = sum(comp) / float(len(chord))

        # difference from previous chord root on line of fifths
        lof = (lof_difference(m_prev,m_root) if m_prev else 0)

        val = STRENGTH_MULTIPLIER * stren + COMPATIBILITY_MULTIPLIER * comp_score + LOF_MULTIPLIER * lof

        if val > best_weight:
            best_root,best_weight = m_root,val

    return best_root,best_weight

def main():
    chord = []

    for trk in session.query(Track).all():

        # previous note
        m_prev = None

        # transition threshold
        threshold = -len(line_of_fifths)

        # iterate through all notes in this track
        for note in sorted(trk.notes,key=lambda note: note.start):

            # initialize root
            note.root = m_prev.midi if m_prev else 0
            note.iso_root = m_prev.name if m_prev else ""

            if len(chord) == 0:
                chord.append(note)
            elif note.start == chord[-1].start:
                chord.append(note)
            else:
                #     process the chord
                best_root,best_weight = best_next_root(m_prev,chord)

                # update node.root if better than threshold
                if best_weight > threshold:
                    note.root = best_root.midi
                    note.iso_root = best_root.name
                    m_prev,threshold = best_root,best_weight
                    print note.iso_root
                else:
                    threshold *= THRESHOLD_MULTIPLIER

                chord = []

        session.commit()

if __name__ == '__main__':
    main()