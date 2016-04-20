from db import Session,Song,Track,Note

session = Session()

import music21

# in choosing roots for chord-spans, prefer certain TPC-root relationships over others, in the following order:
# 1, 5, 3, b3, b7, b5, b9, ornamental
# see https://en.wikipedia.org/wiki/Interval_(music) for interval defn
prefs = ['P1','P5','P3','m3','m7','m5','m9']

# implementing Temperley's HPR 1
def compatibility(m_root,m_note):
    interval = music21.interval.notesToInterval(m_root,m_note).simpleName
    try:
        return len(prefs) - prefs.index(interval)
    except ValueError,e:
        return 0

# the strength of the note corresponds to where it lies relative to 1,1/2,1/4 notes...
def beat_strength(note):
    if note.start % 32 == 0:
        return 3
    elif note.start % 16 == 0:
        return 2
    elif note.start % 8 == 0:
        return 1
    else:
        return 0

line_of_fifths = ["B#","E#","A#","D#","G#","C#","F#","B","E","A","D","G","C","F","B-","E-","A-","D-","G-","C-","F-"]

# return the difference between two notes on the line of fifths.
def lof_difference(m_prev,m_note):
    prev_pos = line_of_fifths.index(m_prev.name)
    note_pos = line_of_fifths.index(m_note.name)

    return abs(note_pos - prev_pos)

# PARAMETERS TO THE MODEL:
COMPATIBILITY_MULTIPLIER = 1
LOF_MULTIPLIER = -1
STRENGTH_MULTIPLIER = 1

THRESHOLD_MULTIPLIER = 0.9

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