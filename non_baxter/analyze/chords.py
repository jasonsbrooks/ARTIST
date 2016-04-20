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

# a note is a strong beat if it starts the measure
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

def lof_difference(m_prev,m_note):
    prev_pos = line_of_fifths.index(m_prev.name)
    note_pos = line_of_fifths.index(m_note.name)

    return abs(note_pos - prev_pos)

def best_next_root(m_prev,m_note):

    # start with C, weight of 0
    best_root,best_weight = music21.note.Note('C'),-len(line_of_fifths)

    # try all possible roots
    for m_root in music21.scale.ChromaticScale('C').pitches:

        # compatibility score
        comp = compatibility(m_root,m_note)

        # difference from previous chord root on line of fifths
        lof = (lof_difference(m_prev,m_note) if m_prev else 0)

        val = comp - lof

        if val > best_weight:
            best_root,best_weight = m_root,val

    return best_root,best_weight


for trk in session.query(Track).all():

    # previous note
    m_prev = None

    # transition threshold
    threshold = -len(line_of_fifths)

    for note in trk.notes:
        m_note = music21.note.Note(note.iso_pitch)

        note.root = (m_prev.midi if m_prev else 0)
        note.iso_root = (m_prev.name if m_prev else "")

        # when starts on quarter note
        if beat_strength(note) > 0:
            (best_root,best_weight) = best_next_root(m_prev,m_note)

            if best_weight > threshold:
                # use this as the new root!
                print best_root

                note.root = best_root.midi
                note.iso_root = best_root.name

                m_prev = best_root
            else:
                threshold *= 0.9

    session.commit()

