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
def strong_beat(note):
    return (note.start % 32 == 0)

line_of_fifths = ["B#","E#","A#","D#","G#","C#","F#","B","E","A","D","G","C","F","B-","E-","A-","D-","G-","C-","F-"]

def lof_difference(m_prev,m_note):
    prev_pos = line_of_fifths.index(m_prev.name)
    note_pos = line_of_fifths.index(m_note.name)

    return abs(note_pos - prev_pos)

w1 = 1
w2 = 1
w3 = 1

for trk in session.query(Track).all():
    m_prev,lof = None,1
    for note in trk.notes:

        best_root,best_weight = music21.note.Note('C'),0

        for m_root in music21.scale.ChromaticScale('C').pitches:
            m_note = music21.note.Note(note.iso_pitch)

            comp = compatibility(m_root,m_note)
            stren = strong_beat(note)
            if m_prev:
                lof = lof_difference(m_prev,m_note)

            val = w1 * comp + w2 * stren + w3 * lof

            if val > best_weight:
                best_root,best_weight = m_root,val


        m_prev = best_root
