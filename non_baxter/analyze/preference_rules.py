import music21

# in choosing roots for chord-spans, prefer certain TPC-root relationships over others, in the following order:
# 1, 5, 3, b3, b7, b5, b9, ornamental
# see https://en.wikipedia.org/wiki/Interval_(music) for interval defn
comp_prefs = ['P1','P5','P3','m3','m7','m5','m9']

# implementing Temperley's HPR 1
def _dual_compatibility(m_root,m_note):
    interval = music21.interval.notesToInterval(m_root,m_note).simpleName
    try:
        return len(comp_prefs) - comp_prefs.index(interval)
    except ValueError,e:
        return 0

def compatibility(notes,m_root):
    if not notes:
        return 0

    comp = []
    for note in notes:
        m_note = music21.note.Note(note.iso_pitch)
        comp.append(_dual_compatibility(m_root,m_note))
    return sum(comp) / float(len(notes))

# the strength of the note corresponds to where it lies relative to 1,1/2,1/4 notes...
def beat_strength(notes):
    val = 0
    for note in notes:
        if note.start % 32 == 0:
            val += 3
        elif note.start % 16 == 0:
            val += 2
        elif note.start % 8 == 0:
            val += 1
        else:
            val += 0
    return val

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

