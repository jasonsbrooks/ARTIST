from db import Session,Song,Track,Note

session = Session()

import music21

for trk in session.query(Track).all():
    print trk

# in choosing roots for chord-spans, prefer certain TPC-root relationships over others, in the following order:
# 1, 5, 3, b3, b7, b5, b9, ornamental
# see https://en.wikipedia.org/wiki/Interval_(music) for interval defn

prefs = {'P1':7,'P5':6,'P3':5,'m3':4,'m7':3,'m5':2,'m9':1}

def hpr_one(root,note):
    r = music21.note.Note(root.iso_pitch)
    n = music21.note.Note(note.iso_pitch)

    interval = music21.interval.notesToInterval(r,n).simpleName
    return (prefs[interval] if interval in prefs else 0)

