from db import Session,Song,Track,Note
from music21 import lily
import music21

lpc = lily.translate.LilypondConverter()
lpMusicList = lily.lilyObjects.LyMusicList()
lpc.context = lpMusicList

session = Session()

chord = []

for note in session.query(Track).first().notes:

    if len(chord) == 0:
        chord.append(note)
    elif note.start == chord[-1].start:
        chord.append(note)
    else:
        ql = note.dur / 8.0
        if ql < 0.25:
            continue
        ql = round(ql,1)

        try:
            ch = music21.chord.Chord([n.iso_pitch for n in chord],quarterLength=ql)
        except Exception,e:
            chord = []
            continue

        chord = []

        try:
            lpc.appendM21ObjectToContext(ch)
            print ch
        except Exception,e:
            continue

# lpc.writeLyFile(ext="ly",fp="out.ly")
with open("out.ly","w") as outfile:
    outfile.write(str(lpMusicList))