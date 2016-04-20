from db import Session,Song,Track,Note
from music21 import lily
import music21

lpc = lily.translate.LilypondConverter()
lpMusicList = lily.lilyObjects.LyMusicList()
lpc.context = lpMusicList

session = Session()

for note in session.query(Track).first().notes:
    if note.dur == 0:
        continue
    m_note = music21.note.Note(note.iso_pitch,quarterLength=note.dur/8.0)
    lpc.appendContextFromNoteOrRest(m_note)

lpc.showPDF()