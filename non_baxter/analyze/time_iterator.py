from db import Session,Song,Track,Note

from song_iterator import SongIterator
from chord_iterator import ChordIterator

class TimeInstance(object):
    def __init__(self,time,chords):
        self.time = time
        self.chords = chords
    def notes(self):
        res = []
        for chord in self.chords:
            res.append(chord.notes)
        return res

class TimeIterator(object):
    def __init__(self,song,durk_step):
        self.durk_step = durk_step
        self.chord_iterator = ChordIterator(song)

        self.current_ts = None
        self.chords_to_consider = []

        # start at min_time. end at max_time
        all_notes = SongIterator(song).notes
        first_note = min(all_notes, key=lambda note: note.start + note.dur)
        self.time = first_note.start
        last_note = max(all_notes,key=lambda note: note.start + note.dur)
        self.max_time = last_note.start + last_note.dur

    def __iter__(self):
        return self

    def next(self):
        if self.time >= self.max_time:
            raise StopIteration()
        else:
            chords = []

            if self.current_ts:
                self.chords_to_consider += self.current_ts.chords

            next_chord = self.chord_iterator.next()
            self.chords_to_consider.append(next_chord)
            while next_chord.start <= self.time + self.durk_step:
                next_chord = self.chord_iterator.next()
                self.chords_to_consider.append(next_chord)

            for chord in self.chords_to_consider:
                if chord.on_at_time(self.time,self.durk_step):
                    chords.append(chord)
                    self.chords_to_consider.remove(chord)

            self.current_ts = TimeInstance(self.time,chords)
            self.time += self.durk_step

            return self.current_ts

session = Session()

song = session.query(Song).first()
for ts in TimeIterator(song,4):
    print ts.chords