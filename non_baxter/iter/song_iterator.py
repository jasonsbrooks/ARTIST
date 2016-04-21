from db import Session,Song,Track,Note

class SongIterator(object):
    def __init__(self,song):
        self.indx = 0

        # get all the notes in the piece
        self.notes = []
        for trk in song.tracks:
            for note in trk.notes:
                self.notes.append(note)

        self.notes.sort(key=lambda n: (n.start,n.dur))

    def __iter__(self):
        return self

    def next(self):
        # stop iteration!
        if self.indx >= len(self.notes):
            raise StopIteration()

        # grab the note and increment index
        note = self.notes[self.indx]
        self.indx += 1

        # ignore rests
        if note.pitch == -1:
            return self.next()
        else:
            return note

if __name__ == '__main__':
    session = Session()

    song = session.query(Song).first()
    for note in SongIterator(song):
        print note