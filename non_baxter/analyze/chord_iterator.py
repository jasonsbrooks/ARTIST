from db import Session,Song,Track,Note

class Chord(object):
    """
    Chord: a list of notes
    ~ starting at the same time
    ~ lasting the same time
    """
    def __init__(self):
        self.notes = []
        self.start = -1
        self.dur = -1

    def add(self,note):
        if len(self.notes) == 0:
            self.dur = note.dur
            self.start = note.start
            self.notes.append(note)
            return note
        elif self.dur == note.dur and self.start == note.start:
            self.notes.append(note)
            return note
        else:
            return False

class Chords(object):
    def __init__(self,session,song):
        self.session = session
        self.indx = 0

        # get all the notes in the piece
        all_notes = []
        for trk in song.tracks:
            for note in trk.notes:
                all_notes.append(note)

        all_notes.sort(key=lambda n: (n.start,n.dur))

        # construct the array of chords
        chord = Chord()
        self.chords = [chord]

        # iterate through all the ntoes
        for note in all_notes:
            if not chord.add(note):

                # construct next chord
                chord = Chord()
                self.chords.append(chord)
                chord.add(note)

    def __iter__(self):
        return self

    def next(self):
        # stop iteration!
        if self.indx >= len(self.chords):
            raise StopIteration()

        # grab the chord and increment index
        chord = self.chords[self.indx]
        self.indx += 1

        return chord.notes

if __name__ == '__main__':
    session = Session()

    song = session.query(Song).first()
    for chord in Chords(session,song):
        print chord