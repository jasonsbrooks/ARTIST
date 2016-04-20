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
        if len(self.notes) == 0 or (self.dur == note.dur and self.start == note.start):
            self.notes.append(note)
            return note
        else:
            return False

class Chords(object):
    def __init__(self,session,song):
        self.session = session

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

        print self.chords


session = Session()

song = session.query(Song).first()
ch = Chords(session,song)