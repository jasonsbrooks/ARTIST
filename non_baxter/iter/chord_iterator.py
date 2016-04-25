from db import get_sessions,Song,Track,Note
from song_iterator import SongIterator
from optparse import OptionParser

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
        """
        Add a note to this chord, after verifying that it has the same start time / duration
        @param note: the Note object to add
        @return: the note if added, False if not added
        """
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

    def end_time(self):
        """
        Compute the end time of this chord
        @return: start + dur
        """
        return self.start + self.dur

    def on_at_time(self,time):
        """
        Determine whether this note is on at a given time
        @param time: the time to check
        @return: True if yes, False if no
        """
        return (self.start <= time) and (time <= self.end_time())

class ChordIterator(object):
    """
    Iterate through a song, Chord by Chord.
    """
    def __init__(self,song):
        self.indx = 0

        # construct the array of chords
        chord = Chord()
        self.chords = [chord]

        # iterate through all the ntoes
        for note in SongIterator(song):
            if not chord.add(note):

                # construct next chord
                chord = Chord()
                self.chords.append(chord)
                chord.add(note)

    def __iter__(self):
        return self

    def next(self):
        """
        Determines and returns the next chord
        @return: the next chord
        """
        # stop iteration!
        if self.indx >= len(self.chords):
            raise StopIteration()

        # grab the chord and increment index
        chord = self.chords[self.indx]
        self.indx += 1

        return chord

# when this file is run directly: used for debugging!
if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("-d", "--durk-step", dest="durk_step", default=4, type="int")
    parser.add_option("-t", "--pool-size", dest="pool_size", default=8, type="int")
    parser.add_option("-u", "--username", dest="db_username", default="postgres")
    parser.add_option("-p", "--password", dest="db_password", default="postgres")
    parser.add_option("-b", "--db", dest="which_db", default=0, help="Which database # to draw from?")
    parser.add_option("-s", "--song", dest="which_song", default=1, help="Which song to debug with?")
    (options, args) = parser.parse_args()

    # get all database sessions
    sessions = get_sessions(options.pool_size,options.db_username,options.db_password)

    # which song to debug with
    song = sessions[options.which_db].query(Song).get(options.which_song)

    # just print out the chords
    for chord in ChordIterator(song):
        print chord.notes