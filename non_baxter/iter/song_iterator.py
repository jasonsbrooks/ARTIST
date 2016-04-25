from db import get_sessions,Song,Track,Note
from optparse import OptionParser

class SongIterator(object):
    """
    Iterate through a song, Note by Note
    """
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
        """
        Determine the next Note in the Song
        @return: the next Note
        """
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

    # grab all database sessions
    sessions = get_sessions(options.pool_size,options.db_username,options.db_password)

    # load the song. just print every note
    song = sessions[options.which_db].query(Song).get(options.which_song)
    for note in SongIterator(song):
        print note