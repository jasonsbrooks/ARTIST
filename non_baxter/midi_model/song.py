from pprint import pformat


class Song(list):
    """
    A Song with the following properties:

    Properties
        title: String representing title of Song
        ppqn: Integer representing "pulses per quarter note" (i.e. 96 ticks/ quarter note)
        tracks: list of Track objects that constitute the song
    """

    def __init__(self, title="", ppqn=120, tracks=[]):
        self.title = title
        self.ppqn = ppqn
        super(Song, self).__init__(tracks)

    def __repr__(self):
        return "Song(title=%r, ppqn=%r, tracks=\\\n%s)" % \
            (self.title, self.ppqn, pformat(list(self)))
