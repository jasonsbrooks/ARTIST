class Song(list):
    """
    A Song with the following properties:

    Properties
        title: String representing title of Song
        tracks: list of Track objects that constitute the song
    """

    def __init__(self, title="", tracks=[]):
        self.title = title
        super(Song, self).__init__(tracks)

    def add_track(self, track):
        """
        adds track object to list of tracks
        """
        self.tracks.append(track)
