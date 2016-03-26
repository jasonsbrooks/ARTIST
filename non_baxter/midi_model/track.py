from pprint import pformat


class Track(list):
    """
    A Track represents a line of music in a Song with a single

    1) time signature
    2) key signature
    3) instrument key

    A Track has the following properties:

    Properties
        time_sig: (Integer, Integer) representing (numerator, denominator) of time signature
            Defaults at (4, 4)
        key_sig: (sf, mi): sf = -7 =>  flats, sf = 4 => 4 sharps, mi = 0 => major key, mi = 1 => minor key
            Defaults at (0, 0) => key of C major
        instrument: Integer representing MIDI instrument key
        tempo: (naive) Integer representing tempo marking
        dynamic: (naive) Integer representing dynamic marking (0-127, corresponds to MIDI velocity)
        start_tick: Integer representing start time of track relative to song (i.e. 0 is beginning of song)
        notes: list of Note objects that constitute the track
    """

    def __init__(self, time_sig=(4, 4), key_sig=(0, 0), instrument=0, tempo=120,
                 dynamic="mf", ppqn=100, start_tick=0, notes=[]):
        self.time_sig = time_sig
        self.key_sig = key_sig
        self.instrument = instrument
        self.tempo = tempo
        self.dynamic = dynamic
        self.ppqn = ppqn
        self.start_tick = start_tick
        super(Track, self).__init__(notes)

    def __repr__(self):
        return "Track(start_tick=%r, ts= %r, ks=%r, instr=%r, \\\n  %s)" % \
            (self.start_tick, self.time_sig, self.key_sig, self.instrument, pformat(list(self)).replace('\n', '\n  '), )
