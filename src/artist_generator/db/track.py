from sqlalchemy import Column, String, Integer, ForeignKey, func
from sqlalchemy.orm import relationship
from . import Base

class Track(Base):
    """
    A Track represents a line of music in a Song with a single

    1) time signature
    2) key signature
    3) instrument key

    Attributes:
        time_sig (int, int): representing (numerator, denominator) of time signature
            Defaults at (4, 4)

        key_sig: (sf, mi): sf = -7 =>  flats, sf = 4 => 4 sharps, mi = 0 => major key, mi = 1 => minor key
            Defaults at (0, 0) => key of C major

        instr_key (int): Integer representing MIDI instrument key

        instr_name (str): String (may be empty) representation of instrument

        channel (int): Integer (0-15) representing MIDI channel

        tempo (int): (naive) Integer representing tempo marking

        dynamic (int): (naive) Integer representing dynamic marking (0-127, corresponds to MIDI velocity)

        start_tick (int): Integer representing start time of track relative to song (i.e. 0 is beginning of song)

        notes (Note[]): list of Note objects that constitute the track
            Note: notes are NOT in chronological order by start
    """

    __tablename__ = 'track'

    id = Column(Integer, primary_key=True)
    time_sig_top = Column(Integer, nullable=False)
    time_sig_bottom = Column(Integer, nullable=False)
    key_sig_top = Column(Integer, nullable=False)
    key_sig_bottom = Column(Integer, nullable=False)
    instr_key = Column(Integer, nullable=False)
    instr_name = Column(String, nullable=False)
    channel = Column(Integer, nullable=False)
    tempo = Column(Integer, nullable=True, default=0)
    dynamic = Column(Integer, nullable=True, default=0)
    start_tick = Column(Integer, nullable=False)
    song_id = Column(Integer, ForeignKey('song.id'))
    song = relationship("Song", back_populates="tracks")
    notes = relationship("Note")

    def __repr__(self):
        return "Track(start_tick=%r, ts= %r/%r, ks=%r/%r, instr_key=%r, instr_name=%r, channel=%r, tempo=%r, dynamic=%r, song=%r, len(notes)=%r)" % \
            (self.start_tick, self.time_sig_top, self.time_sig_bottom, self.key_sig_top, self.key_sig_bottom, self.instr_key,
             self.instr_name, self.channel, self.tempo, self.dynamic, self.song, len(self.notes))
