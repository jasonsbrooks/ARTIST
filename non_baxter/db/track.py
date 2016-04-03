from sqlalchemy import Column, String, Integer, ForeignKey, func
from sqlalchemy.orm import relationship
from . import Base

class Track(Base):
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
        instr_key: Integer representing MIDI instrument key
        instr_name: String (may be empty) representation of instrument
        channel: Integer (0-15) representing MIDI channel
        tempo: (naive) Integer representing tempo marking
        dynamic: (naive) Integer representing dynamic marking (0-127, corresponds to MIDI velocity)
        start_tick: Integer representing start time of track relative to song (i.e. 0 is beginning of song)
        notes: list of Note objects that constitute the track
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
        return "Track(start_tick=%r, ts= %r/%r, ks=%r/%r, instr_key=%r, instr_name=%r, channel=%r, tempo=%r, dynamic=%r, song=%r, notes=%r)" % \
            (self.start_tick, self.time_sig_top, self.time_sig_bottom, self.key_sig_top, self.key_sig_bottom, self.instr_key,
             self.instr_name, self.channel, self.tempo, self.dynamic, self.song, self.notes)
