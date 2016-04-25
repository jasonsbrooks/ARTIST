from sqlalchemy import Column, String, Integer, ForeignKey, func
from sqlalchemy.orm import relationship
from . import Base

class Track(Base):
    """
    A Track represents a line of music in a Song with a single

    1) time signature
    2) key signature
    3) instrument key
    """

    __tablename__ = 'track'

    id = Column(Integer, primary_key=True)
    """int: primary key"""

    time_sig_top = Column(Integer, nullable=False)
    """int: numerator of time signature"""
    time_sig_bottom = Column(Integer, nullable=False)
    """int: denominator of time signature"""

    key_sig_top = Column(Integer, nullable=False)
    """int: number of sharps in the key signature"""

    key_sig_bottom = Column(Integer, nullable=False)
    """int: 0 => major key, 1 => minor key"""

    instr_key = Column(Integer, nullable=False)
    """int: Integer representing MIDI instrument key"""

    instr_name = Column(String, nullable=False)
    """str: String (may be empty) representation of instrument"""

    channel = Column(Integer, nullable=False)
    """int: Integer (0-15) representing MIDI channel"""

    tempo = Column(Integer, nullable=True, default=0)
    """int: (naive) Integer representing tempo marking"""

    dynamic = Column(Integer, nullable=True, default=0)
    """int: (naive) Integer representing dynamic marking (0-127, corresponds to MIDI velocity)"""

    start_tick = Column(Integer, nullable=False)
    """int: Integer representing start time of track relative to song (i.e. 0 is beginning of song)"""

    song_id = Column(Integer, ForeignKey('song.id'))
    """int: id of the Song to which this Track belongs"""

    song = relationship("Song", back_populates="tracks")
    """Song: song to which this Track belongs"""

    notes = relationship("Note")
    """Notes[]: list of Note objects that constitute the track
    
            Note: notes are NOT in chronological order by start"""

    def __repr__(self):
        return "Track(start_tick=%r, ts= %r/%r, ks=%r/%r, instr_key=%r, instr_name=%r, channel=%r, tempo=%r, dynamic=%r, song=%r, len(notes)=%r)" % \
            (self.start_tick, self.time_sig_top, self.time_sig_bottom, self.key_sig_top, self.key_sig_bottom, self.instr_key,
             self.instr_name, self.channel, self.tempo, self.dynamic, self.song, len(self.notes))
