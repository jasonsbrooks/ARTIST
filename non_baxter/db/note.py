from sqlalchemy import Column, Integer, ForeignKey
from sqlalchemy.orm import relationship
from . import Base

class Note(Base):
    """
    A Note represents a line of note in a track with the following properties:

    Properties
        pitch: Integer from 0-127 representing note pitch
            Note: Defaults at 60 (C3)
            Note: -1 represents a "rest note"
        dur: Integer where 1 === 32nd note and 32 === whole note.  Call this unit "durks"
        start: Integer representing start time of note relative to Song in "durk" units (see dur)
        tick_dur: Integer representing duration of note in ticks
            Note: will be -1 for rest (no meaning)
        start_tick: Integer representing start time of note relative to Song (i.e. 0 is beginning of song) in tick units
            Note: NOT relative to Track object, relative to Song object!
            Note: will be -1 for rest (no meaning)
        measure: Integer representing measure of Song note is contained within
            Note: 0 is the first measure of a Song
    """

    __tablename__ = 'note'

    id = Column(Integer, primary_key=True)
    pitch = Column(Integer, nullable=False)
    dur = Column(Integer, nullable=False) 
    start = Column(Integer, nullable=False)
    end = Column(Integer, nullable=False)
    tick_dur = Column(Integer, nullable=False) 
    start_tick = Column(Integer, nullable=False)
    measure = Column(Integer, nullable=False)
    track_id = Column(Integer, ForeignKey('track.id'))
    track = relationship("Track", back_populates="notes")

    def __repr__(self):
        return "note(pitch=%r, dur=%r, start=%r, tick_dur=%r, start_tick=%r, measure=%r)" % \
            (self.pitch, self.dur, self.start, self.tick_dur, self.start_tick, self.measure)
