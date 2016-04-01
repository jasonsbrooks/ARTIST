from sqlalchemy import Column, DateTime, String, Integer, ForeignKey, func
from sqlalchemy.orm import relationship, backref

from db_reset import Base

class Note(Base):
    """
    A Note represents a line of note in a track with the following properties:

    Properties
        pitch: Integer from 0-127 representing note pitch
            Defaults at 60 (C3)
            Should -1 represent a rest?
        dur: ??? Float where 1.0 ~ quarter note
        tick_dur: Integer representing duration of note in ticks
        start_tick: Integer representing start time of note relative to Song (i.e. 0 is beginning of song)
            Note: NOT relative to Track object, relative to Song object!
        measure: Integer representing measure of Song note is contained within
            Note: 0 is the first measure of a Song
    """

    __tablename__ = 'note'

    id = Column(Integer, primary_key=True)
    pitch = Column(Integer, nullable=False)
    dur = Column(Integer, nullable=False) 
    tick_dur = Column(Integer, nullable=False) 
    start_tick = Column(Integer, nullable=False)
    measure = Column(Integer, nullable=False)
    track_id = Column(Integer, ForeignKey('track.id'))

    # def __init__(self, pitch=60, dur=0, tick_dur=0, start_tick=0, measure=0):
    #     self.pitch = pitch
    #     self.dur = dur
    #     self.tick_dur = tick_dur
    #     self.start_tick = start_tick
    #     self.measure = measure

    def __repr__(self):
        return "note(pitch=%r, dur=%r, start_tick=%r, measure=%r)" % \
            (self.pitch, self.dur, self.start_tick, self.measure)
