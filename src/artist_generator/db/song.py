from sqlalchemy import Column, DateTime, String, Integer, ForeignKey, func
from sqlalchemy.orm import relationship
from . import Base

class Song(Base):
    """
    A Song with the following properties:

    Properties
        title: String representing title of Song
        ppqn: Integer representing "pulses per quarter note" (i.e. 96 ticks/ quarter note)
        tracks: list of Track objects that constitute the song
    """
    __tablename__ = 'song'

    id = Column(Integer, primary_key=True)
    title = Column(String, nullable=False)
    ppqn = Column(Integer, nullable=False)
    tracks = relationship("Track")

    def __repr__(self):
        return "Song(title=%r, ppqn=%r, len(tracks)=%r)" % \
            (self.title, self.ppqn, len(self.tracks))