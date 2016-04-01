from pprint import pformat

from sqlalchemy import Column, DateTime, String, Integer, ForeignKey, func
from sqlalchemy.orm import relationship, backref

from db_reset import Base

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

    # def __init__(self, title="", ppqn=120, tracks=[]):
    #     self.title = title
    #     self.ppqn = ppqn
    #     super(Song, self).__init__(tracks)

    def __repr__(self):
        return "Song(title=%r, ppqn=%r, tracks=\\\n%s)" % \
            (self.title, self.ppqn, pformat(list(self)))
