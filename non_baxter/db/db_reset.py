from sqlalchemy import create_engine

from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base

from pprint import pformat

from sqlalchemy import Column, DateTime, String, Integer, ForeignKey, func
from sqlalchemy.orm import relationship, backref

import sys

Base = declarative_base()

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
        return "Song(title=%r, ppqn=%r, tracks=%r)" % \
            (self.title, self.ppqn, self.tracks)


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


def create():
	engine = create_engine('sqlite:////tmp/artist.db')
	session = sessionmaker()
	session.configure(bind=engine)
	Base.metadata.drop_all(engine)
	Base.metadata.create_all(engine)

if __name__ == '__main__':
	if len(sys.argv) > 1 and sys.argv[1] == 'create':
		print "Creating database!"
		create()
	pass
