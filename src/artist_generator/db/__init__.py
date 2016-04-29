"""
The db package contains model definitions for

- `song.py`
- `track.py`: a line of music in a Song with a single time signature, key signature, instrument key
- `note.py`

and

- `helpers.py` for parsing the MIDI files (this is the exciting part !),
- `midi_store.py` for running the parser,
- `reset.py` for creating / dropping databases.
"""
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

def get_engines(num,usr="postgres",pwd="postgres"):
    """
    Get engines for num different databases
    Args:
        num: number of databases
        usr: database username
        pwd: database password

    Returns:
        engine[]: the database engines
    """
    engines = []
    for i in xrange(num):
        engines.append(create_engine("postgresql://" + usr + ":" + pwd
                               + "@localhost:5432/artist_" + str(i), pool_size=20, max_overflow=100))
    return engines

def get_sessions(num,usr="postgres",pwd="postgres"):
    """
    Get sessions for num different databases
    Args:
        num: number of databases
        usr: database username
        pwd: database password

    Returns:
        Session[]: the database sessions
    """
    engines = get_engines(num,usr,pwd)
    sessions = []
    for engine in engines:
        Session = sessionmaker(bind=engine)
        sessions.append(Session())
    return sessions

from sqlalchemy.ext.declarative import declarative_base
Base = declarative_base()

from song import Song
from track import Track
from note import Note