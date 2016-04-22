from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

def get_engines(num,usr="postgres",pwd="postgres"):
    engines = []
    for i in xrange(num):
        engines.append(create_engine("postgresql://" + usr + ":" + pwd
                               + "@localhost:5432/artist_" + str(i), pool_size=20, max_overflow=100))
    return engines

def get_sessions(num,usr="postgres",pwd="postgres"):
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
from exceptions import InvalidKeySignature