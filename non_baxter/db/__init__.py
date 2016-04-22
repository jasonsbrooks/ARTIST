from sqlalchemy import create_engine

def get_engines(num,usr="postgres",pwd="postgres"):
    engines = []
    for i in xrange(num):
        engines.append(create_engine("postgresql://" + usr + ":" + pwd
                               + "@localhost:5432/artist_" + str(i), pool_size=20, max_overflow=100))
    return engines

from sqlalchemy.ext.declarative import declarative_base
Base = declarative_base()

from song import Song
from track import Track
from note import Note