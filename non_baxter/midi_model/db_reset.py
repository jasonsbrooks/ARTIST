from note import *
from song import *
from track import *
from sqlalchemy import create_engine

from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

engine = create_engine('sqlite:////tmp/artist.db')
session = sessionmaker()
session.configure(bind=engine)
Base.metadata.create_all(engine)
