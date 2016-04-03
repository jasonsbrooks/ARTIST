from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

_engine = create_engine('sqlite:////tmp/artist.db')
Session = sessionmaker(bind=_engine)

from sqlalchemy.ext.declarative import declarative_base
Base = declarative_base()