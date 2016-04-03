from sqlalchemy import create_engine

from pprint import pformat

from sqlalchemy import Column, DateTime, String, Integer, ForeignKey, func
from sqlalchemy.orm import relationship, backref

import sys


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
