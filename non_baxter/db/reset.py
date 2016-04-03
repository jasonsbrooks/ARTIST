from sqlalchemy import create_engine

from pprint import pformat

from sqlalchemy import Column, DateTime, String, Integer, ForeignKey, func
from sqlalchemy.orm import relationship, backref

from . import engine,Base

import sys

def create():
	Base.metadata.drop_all(engine)
	Base.metadata.create_all(engine)

if __name__ == '__main__':
	if len(sys.argv) > 1 and sys.argv[1] == 'create':
		print "Creating database!"
		create()
	pass
