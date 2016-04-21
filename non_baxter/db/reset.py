from sqlalchemy_utils import database_exists, create_database, drop_database

from . import engine,Base

import sys

def create():
    if not database_exists(engine.url):
        create_database(engine.url)
        Base.metadata.create_all(engine)

def drop():
    if database_exists(engine.url):
        drop_database(engine.url)
        Base.metadata.drop_all(engine)

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'create':
        print "Creating database!"
        create()
    elif len(sys.argv) > 1 and sys.argv[1] == 'drop':
        print "Dropping database!"
        drop()
    pass
