from sqlalchemy_utils import database_exists, create_database, drop_database

from . import get_engines,Base
from optparse import OptionParser
import sys

def create(eng):
    if not database_exists(eng.url):
        create_database(eng.url)
        Base.metadata.create_all(eng)

def drop(eng):
    if database_exists(eng.url):
        drop_database(eng.url)

def create_many(num,usr="postgres",pwd="postgres"):
    for engine in get_engines(num,usr,pwd):
        create(engine)

def drop_many(num,usr="postgres",pwd="postgres"):
    for engine in get_engines(num,usr,pwd):
        drop(engine)

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("-u", "--username", dest="db_username", default="postgres")
    parser.add_option("-p", "--password", dest="db_password", default="postgres")
    (options, args) = parser.parse_args()

    if len(sys.argv) > 2 and sys.argv[1] == 'create':
        print "Creating " + sys.argv[2] + " databases!"
        create_many(int(sys.argv[2]),options.db_username,options.db_password)

    elif len(sys.argv) > 2 and sys.argv[1] == 'drop':
        print "Dropping " + sys.argv[2] + " databases!"
        drop_many(int(sys.argv[2]),options.db_username,options.db_password)
