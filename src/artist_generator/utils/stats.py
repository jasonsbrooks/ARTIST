#!/usr/bin/env python
"""
A quick Python script to calculate the total number of songs, tracks, and notes stored in our database(s)

USAGE:
    python -m utils.stats -t DBPOOL_SIZE -u USERNAME -p PASSWORD

where:
    DBPOOL_SIZE is the number of databases
    USERNAME is the database username
    PASSWORD is the database password
"""

from db import get_sessions,Song,Track,Note
from optparse import OptionParser

def stat(options):
    """
    Calculate the statistic on our database(s).

    Args:
        options (dict): specifying pool_size, db_username, db_password
    """
    song_count,trk_count,note_count = 0,0,0
    for session in get_sessions(options.pool_size,options.db_username,options.db_password):
        song_count += session.query(Song).count()
        trk_count += session.query(Track).count()
        note_count += session.query(Note).count()

    print "song count:", song_count
    print "track count:", trk_count
    print "note count: ", note_count

if __name__ == '__main__':
    parser = OptionParser()

    # parse the options
    parser.add_option("-t", "--pool-size", dest="pool_size", default=8, type="int")
    parser.add_option("-u", "--username", dest="db_username", default="postgres")
    parser.add_option("-p", "--password", dest="db_password", default="postgres")
    (options, args) = parser.parse_args()

    # and calculate the statistics
    stat(options)