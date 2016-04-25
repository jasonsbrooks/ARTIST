#!/usr/bin/env python
"""
A quick utility script to mark analyzed songs as analyzed.
A song has been analyzed if any notes contain a non-NULL root.
"""

from db import get_sessions,Song
from optparse import OptionParser
from iter import SongIterator


def main():
    parser = OptionParser()

    parser.add_option("-t", "--pool-size", dest="pool_size", default=8, type="int")
    parser.add_option("-u", "--username", dest="db_username", default="postgres")
    parser.add_option("-p", "--password", dest="db_password", default="postgres")
    (options, args) = parser.parse_args()

    count = 0

    # iterate through all database sessions
    for session in get_sessions(options.pool_size,options.db_username,options.db_password):
        # through all songs and notes
        for song in session.query(Song).all():
            print count, ".", song
            # the song has already been marked as analyzed
            if song.analyzed:
                print "\t(song.analyzed == True)"
                continue

            # check notes in the song
            for note in SongIterator(song):
                # if a note has a root, then the song has been analyzed
                if note.root != None:
                    song.analyzed = True
                    break

            # print out the results
            if song.analyzed:
                print "\tAlready analyzed."
            else:
                print "\tNeed to analyze."

            session.commit()
            count += 1

if __name__ == '__main__':
    main()

