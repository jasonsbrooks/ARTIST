from db import get_sessions,Song,Track,Note
from optparse import OptionParser

def stat(options):
    song_count,trk_count,note_count = 0,0,0
    for session in get_sessions(options.pool_size,options.db_username,options.db_password):
        print session
        song_count += session.query(Song).count()
        trk_count += session.query(Track).count()
        note_count += session.query(Note).count()

    print "song count:", song_count
    print "track count:", trk_count
    print "note count: ", note_count

if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("-t", "--pool-size", dest="pool_size", default=8, type="int")
    parser.add_option("-u", "--username", dest="db_username", default="postgres")
    parser.add_option("-p", "--password", dest="db_password", default="postgres")
    (options, args) = parser.parse_args()

    stat(options)