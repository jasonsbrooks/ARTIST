#!/usr/bin/python

'''
Stores midi files in a directory into DB

python -m db.midi_store [-d data_directory] [-t pool_size] [-u username] [-p password]

where:
    - `data_directory` is the location of the MIDI files.
    - `pool_size` is the number of databases
    - `username` is the database username
    - `password` is the database password
'''

import os,fnmatch
from helpers import Runner
from optparse import OptionParser
from multiprocessing import Queue
from . import get_engines
from utils import Counter

def main():
    """
    Called when midi_store is run directly. Starts a series of database worker processes and kicks off the parsing of
     the MIDI files and storing them in the database(s).
    """
    parser = OptionParser()

    parser.add_option("-d", "--data-directory", dest="data_directory", default="data/")
    parser.add_option("-t", "--pool-size", dest="pool_size", default=8, type="int")
    parser.add_option("-u", "--username", dest="db_username", default="postgres")
    parser.add_option("-p", "--password", dest="db_password", default="postgres")

    (options, args) = parser.parse_args()

    # construct the mp.Queue of midi files to process
    q = Queue()
    for root, dirnames, filenames in os.walk(options.data_directory):
        for filename in fnmatch.filter(filenames, '*.mid'):
            midiPath = os.path.abspath(os.path.join(root, filename))
            q.put(midiPath)

    # construct the series of database engines
    engines = get_engines(options.pool_size,options.db_username,options.db_password)
    processes = []
    counter = Counter(0)
    for i in xrange(options.pool_size):
        p = Runner(q,engines[i],counter)
        p.start()
        processes.append(p)

    # wait for all engines to complete.
    for p in processes:
        p.join()

if __name__ == "__main__":
    main()
    pass
