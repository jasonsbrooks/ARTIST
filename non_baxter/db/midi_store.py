#!/usr/bin/python

'''
Stores midi files in a directory into DB

>> ./midi_store.py [optional-folder]

if no folder specified, midi_store.py will store all midi files in current directory
'''

import os,fnmatch
from helpers import Runner
from optparse import OptionParser
from multiprocessing import Queue
import reset

def main():
    parser = OptionParser()

    parser.add_option("-d", "--data-directory", dest="data_directory", default="data/")
    parser.add_option("-t", "--pool-size", dest="pool_size", default=8, type="int")
    parser.add_option("-u", "--username", dest="db_username", default="postgres")
    parser.add_option("-p", "--password", dest="db_password", default="postgres")

    (options, args) = parser.parse_args()

    q = Queue()
    for root, dirnames, filenames in os.walk(options.data_directory):
        for filename in fnmatch.filter(filenames, '*.mid'):
            midiPath = os.path.abspath(os.path.join(root, filename))
            q.put(midiPath)

    engines = reset.get_engines(options.pool_size,options.db_username,options.db_password)
    processes = []
    for i in xrange(options.pool_size):
        p = Runner(q,engines[i])
        p.start()
        processes.append(p)

    for p in processes:
        p.join()

if __name__ == "__main__":
    main()
    pass
