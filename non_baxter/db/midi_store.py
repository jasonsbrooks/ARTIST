#!/usr/bin/python

'''
Stores midi files in a directory into DB

>> ./midi_store.py [optional-folder]

if no folder specified, midi_store.py will store all midi files in current directory
'''

import os,fnmatch
from helpers import Runner
from Queue import Queue
from optparse import OptionParser

def main():
    parser = OptionParser()

    parser.add_option("-d", "--data-directory", dest="data_directory", default="data/")
    parser.add_option("-t", "--pool-size", dest="thread_pool_size", default=8, type="int")

    (options, args) = parser.parse_args()

    q = Queue()
    for root, dirnames, filenames in os.walk(options.data_directory):
        for filename in fnmatch.filter(filenames, '*.mid'):
            midiPath = os.path.abspath(os.path.join(root, filename))
            q.put(midiPath)

    for i in xrange(options.thread_pool_size):
        thrd = Runner(q)
        thrd.daemon = True
        thrd.start()

    q.join()

if __name__ == "__main__":
    main()
    pass
