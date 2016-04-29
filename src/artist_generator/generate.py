#!/usr/bin/env python
"""
Generate a MIDI piece using trigram then GA.

    $ python generate.py -n ngram_dir -g ga_iter

where:
    - `ngram_dir` is the directory containing the ngram model files
    - `ga_iter` specifies the number of GA iterations to run
"""

import ngram,ga
from optparse import OptionParser

# defulat to generate a twelve bar blues
TWELVE_BAR_BLUES = [1,1,1,1,4,4,1,1,5,4,1,1]

def main():
    parser = OptionParser()

    parser.add_option("-n", "--ngram", dest="ngram",default="undefined")
    parser.add_option("-g", "--ga", dest="ga", type="int", default=800)

    (options, args) = parser.parse_args()

    ga.run(ngram_generate=(lambda: ngram.generate(options.ngram,TWELVE_BAR_BLUES)),num_iter=options.ga)

if __name__ == '__main__':
    main()
