#!/bin/bash -v

# create the database, parse / store the midi files
python -m db.reset create
python -m db.midi_store data/

# analyze music for chord progressions
python -m analyze.chords -t 16 -d 4

# train the ngram model
python -m ngram.train -o ngram/model/ -k C
