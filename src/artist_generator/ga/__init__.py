"""
The ga package contains code for the genetic algorithms, including:

- the main module `ga.py` for running the algorithm,
- `chords.py` for creating and analyzing chords / chord progressions,
- `fitness.py` for calculating the fitness of an individual chromosome,
- `ga_midi.py` for generating MIDI files from the result, and
- `mutation.py` for mutating chromosomes throughout the evolution process.
"""

from ga import run
from ga_midi import create_midi_file
from chords import create_chord_progression