"""
The iter package contains a series of 3 iterators for traversing a piece of music:

- `note_iterator.py` iterates through notes in non-decreasing time order,
- `chord_iterator.py` iterates through a song by chord, and
- `time_iterator.py` iterates through a song by time instance (what is playing at a particular moment).
"""

from song_iterator import SongIterator
from chord_iterator import ChordIterator
from time_iterator import TimeIterator
