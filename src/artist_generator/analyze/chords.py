from db import get_engines,get_sessions,Song,Track,Note
from iter import TimeIterator
from utils import Counter
from sqlalchemy.orm import sessionmaker

from preference_rules import *

import music21,sys
from optparse import OptionParser
from multiprocessing import Process,Queue

class ChordSpan(object):
    """
    A ChordSpan is a series of TimeInstances that all have the same root.
    Each ChordSpan also maintains a pointer (prev_cs) to the previous ChordSpan computed in the song.
    """
    def __init__(self,initial_ts,prev_cs):
        """
        Initialize a ChordSpan

        Args:
            initial_ts: the first TimeInstance to consider
            prev_cs: the previous TimeInstance
        """
        self.tss = [initial_ts]
        self.root = None

        # a back-pointer to the previous best chord-span
        self.prev_cs = prev_cs

    def __repr__(self):
        return "<ChordSpan: root=%r>" % (self.root)

    def last_ts(self):
        """
        Calculate and return the last TimeInstance in this ChordSpan.

        Returns:
            TimeInstance: the last time instance in the ChordSpan
        """
        return max(self.tss,key=lambda ts: ts.time)

    def add(self,ts):
        """
        Add a TimeInstance to this ChordSpan

        Args:
            ts: the TimeInstance to add
        """
        self.tss.append(ts)

    def remove(self,ts):
        """
        Remove a TimeInstance from this ChordSpan

        Args:
            ts: the TimeInstance to remove
        """
        self.tss.remove(ts)

    def notes(self):
        """
        Flatten all notes in the TimeInstances that comprise this ChordSpan.

        Returns:
             All notes played in this ChordSpan
        """
        res = []
        # iterate through all chords
        for ts in self.tss:
            # all notes in this time instance
            for note in ts.notes():
                res.append(note)
        return res

    def roman_numeral(self,track):
        """
        Calculate the roman numeral corresponding to the computed root and key of the corresponding track

        Args:
            track: The track to which a Note in this ChordSpan belongs. Note: Here we assume that at any moment in
            time, there is only one key signature in all tracks of the song.

        Returns:
            the Music21 Roman Numeral object.
        """

        pitch = music21.key.sharpsToPitch(track.key_sig_top)
        key = music21.key.Key(pitch)

        if track.key_sig_bottom == 0:
            scale = music21.scale.MajorScale(self.root.name)
        else:
            scale = music21.scale.MelodicMinorScale(self.root.name)

        chord = music21.chord.Chord([scale.chord.root(),scale.chord.third,scale.chord.fifth])

        return music21.roman.romanNumeralFromChord(chord,key).scaleDegree

    def label(self,depth=0):
        """
        Label all the notes in this ChordSpan with the determined root.

        Then proceed to recursively label the preceding ChordSpan
        """
        rn = None
        # label all the notes in this chord span
        for note in self.notes():
            if self.root:
                note.root = self.root.midi
                note.iso_root = self.root.name
                if not rn:
                    rn = self.roman_numeral(note.track)
                note.roman = rn

        # label the previous chord span (assuming we haven't surpassed max recursion limit)
        if self.prev_cs and depth < sys.getrecursionlimit() - 1:
            self.prev_cs.label()

    def pr_score(self,m_root):
        """
        Calculate the preference rule score, when using m_root as a root for this ChordSpan.
        Note this method is the core of the Preference Rule approach to Harmonic Analysis

        Such an approach is heavily inspired by the work of Daniel Sleater and David Temperley at CMU
        in their Melisma Music Analyzer: http://www.link.cs.cmu.edu/melisma/

        Args:
            m_root (Music21.note.Note): a note representing the proposed root of this chord

        Returns:
             the score obtained using this note as a root
        """
        last_ts = self.last_ts()
        ts_notes = last_ts.notes()

        # calculate the beat strength
        stren = beat_strength(ts_notes)

        # compatibility scores
        comp_score = compatibility(ts_notes,m_root)

        # difference from previous chord root on line of fifths
        lof = (lof_difference(self.prev_cs.root,m_root) if self.prev_cs else 0)

        return STRENGTH_MULTIPLIER * stren + COMPATIBILITY_MULTIPLIER * comp_score + LOF_MULTIPLIER * lof

    def calc_best_root(self):
        """
        Calculate the best root for this chord span

        Returns:
             the combined score of this ChordSpan and its predecessor
        """

        # start with C, weight of 0
        best_root,best_weight = music21.note.Note('C'),-len(line_of_fifths)

        # try all possible roots
        for m_root in music21.scale.ChromaticScale('C').pitches:

            val = self.pr_score(m_root)

            if val > best_weight:
                best_root,best_weight = m_root,val

        # use this as the chord-span root
        self.root = best_root

        # calculate the combined score
        prev_cs_score = (self.prev_cs.score if self.prev_cs else 0)
        return prev_cs_score + best_weight

# increased recursion limit for dynamic programming back-pointer labelling process
RECURSION_LIMIT = 10000

class HarmonicAnalyzer(Process):
    """
    Run Harmonic Analysis in a separate process.

    Such an approach is heavily inspired by the work of Daniel Sleater and David Temperley at CMU
        in their Melisma Music Analyzer: http://www.link.cs.cmu.edu/melisma/
    """
    def __init__(self,durk_step,engine,counter):
        """
        Initialize the Harmonic Analyzer process

        Args:
            durk_step: steps between TimeInstances
            engine: the database engine to draw songs from
            counter (Counter): atomic song counter
        """
        # Initialize the Process
        Process.__init__(self)

        # time step used in TimeIterator
        self.durk_step = durk_step

        # session to pull songs from
        Session = sessionmaker(bind=engine)
        self.session = Session()

        # Counter object representing number of songs that have been processed
        self.counter = counter
	
	# increase the recursion limit
	sys.setrecursionlimit(RECURSION_LIMIT)

    def run(self):
        """
        Start the Process. Note that this method overrides Process.run()
        """

        # Iterate through every song in the database attached to this process
        for song in self.session.query(Song).all():

            # Atomically increment the song counter
            count = self.counter.incrementAndGet()
            print count, ". ", song

            # and run the analysis
            self.analyze(song)


    def analyze(self,song):
        """
        Run Harmonic Analysis on a particular Song

        Args:
            song (Song): the song to analyze
        """
        cs,idx = None,0

	try:
		# construct the iterator
		ti = TimeIterator(song,self.durk_step)
	except ValueError,e:
		# something is very wrong with this song... let's skip it!
		sys.stderr.write("Skipping " + str(song) + ":")
		sys.stderr.write("\t" + str(e))
		return False

        # iterate through every TimeInstance in the song
        for ts in ti:

            # and consider what to do...
            cs = self.consider_ts(cs,ts)
            # print idx, ts, "--", cs.score, ":", cs
            idx += 1

        cs.label()
        self.session.commit()


    def consider_ts(self,cs,ts):
        """
        Consider how to segment a new TimeInstance.

        Note: this method is the core of the Dynamic Programming approach to Harmonic Analysis.

        Args:
            cs: the latest determined ChordSpan in the Song being evaluated
            ts: the TimeInstance under consideration

        Returns:
            the new latest ChordSpan
        """

        # if this is the first ChordSpan created.
        if not cs:
            res = ChordSpan(ts,None)
            score = res.calc_best_root()
        else:
            # we already have a ChordSpan in progress.

            # option 1: start a new chord-span
            opt1_cs = ChordSpan(ts,cs)
            opt1_score = cs.calc_best_root()

            # option 2: add to prior segment
            cs.add(ts)
            opt2_score = cs.calc_best_root()

            # determine which option is superior
            if opt1_score > opt2_score:
                cs.remove(ts)
                res = opt1_cs
                score = opt1_score
            else:
                res = cs
                score = opt2_score

        # set the score on this cs
        res.score = score

        return res

def main():
    """
    Run harmonic analysis on all songs in all databases. This will take a LONG time.
    """
    parser = OptionParser()

    parser.add_option("-d", "--durk-step", dest="durk_step", default=4, type="int")
    parser.add_option("-t", "--pool-size", dest="pool_size", default=8, type="int")
    parser.add_option("-u", "--username", dest="db_username", default="postgres")
    parser.add_option("-p", "--password", dest="db_password", default="postgres")
    (options, args) = parser.parse_args()


    print "Creating", options.pool_size, "processes."
    processes = []

    # Initialize the counter to 0
    counter = Counter(0)

    # get all database engines
    engines = get_engines(options.pool_size,options.db_username,options.db_password)

    # Construct a new HarmonicAnalyzer process for each database.
    for i in xrange(options.pool_size):
        p = HarmonicAnalyzer(options.durk_step,engines[i],counter)
        processes.append(p)

    # Start the processes
    print "Starting", options.pool_size, "processes."
    for p in processes:
        p.start()

    # And wait for them to finish
    for p in processes:
        p.join()

if __name__ == '__main__':
    main()
