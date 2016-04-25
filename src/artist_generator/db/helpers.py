'''
Complete:
1) Get all TimeSignatureEvent and SetTempoEvent
2) tag tracks with this metadata (including time signature and tempo)
2) investigate why occasional on/off note event mismatch errors
    Answer: just use queue
3) duration in get_notes-- subtracting ticks doesn't seem to give you the
    exact right float value...seems to be a bit short everytime
3b) Figure out how to insert rests into note list from information given
    but do this after you think more about what the duration actually means

Todo:
4) figure out a way to NOT include drums (key is 0?  same as piano?)
    idea: drums are always on channel 9 (i.e. 0-indexed 10 from https://www.cs.cmu.edu/~music/cmsip/readings/MIDI%20tutorial%20for%20programmers.html)
5) Add naive tempo/dynamics in track

Important MIDI Notes:
1) NoteOnEvent with velocity of 0 === NoteOffEvent
2) pattern.resolution contains resolution (ppqn)
'''

import midi, sys, pdb, os, re,threading

from collections import defaultdict
from audiolazy import midi2str
from sqlalchemy.orm import sessionmaker
from multiprocessing import Process

from . import Song,Track,Note

DURKS_PER_QUARTER_NOTE = 8

class Runner(Process):
    """
    A database worker process. Parses MIDI files pulled from self.q and stores them in the database specified by self.engine.
    """

    def __init__(self,q,engine,counter):
        """
        Initialize a Runner
        Args:
            q: the queue of MIDI file names
            engine: the database engine into which we store the Notes
            counter: count the number of songs that have been parsed.
        """
        Process.__init__(self)
        self.q = q
        Session = sessionmaker(bind=engine)
        self.session = Session()
        self.counter = counter

    def run(self):
        """
        Start the process
        """
        while True:
            midiPath = self.q.get()

            count = self.counter.incrementAndGet()
            print str(count) + ". Analyzing " + midiPath.split('/')[-1]
            self.midi_to_song(midiPath)
            print "Finished " + midiPath.split('/')[-1]

    def midi_to_song(self,midifilename):
        """
        Takes midi file and returns a representative song object

        - a Song is a list of many Tracks
        - a Track is a list of many Notes representing a common 1) key signature, 2) time signature, and 3) instrument key

        Args:
            midifilename: filename of the MIDI file to parse and store in the database
        """

        try:
            pattern = midi.read_midifile(midifilename)
        except Exception as e:
            print 'Exception: ' + str(e)
            return

        pattern.make_ticks_abs()  # makes ticks absolute instead of relative
        resolution = pattern.resolution  # note pattern.resolution contains resolution (ppqn)

        title = re.sub(r'[^\x00-\x7F]+', ' ', os.path.basename(midifilename))

        # create Song object
        song = Song(title=title, ppqn=resolution)
        self.session.add(song)

        # create ordered list of all time and key signature events
        time_sig_events = []
        key_sig_events = []
        all_sig_events = []

        for track in pattern:
            for event in track:
                if type(event) is midi.TimeSignatureEvent:
                    time_sig = {'start_tick': event.tick, 'n': event.data[0], 'd': 2**event.data[1], 'type': 'time'}  # see track.py, note that in MIDI denom is a negative power of 2
                    time_sig_events.append(time_sig)
                elif type(event) is midi.KeySignatureEvent:
                    key_sig = {'start_tick': event.tick, 'sf': event.data[0], 'mi': event.data[1], 'type': 'key'}  # see track.py
                    key_sig_events.append(key_sig)

        time_sig_events.sort(key=lambda x: x['start_tick'])
        key_sig_events.sort(key=lambda x: x['start_tick'])

        # (!!!) it's possible that no time/key sig given in MIDI file
        # in these cases, we create a list with a single default key/time signature
        # according to midi standards:
        # default key: (0, 0)
        # default time: (4, 4)
        if len(key_sig_events) == 0:
            key_sig_events.append({'start_tick': 0, 'sf': 0, 'mi': 0, 'type': 'key'})
        if len(time_sig_events) == 0:
            time_sig_events.append({'start_tick': 0, 'n': 4, 'd': 4, 'type': 'time'})

        # iterate thru tracks
        # create appropriate song, track, and note structure
        for midi_track in pattern:
            instr_track_data = self.is_instr_track(midi_track)
            if instr_track_data:
                instr_key = instr_track_data[0]
                instr_name = instr_track_data[1]
                channel = instr_track_data[2]

                # Create temporary copy of time_sig_events and key_sig_events
                temp_time_sig_events = time_sig_events[:]
                temp_key_sig_events = key_sig_events[:]

                # Note: this assumes that a key and time signature are intialized at tick 0!
                ts = temp_time_sig_events.pop(0)  # holds current time signature
                ks = temp_key_sig_events.pop(0)  # holds current key signature

                all_sig_events = temp_time_sig_events + temp_key_sig_events
                all_sig_events.sort(key=lambda x: x['start_tick'])

                instr_name = re.sub(r'[^\x00-\x7F]+', ' ', instr_name)  # converts non-ASCII chars to spaces

                track = Track(time_sig_top=ts['n'], time_sig_bottom=ts['d'], key_sig_top=ks['sf'],
                              key_sig_bottom=ks['mi'], instr_key=instr_key, instr_name=instr_name,
                              channel=channel, start_tick=0, song=song)
                self.session.add(track)

                # init these next temp vars in case no sig events left
                next_sig_event = None
                next_track_tick = sys.maxint

                if len(all_sig_events) > 0:
                    next_sig_event = all_sig_events[0]
                    next_track_tick = next_sig_event['start_tick']  # holds max tick of current track

                notes = self.get_notes(self.get_note_events(midi_track), resolution, track)

                for n in notes:
                    if not n.start_tick < next_track_tick:  # still on current track
                        # update key or time signature
                        if next_sig_event['type'] == 'key':
                            ks = next_sig_event
                        elif next_sig_event['type'] == 'time':
                            ts = next_sig_event

                        self.insert_rests_into_track(track, resolution)  # insert rests into track

                        instr_name = re.sub(r'[^\x00-\x7F]+', ' ', instr_name)  # converts non-ASCII chars to spaces

                        # create new track and update temporary variables
                        track = Track(time_sig_top=ts['n'], time_sig_bottom=ts['d'], key_sig_top=ks['sf'],
                                      key_sig_bottom=ks['mi'], instr_key=instr_key, instr_name=instr_name,
                                      channel=channel, start_tick=next_sig_event['start_tick'], song=song)
                        self.session.add(track)

                        all_sig_events.pop(0)

                        if len(all_sig_events) > 0:
                            next_sig_event = all_sig_events[0]
                            next_track_tick = next_sig_event['start_tick']
                        else:  # if no events left, set next_track_tick to maxint
                            next_sig_event = None
                            next_track_tick = sys.maxint

                    n.track = track
                    self.session.add(n)

                self.insert_rests_into_track(track, resolution)  # insert rests into track
        self.session.commit()
        return song

    def insert_rests_into_track(self,track, ppqn):
        """
        Take a Track object, inserts rest inot the appropriate locations (where no pitch is being played)

        Warnings:
            Directly modifies the Track object that is passed to it!

        Args:
            track: the Track into which we insert rests
            ppqn:
        """
        # find total number of durks in track
        firstNote = self.session.query(Note).filter(Note.track == track).order_by(Note.start.asc()).first()
        lastNote = self.session.query(Note).filter(Note.track == track).order_by(Note.end.desc()).first()

        # if no notes in track, just return
        if firstNote is None or lastNote is None:
            return

        max_durk = lastNote.end
        min_durk = firstNote.start

        # initialize rest list (each entry represents one durk)
        rest_list = [True for i in range(max_durk+1)]

        # change all durks in rest_list to 0 where note is being played
        for idx, note in enumerate(track.notes):
            # try:
            # if idx == len(track.notes) - 1:
            #     pdb.set_trace()
            for durk in range(note.dur):
                try:
                    rest_list[note.start + durk] = False  # set rest_list to False for each durk where a note pitch is playing
                except:
                    pdb.set_trace()
        # iterate through completed rest_list and insert appropriate rests into track
        current_durk = min_durk
        while current_durk < max_durk:
            if rest_list[current_durk]:  # rest found!
                rest_start = current_durk  # store start of rest note

                while current_durk < max_durk and rest_list[current_durk]:  # increment current_durk until pitch found
                    current_durk += 1

                rest_dur = current_durk - rest_start
                rest_measure = rest_start / (DURKS_PER_QUARTER_NOTE * track.time_sig_bottom)  # note 8 is number of durks per quarter note

                note = Note(pitch=-1, dur=rest_dur, start=rest_start, end=rest_dur+rest_start,
                            tick_dur=-1, start_tick=-1, measure=rest_measure, track=track)
                self.session.add(note)
            else:
                current_durk += 1

        return


    def is_instr_track(self,track):
        """
        Checks if midi track object is an instrument track

        Args:
            track (Track): the track to check

        Returns:
            dict: dic (instr_key, instr_name (may be empty), channel) if true. None if false

        """
        should_return = False
        instr_key = -1
        instr_name = ""
        channel = -1
        for event in track:
            if type(event) is midi.ProgramChangeEvent:  # only act if actual instrument track!
                should_return = True
                instr_key = event.data[0]  # event.data[0] contains instrument key
                channel = event.channel
            if type(event) is midi.TrackNameEvent:
                instr_name = event.text

        if should_return:
            return (instr_key, instr_name, channel)
        else:
            return None


    def get_notes(self,note_events, ppqn, track):
        """
        Given a list of note events (from function get_note_events), and ppqn (pulses per quarter note)
           returns list of note objects with appropriate durations and other properties

        Notes:
            this method calculates appropriate durations based on ppqn of track

        Args:
            note_events: the note events
            ppqn: pulses per quarter notes
            track (Track): the Track to analyze.

        Returns:
            Note[]: list of note objects with appropriate durations and other properties
        """
        note_objs = []
        unclosed_notes = defaultdict(lambda: [])  # holds running hash of notes that haven't seen an off event yet
                                                  # key is note pitch (integer from 0-127), value is a QUEUE of note_event tuples
        for note_event in note_events:
            on_off = note_event[0]
            tick = note_event[1]
            pitch = note_event[2]
            if on_off == 1:  # NoteOnEvent
                unclosed_notes[pitch].append(note_event)
            elif on_off == 0:  # NoteOffEvent
                if len(unclosed_notes[pitch]) == 0:
                    measure = note_event[1] / (ppqn * track.time_sig_bottom)
                    print 'Warning: <get_notes> NoteOffEvent without corresponding NoteOnEvent %r, measure: %r, instr: %r, pitch: %r ' % (str(note_event), str(measure), track.instr_name, str(pitch))
                else:
                    note_on = unclosed_notes[pitch].pop(0)
                    start_tick = note_on[1]
                    tick_dur = tick - start_tick

                    start = .25 * start_tick / ppqn  # 1 === whole note, .25 === quarter note
                    start = int(round(start * 32))  # final conversion to durk units: 1 === a 32nd note....32  === a whole note
                    dur = .25 * tick_dur / ppqn  # 1 === whole note, .25 === quarter note
                    dur = int(round(dur * 32))  # final conversion to durk units: 1 === a 32nd note....32  === a whole note

                    measure = start_tick / (ppqn * track.time_sig_bottom)

                    note_obj = Note(pitch=pitch, dur=dur, start=start, end=dur+start,
                                    tick_dur=tick_dur, start_tick=start_tick, measure=measure, track=track,
                                    iso_pitch=midi2str(pitch))
                    note_objs.append(note_obj)

            else:  # Error checking
                print 'Warning: <get_notes> Note is neither On nor Off event'

        return note_objs


    def get_note_events(self,track):
        """
        Given a track returns NoteOnEvent and NoteOffEvent as list of tuples (on/off, tick, pitch, velocity)

        Args:
            track (Track): the Track to analyze

        Returns:
            tuple: NoteOnEvent and NoteOffEvent as list of tuples (on/off, tick, pitch, velocity)
        """
        notes = []
        for event in track:
            if type(event) is midi.NoteOnEvent:
                if event.data[1] != 0:
                    note = (1, event.tick, event.data[0], event.data[1])
                else:  # when velocity is 0, equivalent to NoteOffEvent
                    note = (0, event.tick, event.data[0], event.data[1])
                notes.append(note)
            elif type(event) is midi.NoteOffEvent:
                note = (0, event.tick, event.data[0], event.data[1])
                notes.append(note)

        return notes