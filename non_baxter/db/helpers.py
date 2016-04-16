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

import midi, sys, pdb, os, re

from collections import defaultdict

from sqlalchemy import desc, asc

from . import Base,Session,Song,Track,Note

DURKS_PER_QUARTER_NOTE = 8

session = Session()


# takes midi file and returns a representative song object
# a Song is a list of many Tracks
# a Track is a list of many Notes representing a common
#   1) key signature, 2) time signature, and 3) instrument key
def midi_to_song(midifilename):

    try:
        pattern = midi.read_midifile(midifilename)
    except Exception as e:
        print 'Exception: ' + str(e)
        return

    pattern.make_ticks_abs()  # makes ticks absolute instead of relative
    resolution = pattern.resolution  # note pattern.resolution contains resolution (ppqn)
    print 'Resolution (ppqn): ' + str(resolution)

    title = re.sub(r'[^\x00-\x7F]+', ' ', os.path.basename(midifilename))

    # get chord list
    p_file_name = os.path.splitext(midifilename)[0]+'.k'
    chord_list = chord_extraction(p_file_name)

    # create Song object
    song = Song(title=title, ppqn=resolution)
    session.add(song)

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
        instr_track_data = is_instr_track(midi_track)
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
            session.add(track)

            # init these next temp vars in case no sig events left
            next_sig_event = None
            next_track_tick = sys.maxint

            if len(all_sig_events) > 0:
                next_sig_event = all_sig_events[0]
                next_track_tick = next_sig_event['start_tick']  # holds max tick of current track

            notes = get_notes(get_note_events(midi_track), resolution, track, chord_list)

            for n in notes:
                if not n.start_tick < next_track_tick:  # still on current track
                    # update key or time signature
                    if next_sig_event['type'] == 'key':
                        ks = next_sig_event
                    elif next_sig_event['type'] == 'time':
                        ts = next_sig_event

                    insert_rests_into_track(track, resolution)  # insert rests into track

                    instr_name = re.sub(r'[^\x00-\x7F]+', ' ', instr_name)  # converts non-ASCII chars to spaces

                    # create new track and update temporary variables
                    track = Track(time_sig_top=ts['n'], time_sig_bottom=ts['d'], key_sig_top=ks['sf'],
                                  key_sig_bottom=ks['mi'], instr_key=instr_key, instr_name=instr_name,
                                  channel=channel, start_tick=next_sig_event['start_tick'], song=song)
                    session.add(track)

                    all_sig_events.pop(0)

                    if len(all_sig_events) > 0:
                        next_sig_event = all_sig_events[0]
                        next_track_tick = next_sig_event['start_tick']
                    else:  # if no events left, set next_track_tick to maxint
                        next_sig_event = None
                        next_track_tick = sys.maxint

                n.track = track
                session.add(n)

            insert_rests_into_track(track, resolution)  # insert rests into track
    session.commit()
    return song


# Takes a track object
# Inserts rests into appropriate locations (where no pitch is being played)
# Returns nothing
# Danger: directly modifies the track object that is passed to it!
def insert_rests_into_track(track, ppqn):
    # find total number of durks in track
    firstNote = session.query(Note).filter(Note.track == track).order_by(Note.start.asc()).first()
    lastNote = session.query(Note).filter(Note.track == track).order_by(Note.end.desc()).first()

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
                        tick_dur=-1, start_tick=-1, measure=rest_measure, track=track, root=None)
            session.add(note)
        else:
            current_durk += 1

    return


# checks if midi track object is an instrument track
# returns dic (instr_key, instr_name (may be empty), channel) if true
# else returns None
def is_instr_track(track):
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


# given a list of note events (from function get_note_events)
#   and ppqn (pulses per quarter note)
# returns list of note objects with appropriate durations and other properties
# this method calculates appropriate durations based on ppqn of track
def get_notes(note_events, ppqn, track, chord_list):
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
                print 'Warning: <get_notes> NoteOffEvent without corresponding NoteOnEvent %r, measure: %r, instr: %r, pitch: %r ' % (str(note_event), str(measure), track.instr_name, pitch_to_str(pitch))
            else:
                note_on = unclosed_notes[pitch].pop(0)
                start_tick = note_on[1]
                tick_dur = tick - start_tick

                start = .25 * start_tick / ppqn  # 1 === whole note, .25 === quarter note
                start = int(round(start * 32))  # final conversion to durk units: 1 === a 32nd note....32  === a whole note
                dur = .25 * tick_dur / ppqn  # 1 === whole note, .25 === quarter note
                dur = int(round(dur * 32))  # final conversion to durk units: 1 === a 32nd note....32  === a whole note

                measure = start_tick / (ppqn * track.time_sig_bottom)

                chord_root = -99
                for chord in chord_list:  # assumes chord_list in order by ascending start_ticks
                    if start_tick >= chord['start_tick']:
                        chord_root = chord['root']

                print 'This is the root: ' + str(chord_root)
                if chord_root is None:
                    print 'wtf'
                    return

                note_obj = Note(pitch=pitch, dur=dur, start=start, end=dur+start,
                                tick_dur=tick_dur, start_tick=start_tick, measure=measure, track=track, root=chord_root)
                print note_obj
                note_objs.append(note_obj)
        else:  # Error checking
            print 'Warning: <get_notes> Note is neither On nor Off event'

    return note_objs


# given a track returns NoteOnEvent and NoteOffEvent as list of tuples (on/off, tick, pitch, velocity)
def get_note_events(track):
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


# given a .p chord file from melisma
# returns list of chord dictionaries [{root:  , start_tick:  , end_tick:  } ... ]
# sorts in ascending order of start_tick
def chord_extraction(filename):
    chord_list = []

    with open(filename) as f:
        content = f.readlines()

    for line in content:
        line_tokens = line.split()
        if len(line_tokens) > 0 and line_tokens[0] == "Chord":
            chord_obj = {"root": int(line_tokens[3]), "start_tick": int(line_tokens[1]), "end_tick": int(line_tokens[2])}
            chord_list.append(chord_obj)

    chord_list.sort(key=lambda x: x['start_tick'])  # sort in ascending order by start_tick
    return chord_list


'''
START Deprecated Methods
'''


# takes midi file and constructs dictionary with following structure:
# {instr_key: [track1, track2, ...], ...} where track1, track2...are list of tuples returned by get_note_events
# currently assumes there exists only one ProgramChangeEvent per instrument track
def extract(midifilename):
    instr_tracks_dic = {}
    pattern = midi.read_midifile(midifilename)
    pattern.make_ticks_abs()  # makes ticks absolute instead of relative

    for track in pattern:
        print track[0:20]
        for event in track:
            if type(event) is midi.ProgramChangeEvent:  # only act if this track is an actual instrument track!
                instr_key = event.data[0]  # event.data[0] contains instrument key
                notes = get_note_events(track)

                if not(instr_key in instr_tracks_dic):
                    instr_tracks_dic[instr_key] = [notes]
                else:
                    instr_tracks_dic[instr_key].append(notes)

                break  # this assumes that there is only one ProgramChangeEvent per instrument track
    return instr_tracks_dic


# transposes pitch to be same pitch within range [lo,hi] inclusive
# if multiple pitch matches exist within range, match closest to given pitch
def transpose_pitch(pitch, lo, hi):
    octave = 12

    if (hi - lo) < (octave - 1):
        print "Transpose Error: hi - lo < 11, make sure range spans a full octave"
        sys.exit(1)

    while pitch < lo:
        pitch += octave
    while pitch > hi:
        pitch -= octave

    return pitch


# transposes list of notes in structure returned by function get_note_events
# within range [lo, hi] inclusive
def transpose_note_list(note_list, lo, hi):
    new_notes = []

    for note in note_list:
        new_pitch = transpose_pitch(note[2], lo, hi)
        new_note = (note[0], note[1], new_pitch, note[2])
        new_notes.append(new_note)

    return new_notes


# transposes all notes within dictionary returned by function extract
# within range [lo, hi] inclusive
def transpose_instr_tracks_dic(instr_tracks_dic, lo, hi):
    new_instr_tracks_dic = {}

    for instr_key, list_of_tracks in instr_tracks_dic.iteritems():
        for track in list_of_tracks:
            new_note_list = transpose_note_list(track, lo, hi)

            if not(instr_key in new_instr_tracks_dic):
                new_instr_tracks_dic[instr_key] = [new_note_list]
            else:
                new_instr_tracks_dic[instr_key].append(new_note_list)

    return new_instr_tracks_dic


# given a pitch value returns the appropriate note "letter"
def pitch_to_str(pitch):
    p_to_l_dic = {0: 'C',
                  1: 'C#',
                  2: 'D',
                  3: 'D#',
                  4: 'E',
                  5: 'F',
                  6: 'F#',
                  7: 'G',
                  8: 'G#',
                  9: 'A',
                  10: 'A#',
                  11: 'B'}

    return p_to_l_dic[pitch % 12]


# given a duration returns the appropriate note duration designation
def dur_to_str(dur):
    d_to_l_dic = {1: '32nd',
                  2: '16th',
                  3: '16th dotted',
                  4: 'eighth',
                  6: 'eighth dotted',
                  8: 'quarter',
                  12: 'quarter dotted',
                  16: 'half',
                  24: 'half dotted',
                  32: 'whole'}

    if dur not in d_to_l_dic:
        print 'Error in <dur_to_str>: invalid duration'
        return
    return d_to_l_dic[dur]

'''
END Deprecated Methods
'''


# test helper methods
def main():
    # test_dic = extract("MIDI_sample.mid")
    # test_dic = extract("uzeb_cool_it.mid")
    # test_note_list = []

    # for k, v in test_dic.iteritems():
    #     # print repr(k)
    #     # print len(v)
    #     test_note_list = v[0]

    # print transpose_note_list(test_note_list, 40, 51)
    # print transpose_instr_tracks_dic(test_dic, 40, 51)

    # song = midi_to_song("MIDI_sample.mid")
    # song = midi_to_song("uzeb_cool_it.mid")
    print os.getcwd()
    song = midi_to_song("./data/ajsmidi/i_gotta_right_hh.mid")
    print song
    # pass


if __name__ == "__main__":
    main()
    pass
