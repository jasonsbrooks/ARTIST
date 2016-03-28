'''
Complete:
1) Get all TimeSignatureEvent and SetTempoEvent
2) tag tracks with this metadata (including time signature and tempo)

Todo:
2) investigate why occasional on/off note event mismatch errors
3) duration in get_notes-- subtracting ticks doesn't seem to give you the
    exact right float value...seems to be a bit short everytime
3b) Figure out how to insert rests into note list from information given
    but do this after you think more about what the duration actually means
4) figure out a way to NOT include drums (key is 0?  same as piano?)
    idea: drums are always on channel 9 (i.e. 0-indexed 10 from https://www.cs.cmu.edu/~music/cmsip/readings/MIDI%20tutorial%20for%20programmers.html)
5) Add naive tempo/dynamics in track

Important MIDI Notes:
1) NoteOnEvent with velocity of 0 === NoteOffEvent
2) pattern.resolution contains resolution (ppqn)
'''

import midi
import sys
from song import Song
from track import Track
from note import Note
from collections import defaultdict

DURKS_PER_QUARTER_NOTE = 8


# Takes a track object
# Inserts rests into appropriate locations (where no pitch is being played)
# Returns nothing
# Danger: directly modifies the track object that is passed to it!
def insert_rests_into_track(track, ppqn):
    # find total number of durks in track
    max_durk = track[-1].start + track[-1].dur
    min_durk = track[0].start

    # initialize rest list (each entry represents one durk)
    rest_list = [True for i in range(max_durk)]

    # change all durks in rest_list to 0 where note is being played
    for note in track:
        for durk in range(note.dur):
            rest_list[note.start + durk] = False  # set rest_list to False for each durk where a note pitch is playing

    # iterate through completed rest_list and insert appropriate rests into track
    current_durk = min_durk
    while current_durk < max_durk:
        if rest_list[current_durk]:  # rest found!
            rest_start = current_durk  # store start of rest note

            while rest_list[current_durk]:  # increment current_durk until pitch found
                current_durk += 1

            rest_dur = current_durk - rest_start
            rest_measure = rest_start / (DURKS_PER_QUARTER_NOTE * track.time_sig[1])  # note 8 is number of durks per quarter note

            note = Note(pitch=-1, dur=rest_dur, start=rest_start,
                        tick_dur=-1, start_tick=-1, measure=rest_measure)

            track.append(note)  # store this contiguous rest block as "rest note" in the track

        else:
            current_durk += 1

    return


# takes midi file and returns a representative song object
# a Song is a list of many Tracks
# a Track is a list of many Notes representing a common
#   1) key signature, 2) time signature, and 3) instrument key
def midi_to_song(midifilename):
    pattern = midi.read_midifile(midifilename)
    pattern.make_ticks_abs()  # makes ticks absolute instead of relative
    resolution = pattern.resolution  # note pattern.resolution contains resolution (ppqn)
    print 'Resolution (ppqn): ' + str(resolution)

    # create Song object
    song = Song(title=midifilename, ppqn=resolution)

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

            track = Track(time_sig=(ts['n'], ts['d']), key_sig=(ks['sf'], ks['mi']),
                          instr_key=instr_key, instr_name=instr_name, channel=channel,
                          start_tick=0)

            # init these next temp vars in case no sig events left
            next_sig_event = None
            next_track_tick = sys.maxint

            if len(all_sig_events) > 0:
                next_sig_event = all_sig_events[0]
                next_track_tick = next_sig_event['start_tick']  # holds max tick of current track

            notes = get_notes(get_note_events(midi_track), resolution, track)

            for n in notes:
                if n.start_tick < next_track_tick:  # still on current track
                    track.append(n)
                else:  # must append current track to song and create new track for this note
                    # append current track to song
                    song.append(track)

                    # update key or time signature
                    if next_sig_event['type'] == 'key':
                        ks = next_sig_event
                    elif next_sig_event['type'] == 'time':
                        ts = next_sig_event

                    # create new track and update temporary variables
                    track = Track(time_sig=(ts['n'], ts['d']), key_sig=(ks['sf'], ks['mi']),
                                  instr_key=instr_key, instr_name=instr_name,
                                  channel=channel,
                                  start_tick=next_sig_event['start_tick'])

                    all_sig_events.pop(0)

                    if len(all_sig_events) > 0:
                        next_sig_event = all_sig_events[0]
                        next_track_tick = next_sig_event['start_tick']
                    else:  # if no events left, set next_track_tick to maxint
                        next_sig_event = None
                        next_track_tick = sys.maxint

                    # append note to new track
                    track.append(n)

            insert_rests_into_track(track, resolution)  # insert rests into track
            song.append(track)  # final append of track

    return song


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
def get_notes(note_events, ppqn, track):
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
                measure = note_event[1] / (ppqn * track.time_sig[1])
                print 'Error: <get_notes> NoteOffEvent without corresponding NoteOnEvent %r, measure: %r, instr: %r, pitch: %r ' % (str(note_event), str(measure), track.instr_name, pitch_to_str(pitch))
            else:
                note_on = unclosed_notes[pitch].pop(0)
                start_tick = note_on[1]
                tick_dur = tick - start_tick

                start = .25 * start_tick / ppqn  # 1 === whole note, .25 === quarter note
                start = int(round(start * 32))  # final conversion to durk units: 1 === a 32nd note....32  === a whole note
                dur = .25 * tick_dur / ppqn  # 1 === whole note, .25 === quarter note
                dur = int(round(dur * 32))  # final conversion to durk units: 1 === a 32nd note....32  === a whole note

                measure = start_tick / (ppqn * track.time_sig[1])

                note_obj = Note(pitch=pitch, dur=dur, start=start,
                                tick_dur=tick_dur, start_tick=start_tick, measure=measure)
                note_objs.append(note_obj)
        else:  # Error checking
            print 'Error: <get_notes> Note is neither On nor Off event'

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

    song = midi_to_song("MIDI_sample.mid")
    # song = midi_to_song("uzeb_cool_it.mid")
    print song


if __name__ == "__main__":
    main()
    pass
