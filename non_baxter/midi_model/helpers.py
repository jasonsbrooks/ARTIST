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
    for track in pattern:
        instr_track_data = is_instr_track(track)
        if instr_track_data:
            instr_key = instr_track_data[0]
            instr_name = instr_track_data[1]
            channel = instr_track_data[2]

            notes = get_notes(get_note_events(track), resolution)

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
                          ppqn=resolution, start_tick=0)

            # init these next temp vars in case no sig events left
            next_sig_event = None
            next_track_tick = sys.maxint

            if len(all_sig_events) > 0:
                next_sig_event = all_sig_events[0]
                next_track_tick = next_sig_event['start_tick']  # holds max tick of current track

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
                                  channel=channel, ppqn=resolution,
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
            song.append(track)  # final append of track

    return song


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
# returns list of note objects with appropriate durations
# this method calculates appropriate durations based on ppqn of track
def get_notes(note_events, ppqn):
    note_objs = []
    unclosed_notes = {}  # holds running hash of notes that haven't seen an off event yet
                         # key is note pitch (integer from 0-127), value is note_event tuple
    for note_event in note_events:
        on_off = note_event[0]
        tick = note_event[1]
        pitch = note_event[2]
        if on_off == 1:  # NoteOnEvent
            if pitch in unclosed_notes:  # error check if two consecutive noteonevents without noteoffevent
                print 'Error: <get_notes> consecutive unclosed NoteOnEvent of same pitch: ' + str(note_event)
            unclosed_notes[pitch] = note_event
        elif on_off == 0:  # NoteOffEvent
            if pitch not in unclosed_notes:
                print 'Error: <get_notes> NoteOffEvent without corresponding NoteOnEvent' + str(note_event)
            else:
                note_on = unclosed_notes[pitch]
                start_tick = note_on[1]
                tick_dur = tick - start_tick
                dur = 1.0 * tick_dur / ppqn
                note_obj = Note(pitch=pitch, dur=dur, tick_dur=tick_dur, start_tick=start_tick, ppqn=ppqn)
                note_objs.append(note_obj)
                del unclosed_notes[pitch]  # delete this pitch key in unclosed_notes dictionary
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
    print song


if __name__ == "__main__":
    main()
    pass
