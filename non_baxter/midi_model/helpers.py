'''
Todo:
1) Get all TimeSignatureEvent and SetTempoEvent
2) tag tracks with this metadata (including time signature and tempo)
3) duration in get_notes-- subtracting ticks doesn't seem to give you the
    exact right float value...seems to be a bit short everytime
3b) Figure out how to insert rests into note list from information given
    but do this after you think more about what the duration actually means
4) figure out a way to NOT include drums (key is 0?  same as piano?)
    idea: drums are always on channel 9 (i.e. 0-indexed 10 from https://www.cs.cmu.edu/~music/cmsip/readings/MIDI%20tutorial%20for%20programmers.html)
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

    time_sig_events = []
    # first, create ordered list of all time and key signature events
    for track in pattern:
        for event in track:
            if type(event) is midi.TimeSignatureEvent:
                time_sig = (event.data[0], 2**event.data[1])  # (num, denom), note that in MIDI denom is a negative power of 2




        for event in track:
            if type(event) is midi.ProgramChangeEvent:  # only act if this track is an actual instrument track!
                instr_key = event.data[0]  # event.data[0] contains instrument key
                notes = get_note_events(track)
                print get_notes(notes, resolution)[0:20]


                if not(instr_key in instr_tracks_dic):
                    instr_tracks_dic[instr_key] = [notes]
                else:
                    instr_tracks_dic[instr_key].append(notes)

                break  # this assumes that there is only one ProgramChangeEvent per instrument track
    return instr_tracks_dic

# takes midi file and constructs dictionary with following structure:
# {instr_key: [track1, track2, ...], ...} where track1, track2...are list of tuples returned by get_note_events
# currently assumes there exists only one ProgramChangeEvent per instrument track
def extract(midifilename):
    instr_tracks_dic = {}
    pattern = midi.read_midifile(midifilename)
    pattern.make_ticks_abs()  # makes ticks absolute instead of relative
    resolution = pattern.resolution  # note pattern.resolution contains resolution (ppqn)
    print resolution

    for track in pattern:
        print track[0:20]
        for event in track:
            if type(event) is midi.ProgramChangeEvent:  # only act if this track is an actual instrument track!
                instr_key = event.data[0]  # event.data[0] contains instrument key
                notes = get_note_events(track)
                print get_notes(notes, resolution)[0:20]


                if not(instr_key in instr_tracks_dic):
                    instr_tracks_dic[instr_key] = [notes]
                else:
                    instr_tracks_dic[instr_key].append(notes)

                break  # this assumes that there is only one ProgramChangeEvent per instrument track
    return instr_tracks_dic


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
    test_dic = extract("uzeb_cool_it.mid")
    test_note_list = []

    for k, v in test_dic.iteritems():
        # print repr(k)
        # print len(v)
        test_note_list = v[0]

    # print transpose_note_list(test_note_list, 40, 51)
    # print transpose_instr_tracks_dic(test_dic, 40, 51)


if __name__ == "__main__":
    main()
    pass
