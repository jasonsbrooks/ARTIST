'''
Todo:
1) Get all TimeSignatureEvent and SetTempoEvent
2) tag tracks with this metadata (including time signature and tempo)
'''


import midi
import sys


# takes midi file and constructs dictionary with following structure:
# {instr_key: [track1, track2, ...], ...} where track1, track2...are list of tuples returned by get_notes
# currently assumes there exists only one ProgramChangeEvent per instrument track
def extract(midifilename):
    instr_tracks_dic = {}
    pattern = midi.read_midifile(midifilename)

    for track in pattern:
        print track[0:20]
        for event in track:
            if type(event) is midi.ProgramChangeEvent:  # only act if this track is an actual instrument track!
                instr_key = event.data[0]  # event.data[0] contains instrument key
                notes = get_notes(track)

                if not(instr_key in instr_tracks_dic):
                    instr_tracks_dic[instr_key] = [notes]
                else:
                    instr_tracks_dic[instr_key].append(notes)

                break  # this assumes that there is only one ProgramChangeEvent per instrument track
    return instr_tracks_dic


# given a track returns NoteOnEvent and NoteOffEvent as list of tuples (on/off, tick, pitch, velocity)
def get_notes(track):
    notes = []
    for event in track:
        if type(event) is midi.NoteOnEvent:
            note = (1, event.tick, event.data[0], event.data[1])
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


# transposes list of notes in structure returned by function get_notes
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
