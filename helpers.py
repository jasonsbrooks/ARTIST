import midi


# currently assumes there exists only one ProgramChangeEvent per instrument track
#  also assumes that ProgramChangeEvent represents
def extract(midifilename):
    instr_notes_dic = {}  # {instr_key: [track1, track2]} where track1...are list of NoteOnEvents and NoteOffEvents
    pattern = midi.read_midifile(midifilename)

    for track in pattern:
        for event in track:
            if type(event) is midi.ProgramChangeEvent:  # only act if this track is an actual instrument track!
                instr_key = event.data[0]  # event.data[0] contains instrument key
                notes = get_notes(track, instr_key)

                if not(instr_key in instr_notes_dic):
                    instr_notes_dic[instr_key] = [notes]
                else:
                    instr_notes_dic[instr_key].append(notes)

                break  # this assumes that there is only one ProgramChangeEvent per instrument track
    return instr_notes_dic


#  given a track returns NoteOnEvent and NoteOffEvent as list of tuples (on/off, tick, pitch, velocity)
def get_notes(track, instr_key):
    notes = []
    for event in track:
        if type(event) is midi.NoteOnEvent:
            note = (1, event.tick, event.data[0], event.data[1])
            notes.append(note)
        elif type(event) is midi.NoteOffEvent:
            note = (0, event.tick, event.data[0], event.data[1])
            notes.append(note)

    return notes


# test midi
test_dic = extract("MIDI_sample.mid")
# for k,v in test_dic.iteritems():
#     print k
#     print v
