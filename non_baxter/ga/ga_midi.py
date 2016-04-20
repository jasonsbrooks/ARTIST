'''
Note: by default, 100 ticks is a quarter note
thus:
8th: 50 ticks
16th: 25 ticks
32nd: 12.5 ticks
'''

import midi, time, datetime

ed_to_midi_dic_c_maj = {
    1: midi.C_4,
    2: midi.D_4,
    3: midi.E_4,
    4: midi.F_4,
    5: midi.G_4,
    6: midi.A_4,
    7: midi.B_4,
    8: midi.C_5,
    9: midi.D_5,
    10: midi.E_5,
    11: midi.F_5,
    12: midi.G_5,
    13: midi.A_5,
    14: midi.B_5,
    15: midi.C_6,
    16: midi.D_6,
    17: midi.E_6,
    18: midi.F_6,
    19: midi.G_6,
    20: midi.A_6,
    21: midi.B_6
}


# given chromosome = (fitness, genotype)
# creates midi file titled the fitness level
def create_midi_file((fitness, genotype)):
    # Instantiate a MIDI Pattern (contains a list of tracks)
    pattern = midi.Pattern()
    # Instantiate a MIDI Track (contains a list of MIDI events)
    track = midi.Track()
    # Append the track to the pattern
    pattern.append(track)

    for i, (ed, dur) in enumerate(genotype):
        # Instantiate a MIDI note on event, append it to the track
        midi_pitch = midi.B_6
        midi_velocity = 20
        if (ed == -1):  # i.e. a rest
            midi_velocity = 0
        else:
            midi_pitch = ed_to_midi_dic_c_maj[ed]
        on = midi.NoteOnEvent(tick=0, velocity=midi_velocity, pitch=midi_pitch)
        track.append(on)

        # Instantiate a MIDI note off event, append it to the track
        midi_tick_dur = int(dur*27.5)  # 12.5
        off = midi.NoteOffEvent(tick=midi_tick_dur, pitch=midi_pitch)
        track.append(off)

    # Add the end of track event, append it to the track
    eot = midi.EndOfTrackEvent(tick=1)
    track.append(eot)
    # Print out the pattern
    print pattern

    # Save the pattern to disk
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    midi.write_midifile(str(fitness) + "_" + st + ".mid", pattern)
