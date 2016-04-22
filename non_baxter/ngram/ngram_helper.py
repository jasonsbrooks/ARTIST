import sys
from db import InvalidKeySignature

# transposes pitch to be same pitch within range [lo,hi] inclusive
# if multiple pitch matches exist within range, matches to 'closest' pitch
def range_transpose_pitch(pitch, lo, hi):
    octave = 12

    if (hi - lo) < (octave - 1):
        print "Error in range_transpose_pitch: hi - lo < 11, make sure range spans a full octave"
        sys.exit(1)

    while pitch < lo:
        pitch += octave
    while pitch > hi:
        pitch -= octave

    return pitch


# given a pitch (MIDI integer from 0-127)
# returns correpsonding string representation of note
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


# key_sig: (sf, mi)
#    sf = -7 =>  flats, sf = 4 => 4 sharps, mi = 0 => major key, mi = 1 => minor key
# takes a pitch, and converts from from_ks to appropriate key in to_ks
# ensures pitch in range [0, 127]
# Note: could convert in either positive or negative direction right now!
#   so diff range in pitch is [-11, 11]
def key_transpose_pitch(pitch, from_ks, to_ks):

    if from_ks[0] > 7:
        from_ks[0] = from_ks[0] - 256

    if to_ks[0] > 7:
        to_ks[0] = to_ks[0] - 256

    octave = 12

    sf_to_root_major = {7: 1,  # Db
                        6: 6,  # Gb/F#
                        5: 11,  # B
                        4: 4,  # E
                        3: 9,  # A
                        2: 2,  # D
                        1: 7,  # G
                        0: 0,  # C
                        -1: 5,  # F
                        -2: 10,  # Bb
                        -3: 3,  # Eb
                        -4: 8,  # Ab
                        -5: 1,  # Db
                        -6: 6,  # Gb/F#
                        -7: 11  # B
                        }

    try:
        if from_ks[1] == 0:  # major
            from_root = sf_to_root_major[from_ks[0]]
        else:  # minor
            from_root = (sf_to_root_major[from_ks[0]] - 3) % octave

        if to_ks[1] == 0:
            to_root = sf_to_root_major[to_ks[0]]
        else:
            to_root = (sf_to_root_major[to_ks[0]] - 3) % octave
    except Exception as e:
        print 'Exception in key_tranpose_pitch: invalid key signature, ' + str(e)
        raise InvalidKeySignature()

    diff = to_root - from_root  # Note: wrong assupmtion that diff is always positive...
    new_pitch = pitch + diff

    new_pitch = range_transpose_pitch(new_pitch, 0, 127)  # make sure pitch within [0,127]

    return new_pitch
