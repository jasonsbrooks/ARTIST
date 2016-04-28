import sys
from exceptions import InvalidKeySignature

def range_transpose_pitch(pitch, lo, hi):
    """
    Transposes pitch to be same pitch within range [lo,hi] inclusive.
    If multiple pitch matches exist within range, matches to 'closest' pitch

    Args:
        pitch (int): pitch to transpose
        lo (int): lower bound
        hi (int): upper bound

    Returns:
        int: transposed MIDI pitch
    """
    octave = 12

    if (hi - lo) < (octave - 1):
        print "Error in range_transpose_pitch: hi - lo < 11, make sure range spans a full octave"
        sys.exit(1)

    while pitch < lo:
        pitch += octave
    while pitch > hi:
        pitch -= octave

    return pitch


def pitch_to_str(pitch):
    """
    Calculate the corresponding string representation of a note, given the MIDI pitch number
    Args:
        pitch (int): MIDI pitch number

    Returns:
        str: corresponding note name
    """
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


def key_transpose_pitch(pitch, from_ks, to_ks):
    """
    Takes a pitch and transposes it from from_ks to to_ks.

    Note: could convert in either positive or negative direction right now! so diff range in pitch is [-11, 11]

    Args:
        pitch (int): MIDI number of the pitch
        from_ks: source key signature (top,bottom)
        to_ks: destination key signature (top,bottom)

    Returns:
        int: transposed MIDI pitch
    """

    if from_ks[0] > 7:
        from_ks = (from_ks[0] - 256,from_ks[1])

    if to_ks[0] > 7:
        to_ks = (to_ks[0] - 256,to_ks[1])

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
