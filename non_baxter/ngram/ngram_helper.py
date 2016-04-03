import sys


# transposes pitch to be same pitch within range [lo,hi] inclusive
# if multiple pitch matches exist within range, matches to 'closest' pitch
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
