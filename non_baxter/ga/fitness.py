from chords import get_chord_notes


# given genotype and durk point
# returns pitch being played at that durk point and whether or not pitch ends perfectly on time
# can also take chord_progression instead of genotype and return chord
def get_pitch_at_time(genotype, time):
    pitch = -9999
    dur_running_total = 0
    end_on_time = False
    for i, (ed, dur) in enumerate(genotype):
        dur_running_total += dur
        if dur_running_total > time:
            pitch = ed
            break
        elif dur_running_total == time:
            pitch = ed
            end_on_time = True
            break

    return (pitch, end_on_time)

# returns total penalty for large intervals
# max_interval = 9, weight = -20
def large_intervals(genotype, max_interval=9, weight=20):
    total = 0
    for i, _ in enumerate(genotype):
        if i != 0:
            if abs(genotype[i-1][0] - genotype[i][0]) > max_interval:
                total += weight * -1
    return total


# absolute pitch intervals
# pattern must be 5 notes in length
def pattern_matching(genotype, weight=20):
    pattern_len = 5
    genotype_len = len(genotype)
    total = 0
    patterns = []  # just a list of pitch lists of length pattern_len
    for i in range(genotype_len):
        if i <= genotype_len - pattern_len:
            pattern_to_add = []
            for j in range(5):
                pattern_to_add.append(genotype[i+j][0])
            if pattern_to_add in patterns:
                if pattern_to_add.count(pattern_to_add[0]) == len(pattern_to_add):  # checks if pattern is all repeated notes!
                    total += weight * -1
                else:  # a legit pattern matching!
                    total += weight
            else:
                patterns.append(pattern_to_add)

    return total



# checks what happens to the n-1 chord changes
# Weighting: consonant suspension (+10), dissonant suspension (-20),
# no suspension (+5), a rest (+5)
def suspensions(genotype, chord_progression, weight=20):
    total = 0
    total_dur = 0
    for i, (chord_root, chord_dur) in enumerate(chord_progression):
        total_dur += chord_dur
        (suspension_pitch, end_on_time) = get_pitch_at_time(genotype, total_dur)
        if end_on_time:
            total += weight * .25  # no suspension
        elif suspension_pitch == -1:
            total += weight * .25  # rest
        elif i < len(chord_progression) - 1:
            if suspension_pitch in get_chord_notes(chord_root) and suspension_pitch in get_chord_notes(chord_progression[i+1][0]): # note member of both chords
                total += weight * .5
            else:
                total += weight * -1
        else:
            print 'error in suspensions'

    return total


# first beat of bar
# chord note (+10), rest (+10), non-chord (-20)
# DANGER: assumes each chord in chord_progression spans entire measure
# AND that all chords are the same duration
def downbeat(genotype, chord_progression, weight=20):
    total = 0
    total_dur = 0
    for i, (chord_root, chord_dur) in enumerate(chord_progression):
        (suspension_pitch, _) = get_pitch_at_time(genotype, total_dur)
        if suspension_pitch in get_chord_notes(chord_root):
            total += weight * .5
        elif suspension_pitch == -1:
            total += weight * .5
        else:
            total += weight * -1

        total_dur += chord_dur

    return total


# first beat of bar
# chord note (+10), rest (+10), non-chord (-20)
# DANGER: assumes each chord in chord_progression spans entire measure
# AND that all chords are the same duration
def halfbar(genotype, chord_progression, weight=20):
    d = sum([d for (_, d) in chord_progression])
    total = 0
    total_dur = chord_progression[0][1]/2  # DANGER: ASSUMES that all chords are the same duration
    for i, (chord_root, chord_dur) in enumerate(chord_progression):
        (suspension_pitch, _) = get_pitch_at_time(genotype, total_dur)
        if suspension_pitch in get_chord_notes(chord_root):
            total += weight * .25
        elif suspension_pitch == -1:
            total += weight * .25
        else:
            total += weight * -1

        total_dur += chord_dur

    return total


# generally penalizes long notes
# long_note defaults at 8 (quarter note)
# simplified version from paper:
# if long note: chord note (+10), non-chord note (-20), rest (-20)
def longnote(genotype, chord_progression, weight=20, long_note=8):
    total = 0
    total_dur = 0
    for i, (ed, dur) in enumerate(genotype):
        if dur >= long_note:
            (chord_root, _) = get_pitch_at_time(chord_progression, total_dur)  # DANGER: note used for chord_progression this tiem!
            if ed in get_chord_notes(chord_root):
                total += weight * .5
            else:
                total += weight * -1

        total_dur += dur

    return total


# given a genotype (list of (extended_degree, duration))
# returns fitness value for that genotype
# Detailed: prints (total, Dic containing detailed breakdown)
def calc_fitness(genotype, chord_progression, detailed=False):
    li = large_intervals(genotype)
    pm = pattern_matching(genotype)
    su = suspensions(genotype, chord_progression)
    db = downbeat(genotype, chord_progression)
    hb = halfbar(genotype, chord_progression)
    ln = longnote(genotype, chord_progression)

    total = li + pm + su + db + hb + ln
    if detailed:
        return (total, {"li": li, "pm": pm, "su": su, "db": db, "hb": hb, "ln": ln})
    else:
        return total
