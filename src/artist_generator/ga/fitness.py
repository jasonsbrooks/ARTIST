from chords import get_chord_notes


# given genotype and durk point
# returns pitch being played at that durk point and whether or not pitch ends perfectly on time
# can also take chord_progression instead of genotype and return chord
def get_pitch_at_time(genotype, time):
    """given genotype and durk time point, returns pitch being played at that durk point and whether or not pitch ends perfectly on time
    
    Args:
        genotype ((int, int)[]): genotype of chromosome, which is list of (pitch, dur)
        time (int): time point in durks
    
    Returns:
        (int, bool): (pitch, end_on_time) where pitch is [1,21] and end_on_time represents whether or not the pitch ends perfectly at the time point time.
    """
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
    """takes genotype and gives fitness penalty for large intervals 
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        max_interval (int, optional): Defaults at 9.  Defines max interval before penalty begins
        weight (int, optional): Defaults at 20.  Defines penalty/reward rate
    
    Returns:
        int: aggregate fitness value based on large intervals
    """
    total = 0
    for i, _ in enumerate(genotype):
        if i != 0:
            if abs(genotype[i-1][0] - genotype[i][0]) > max_interval:
                total += weight * -1
    return total


# absolute pitch intervals
# pattern must be 5 notes in length
def pattern_matching(genotype, weight=20):
    """fitness function handling absolute pitch intervals
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        weight (int, optional): Defaults at 20.  Defines penalty/reward rate
    
    Returns:
        int: aggregate fitness value based on pattern matching
    """
    # pattern_len = 3
    pattern_len = 5
    genotype_len = len(genotype)
    total = 0
    patterns = []  # just a list of pitch lists of length pattern_len
    for i in range(genotype_len):
        if i <= genotype_len - pattern_len:
            pattern_to_add = []
            for j in range(pattern_len):
                pattern_to_add.append(genotype[i+j][0])
            if pattern_to_add in patterns:
                if pattern_to_add.count(pattern_to_add[0]) == len(pattern_to_add):  # checks if pattern is all repeated notes!
                    total += weight * -10
                else:  # a legit pattern matching!
                    total += weight
            else:
                patterns.append(pattern_to_add)

    return total


# checks what happens to the n-1 chord changes
# Weighting: consonant suspension (+10), dissonant suspension (-20),
# no suspension (+5), a rest (+5)
def suspensions(genotype, chord_progression, weight=20):
    """fitness function handling suspensions
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        chord_progression ((int, int)[]): chord progression of song as list of (chord_root, dur)
        weight (int, optional): Defaults at 20.  Defines penalty/reward rate
    
    Returns:
        int: aggregate fitness value based on suspensions
    """
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
            if suspension_pitch in get_chord_notes(chord_root) and suspension_pitch in get_chord_notes(chord_progression[i+1][0]):  # note member of both chords
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
    """fitness function handling downbeats
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        chord_progression ((int, int)[]): chord progression of song as list of (chord_root, dur)
        weight (int, optional): Defaults at 20.  Defines penalty/reward rate
    
    Returns:
        int: aggregate fitness value based on downbeats
    """
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
    """fitness function handling halfbars
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        chord_progression ((int, int)[]): chord progression of song as list of (chord_root, dur)
        weight (int, optional): Defaults at 20.  Defines penalty/reward rate
    
    Returns:
        int: aggregate fitness value based on halfbars
    """
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
    """fitness function handling long notes
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        chord_progression ((int, int)[]): chord progression of song as list of (chord_root, dur)
        weight (int, optional): Defaults at 20.  Defines penalty/reward rate
        long_note (int, optional): Defaults at 8.  Defines limit for what constitutes a "long note" (anything >= to long_note is considered "long")
    
    Returns:
        int: aggregate fitness value based on long notes
    """
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


# penalize all shorter than 8th note
def penalize_short(genotype, weight=20):
    """fitness function handling short notes
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        weight (int, optional): Defaults at 20.  Defines penalty/reward rate
    
    Returns:
        int: aggregate fitness value based on short notes
    """
    total = 0
    for i, (ed, dur) in enumerate(genotype):
        if dur < 4:
            total += weight * -1
    return total


# pulse is the "eighth note" pulse
# assume that's every "4"
# DANGER: this is terrible
# also reward off-beat pulse
def penalize_off_pulse(genotype, weight=20):
    """fitness function handling off pulse notes
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        weight (int, optional): Defaults at 20.  Defines penalty/reward rate
    
    Returns:
        int: aggregate fitness value based on off pulse notes
    """
    total = 0
    total_dur = 0
    for i, (ed, dur) in enumerate(genotype):
        total_dur += dur
        if not total_dur % 4 == 0:
            total += weight * -.25
        if total_dur % 4 == 0 and total_dur % 8 != 0:  # means off beat!
            total += weight * .25
    return total


# reward last note if it is root
def end_on_root(genotype, weight=20):
    """fitness function handling last note
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        weight (int, optional): Defaults at 20.  Defines penalty/reward rate
    
    Returns:
        int: aggregate fitness value based on last note
    """
    total = 0
    if genotype[-1][0] == 1 or genotype[-1][0] == 8 or genotype[-1][0] == 15:
        total += weight * 1
    else:
        total += weight * -1
    return total

# given a genotype (list of (extended_degree, duration))
# returns fitness value for that genotype
# Detailed: prints (total, Dic containing detailed breakdown)
def calc_fitness(genotype, chord_progression, detailed=False):
    """calculates and returns total fitness based on a given genotype and chord progression
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        chord_progression ((int, int)[]): chord progression of song as list of (chord_root, dur)
        detailed (bool, optional): Defaults at False.  If True, prints to console detailed fitness breakdown.  Otherwise, prints nothing
    
    Returns:
        int: total fitness of the given genotype for particular chord progression
    """
    li = large_intervals(genotype,10)
    pm = pattern_matching(genotype, weight=10)
    su = suspensions(genotype, chord_progression)
    db = downbeat(genotype, chord_progression)
    hb = halfbar(genotype, chord_progression)
    ln = longnote(genotype, chord_progression)
    er = end_on_root(genotype)
    ps = penalize_short(genotype, weight=80)
    pop = penalize_off_pulse(genotype, weight=20)

    total = li + pm + su + db + hb + ln + er + pop + ps
    # total = pm
    if detailed:
        return (total, {"li": li, "pm": pm, "su": su, "db": db, "hb": hb, "ln": ln, "ps": ps, "pop": pop, "er": er})
    else:
        return total
