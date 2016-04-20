from chords import get_chord_notes


# returns total penalty for large intervals
# max_interval = 9, weight = -20
def large_intervals(genotype, max_interval, weight):
    total = 0
    for i, _ in enumerate(genotype):
        if i != 0:
            if abs(genotype[i-1][0] - genotype[i][0]) > max_interval:
                total += weight
    return total


# different from paper for now!
# ignore overlapped patterns
# for now, only absolute pitch intervals
# pattern must be 5 notes in length
def pattern_matching(genotype, weight):
    pattern_len = 5
    total = 0
    intervals = []
    for i, _ in enumerate(genotype):
        if i != 0:
            intervals.append(genotype[i][0] - genotype[i-1][0])

    # check for repeats of intervals
    for i, _ in enumerate(intervals):
        if i >= pattern_len:
            curr_pattern = [intervals[i] for i in range(i-5, i)]
            pattern_good = 0
            first_time_pattern_seen = True
            for j, _ in enumerate(intervals):
                if j <= len(intervals) - pattern_len:
                    if intervals[j] == curr_pattern[pattern_good]:
                        pattern_good += 1
                    else:
                        pattern_good = 0

                    if pattern_good == 5:
                        if (first_time_pattern_seen):
                            first_time_pattern_seen = False
                        else:
                            total += weight
                        pattern_good = 0

    return total


# checks what happens to the n-1 chord changes
# Weighting: consonant suspension (+10), dissonant suspension (-20),
# no suspension (+5), a rest (+5)
def suspensions(genotype, weight=20):
    total = 0
    dur = 0
    measure_dur = 0
    for i, (ed, dur) in enumerate(genotype):
        measure_dur += dur
        if measure_dur > 32:
            pass

        elif measure_dur == 32:  # no suspension
            total += weight * (.25)
        else:
            continue



    return total



# given a genotype (list of (extended_degree, duration))
# returns fitness value for that genotype
def calc_fitness(genotype):
    total = 0
    total += large_intervals(genotype, 9, -20)
    total += pattern_matching(genotype, 20)
    total += suspensions(genotype)
    return total
