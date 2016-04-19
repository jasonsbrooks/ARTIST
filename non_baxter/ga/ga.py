'''
to do
0) find easy way to categorize notes in chord vs notes not in chord to...
1) finish all fitness function parameters

'''

import random

# duration in durk units: 1 === a 32nd note....32  === a whole note
# return duration that corresponds to one of following notes:
#   32nd: 1 16th: 2, 8th: 4, quarter: 8, half: 16 AND
#   dotted 16th: 3, dotted 8th: 6, dotted quarter: 12
# in all, 1, 2, 3, 4, 6, 8, 12, 16
def choose_duration():
    possible_durations = [1, 2, 3, 4, 6, 8, 12, 16]
    return possible_durations[random.randint(0, len(possible_durations) - 1)]

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



# given a genotype (list of (extended_degree, duration))
# returns fitness value for that genotype
def calc_fitness(genotype):
    total = 0
    total += large_intervals(genotype, 9, -20)
    total += pattern_matching(genotype, 20)
    return total



# returns list of (fitness, genotype)
# fitness holds junk value (-99)-- fitness function used to evaluate it
def initialize_chromosomes(n, d):
    chromosomes = []
    for _ in range(n):  # makes n chromosomes
        fitness = -99
        genotype = []
        total_duration = 0
        while total_duration < d:
            # choose rest 12.5% of time for extended_degree
            extended_degree = random.randint(1, 21)
            if random.randint(1, 8) == 8:
                extended_degree = -1

            duration = choose_duration()
            while (duration + total_duration > d):
                duration = choose_duration()  # ensures duration of chromosomes equal to d

            genotype.append((extended_degree, duration))
            total_duration += duration

        fitness = calc_fitness(genotype)
        chromosomes.append((fitness, genotype))
    return chromosomes


# runs ga on
# population of size n
def ga(n, chord_progression):
    d = sum([d for (_, d) in chord_progression])  # d is total duration of song
    chromosome_list = initialize_chromosomes(n, d)  # list of (fitness, genotype)
    



def main():
    chord_progression = [(-1,32) for _ in range(0, 24)] # list of (chord, duration)
    ga(100, chord_progression)
    pass


if __name__ == "__main__":
    main()
