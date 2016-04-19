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


# given a genotype (list of (extended_degree, duration))
# returns fitness value for that genotype
def fitness(genotype):
    total = 0


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

        chromosomes.append(fitness, genotype)
    return chromosomes


# runs ga on
# population of size n
def ga(n, chord_progression):
    d = sum([d for (_, d) in chord_progression])  # d is total duration of song
    chromosome_list = initialize_chromosomes(n, d)  # list of (fitness, genotype)


def main():
    chord_progression = [] # list of (chord, duration)
    ga(100, chord_progression)
    pass


if __name__ == "__main__":
    main()
