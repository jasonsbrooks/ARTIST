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


# returns weighted choice in choices
def weighted_choice(choices):
   total = sum(w for c, w in choices)
   r = random.uniform(0, total)
   upto = 0
   for c, w in choices:
      if upto + w >= r:
         return c
      upto += w
   assert False, "Shouldn't get here"


# specify size and probabilty best individual in pool wins
# deterministic selection of best individual when p =1
# 1-way tournament (tourn_size = 1) equivalent to random selection
def tournament_selection(chromosomes, tourn_size, prob):
    # first randomly select tourn_size individuals from chromosomes
    competitors = []
    while len(competitors) < tourn_size:
        to_append = chromosomes[random.randint(0, len(chromosomes) - 1)]
        if to_append not in competitors:
            competitors.append(to_append)

    competitors.sort(key=lambda x: x[0])  # sort by increasing fitness
    weighted_probs = []
    for i in range(0, tourn_size):
        weighted_probs.append(prob * ((1 - prob) ** i))

    return weighted_choice(zip(competitors, weighted_probs))


# one point crossover
# returns child chromosome
def crossover(parent1, parent2, d):
    genotype1 = parent1[1]
    genotype2 = parent2[1]

    new_genotype1 = []
    new_genotype2_first = []
    new_genotype2_last = []

    # pick random number between 0 and d
    split = random.randint(0, d)

    total_dur = 0
    add_to_genotype1 = True
    for i, (ed, dur) in enumerate(genotype1):
        total_dur += dur
        if not add_to_genotype1:
            new_genotype2_last.append((ed, dur))
        elif total_dur == split:
            new_genotype1.append((ed, dur))
            add_to_genotype1 = False
        elif total_dur > split:
            new_genotype1.append((ed, (split - (total_dur - dur))))
            new_genotype2_last.append((ed, (total_dur - (split - (total_dur - dur)))))
            add_to_genotype1 = False
        else:
            new_genotype1.append((ed, dur))

    total_dur = 0
    add_to_genotype2 = True
    for i, (ed, dur) in enumerate(genotype2):
        total_dur += dur
        if not add_to_genotype2:
            new_genotype2_first.append((ed, dur))
        elif total_dur == split:
            new_genotype2_first.append((ed, dur))
            add_to_genotype2 = False
        elif total_dur > split:
            new_genotype2_first.append((ed, (split - (total_dur - dur))))
            new_genotype1.append((ed, (total_dur - (split - (total_dur - dur)))))
            add_to_genotype2 = False
        else:
            new_genotype2_first.append((ed, dur))

    new_genotype2 = new_genotype2_first + new_genotype2_last

    # check to see if crossover worked
    if sum([a for (_, a) in new_genotype1]) == sum([a for (_, a) in new_genotype2]):
        print "Worked!"
    else:
        print "shoot..."

    if sum([a for (_, a) in genotype1]) == sum([a for (_, a) in genotype2]):
        print "Peace"
    else:
        print "poop"

    return (new_genotype1, new_genotype2)



# runs ga on
# population of size n
def ga(n, chord_progression, num_iter):
    d = sum([d for (_, d) in chord_progression])  # d is total duration of song
    chromosome_list = initialize_chromosomes(n, d)  # list of (fitness, genotype)

    for _ in range(0, num_iter):
        new_chromosome_list = []
        # Elitism?
        # keep the highest fitness chromosome
        chromosome_list.sort(key=lambda x: x[0])  # sort by increasing fitness
        new_chromosome_list.append(chromosome_list[-1])

        # Crossover n times
        for i in range(n):
            parent1 = tournament_selection(chromosome_list, 4, .8)
            parent2 = tournament_selection(chromosome_list, 4, .8)
            (child1, child2) = crossover(parent1, parent2, d)

            # calculate fitness of both children, and add higher to chromosomes
            new_chromosome_list.append((-99, child1))


        # Mutate based on certain probabilities
        # decide on hill-climbing (don't replace parent if it was not at
        # least as fit!)

        # OPERATE (mate)


def main():
    chord_progression = [(-1,32) for _ in range(0, 24)] # list of (chord, duration)
    ga(40, chord_progression, 200)
    pass


if __name__ == "__main__":
    main()
