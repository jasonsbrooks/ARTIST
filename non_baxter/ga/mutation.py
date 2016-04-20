import random
import sys


# transposes pitch to be same pitch within range [lo,hi] inclusive
# if multiple pitch matches exist within range, match closest to given pitch
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


# transpose fragment (ed, dur) by random number of degrees
def mut1(genotype, start_index, end_index):
    transpose = random.randint(-5, 5)
    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            genotype[i] = (transpose_pitch(genotype[i][0] + transpose, 1, 21), genotype[i][1])

    return genotype


# permute in time
def mut2(genotype, start_index, end_index):
    fragment = genotype[start_index:end_index+1]
    random.shuffle(fragment)
    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            genotype[i] = fragment[i-start_index]

    return genotype


# sort into ascending or descending pitch
def mut3(genotype, start_index, end_index):
    fragment = genotype[start_index:end_index+1]
    # 50% chance ascneding/descending
    if random.random() < .5:
        fragment.sort(key=lambda x: x[0], reverse=False)
    else:
        fragment.sort(key=lambda x: x[0], reverse=True)

    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            genotype[i] = fragment[i-start_index]

    return genotype


# reverse in time
def mut4(genotype, start_index, end_index):
    fragment = genotype[start_index:end_index+1]
    # 50% chance ascneding/descending
    fragment.reverse()

    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            genotype[i] = fragment[i-start_index]

    return genotype


# change few pitches while maintaining same rhythm
def mut5(genotype, start_index, end_index):
    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            if random.random() < .3:
                genotype[i] = (random.randint(1, 21), genotype[i][1])

    return genotype


# one-note mutation which changes pitch of one note up or down
def mut6(genotype, start_index):
    for i, (ed, dur) in enumerate(genotype):
        if i == start_index:
            if random.random() < .5:
                genotype[i] = (transpose_pitch(1 + genotype[i][0], 1, 21), genotype[i][1])
            else:
                genotype[i] = (transpose_pitch(-1 + genotype[i][0], 1, 21), genotype[i][1])

    return genotype


# mutates in one of many ways locally
# returns new mutated genotype
def local_mutation(chromosome, d, prob_local=.5):
    genotype = chromosome[1]
    len_genotype = len(genotype)
    end_index = random.randint(0, len_genotype-1)
    start_index = random.randint(0, len_genotype-1)
    if start_index > end_index:
        temp = end_index
        end_index = start_index
        random_end_index = temp


    if random.random() < prob_local:
        genotype = mut1(genotype, start_index, end_index)

    if random.random() < prob_local:
        genotype = mut2(genotype, start_index, end_index)

    if random.random() < prob_local:
        genotype = mut3(genotype, start_index, end_index)

    if random.random() < prob_local:
        genotype = mut4(genotype, start_index, end_index)

    if random.random() < prob_local:
        genotype = mut5(genotype, start_index, end_index)

    if random.random() < prob_local:  # one note mutation
        genotype = mut6(genotype, start_index)

    return genotype


# returns mutated chromosome
def mutate(chromosome, d, prob_local=.5):
    chromosome = local_mutation(chromosome, d, prob_local)
    return chromosome