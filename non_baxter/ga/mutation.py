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


# one-note mutation which changes pitch of all notes in range note up or down
def mut6(genotype, start_index, end_index):
    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            if random.random() < .5:
                genotype[i] = (transpose_pitch(1 + genotype[i][0], 1, 21), genotype[i][1])
            else:
                genotype[i] = (transpose_pitch(-1 + genotype[i][0], 1, 21), genotype[i][1])

    return genotype


# concatenate contiguous rests and identical pitches
def mut7(genotype, start_index, end_index):
    last_pitch = -99
    last_dur = -1000
    new_genotype = []
    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index and i > 0:
            if ed == last_pitch:
                new_genotype.pop()
                new_genotype.append((ed, last_dur + dur))
                last_pitch = ed
                last_dur = last_dur + dur
            else:
                new_genotype.append((ed, dur))
                last_pitch = ed
                last_dur = dur
        else:
            new_genotype.append((ed, dur))
            last_pitch = ed
            last_dur = dur

    return new_genotype


# copies randomly chosen fragment to a different position
def mut8(genotype, start_index, end_index):
    len_genotype = len(genotype)
    genotype_dur = sum([d for (_, d) in genotype])
    if end_index - start_index > (len_genotype/2):
        end_index -= len_genotype/2 - (end_index - start_index)

    fragment = genotype[start_index:end_index]
    fragment_dur = sum([d for (_, d) in fragment])

    new_genotype = []

    # pick random spot to insert fragment
    insert_index = random.randint(0, (len_genotype-1) - (end_index - start_index))

    skip_notes_flag = False
    for i, (ed, dur) in enumerate(genotype):
        if skip_notes_flag:
            fragment_dur -= dur
            if fragment_dur < 0:
                new_genotype.append((ed, fragment_dur * -1))
                skip_notes_flag = False
            elif fragment_dur == 0:
                skip_notes_flag = False
        elif i == insert_index:
            new_genotype += fragment
            skip_notes_flag = True
        else:
            new_genotype.append((ed, dur))

    # if new_genotype too long, trim end
    total_dur = 0
    print new_genotype
    for i, (ed, dur) in enumerate(new_genotype):
        total_dur += dur
        if total_dur >= genotype_dur:
            new_genotype[i] = (ed, dur - (total_dur - genotype_dur))
            new_genotype = new_genotype[0:i+1]

    if sum([d for (_, d) in genotype]) != sum([d for (_, d) in new_genotype]):
        return "unequal lengths mut8"

    return new_genotype


def get_random_start_end(len_genotype):
    end_index = random.randint(0, len_genotype-1)
    start_index = random.randint(0, len_genotype-1)
    if start_index > end_index:
        temp = end_index
        end_index = start_index
        random_end_index = temp

    return (start_index, end_index)


# mutates in one of many ways locally
# returns new mutated genotype
def local_mutation(chromosome, d, prob_local=.5):
    genotype = chromosome[1]
    len_genotype = len(genotype)

    if random.random() < prob_local:
        (start_index, end_index) = get_random_start_end(len_genotype)
        genotype = mut1(genotype, start_index, end_index)

    if random.random() < prob_local:
        (start_index, end_index) = get_random_start_end(len_genotype)
        genotype = mut2(genotype, start_index, end_index)

    if random.random() < prob_local:  # this mutation seems problematic
        (start_index, end_index) = get_random_start_end(len_genotype)
        genotype = mut3(genotype, start_index, end_index)

    if random.random() < prob_local:
        (start_index, end_index) = get_random_start_end(len_genotype)
        genotype = mut4(genotype, start_index, end_index)

    if random.random() < prob_local:
        (start_index, end_index) = get_random_start_end(len_genotype)
        genotype = mut5(genotype, start_index, end_index)

    if random.random() < prob_local:  # one note mutation
        (start_index, end_index) = get_random_start_end(len_genotype)
        genotype = mut6(genotype, start_index, end_index)

    if random.random() < prob_local:
        (start_index, end_index) = get_random_start_end(len_genotype)
        genotype = mut7(genotype, start_index, end_index)

    # if random.random() < prob_local:
    #     (start_index, end_index) = get_random_start_end(len_genotype)
    #     genotype = mut8(genotype, start_index, end_index)

    return genotype


# returns mutated chromosome
def mutate(chromosome, d, prob_local=.5):
    chromosome = local_mutation(chromosome, d, prob_local)
    return chromosome
