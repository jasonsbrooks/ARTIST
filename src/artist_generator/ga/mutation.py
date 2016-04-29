import random
import sys


# transposes pitch to be same pitch within range [lo,hi] inclusive
# if multiple pitch matches exist within range, match closest to given pitch
def transpose_pitch(pitch, lo, hi):
    """transposes given pitch to be same pitch within range [lo,hi] inclusive
    if pitch cannot be tranposed into given range, prints error message and exits
    
    Args:
        pitch (int): pitch to be transposed
        lo (int): lower bound of transpose range
        hi (int): higher bound of transpose range
    
    Returns:
        int: succesfully transposed pitch
    """
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
    """transpose chromosome fragment by random number of degrees from [-5, 5] inclusive
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        start_index (int): index where fragment begins
        end_index (int): index where fragment ends
    
    Returns:
        (int, int)[]: returns new genotype with mutated fragment
    """
    transpose = random.randint(-5, 5)
    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            genotype[i] = (transpose_pitch(genotype[i][0] + transpose, 1, 21), genotype[i][1])

    return genotype


# permute in time
def mut2(genotype, start_index, end_index):
    """permutes chromosome fragment randomly in time
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        start_index (int): index where fragment begins
        end_index (int): index where fragment ends
    
    Returns:
        (int, int)[]: returns new genotype with mutated fragment
    """
    fragment = genotype[start_index:end_index+1]
    random.shuffle(fragment)
    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            genotype[i] = fragment[i-start_index]

    return genotype


# sort into ascending or descending pitch
def mut3(genotype, start_index, end_index):
    """sorts chromosome fragment into either ascending or descending pitch
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        start_index (int): index where fragment begins
        end_index (int): index where fragment ends
    
    Returns:
        (int, int)[]: returns new genotype with mutated fragment
    """
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
    """reverses chromosome fragment in time
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        start_index (int): index where fragment begins
        end_index (int): index where fragment ends
    
    Returns:
        (int, int)[]: returns new genotype with mutated fragment
    """
    fragment = genotype[start_index:end_index+1]
    fragment.reverse()

    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            genotype[i] = fragment[i-start_index]

    return genotype


# change few pitches while maintaining same rhythm
def mut5(genotype, start_index, end_index):
    """changes few pitches in chromosome fragment while maintaing the same rhythm
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        start_index (int): index where fragment begins
        end_index (int): index where fragment ends
    
    Returns:
        (int, int)[]: returns new genotype with mutated fragment
    """
    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            if random.random() < .3:
                genotype[i] = (random.randint(1, 21), genotype[i][1])

    return genotype


# one-note mutation which changes pitch of all notes in range note up or down
def mut6(genotype, start_index, end_index):
    """one-note mutation which changes pitch of all notes in fragment up or down by 1
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        start_index (int): index where fragment begins
        end_index (int): index where fragment ends
    
    Returns:
        (int, int)[]: returns new genotype with mutated fragment
    """
    for i, (ed, dur) in enumerate(genotype):
        if i >= start_index and i <= end_index:
            if random.random() < .5:
                genotype[i] = (transpose_pitch(1 + genotype[i][0], 1, 21), genotype[i][1])
            else:
                genotype[i] = (transpose_pitch(-1 + genotype[i][0], 1, 21), genotype[i][1])

    return genotype


# concatenate contiguous rests and identical pitches
def mut7(genotype, start_index, end_index):
    """concatenates contiguous rests and identical pitches in chromosome fragment
    
    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        start_index (int): index where fragment begins
        end_index (int): index where fragment ends
    
    Returns:
        (int, int)[]: returns new genotype with mutated fragment
    """
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


# inserts randomly chosen fragment to a different position
# trims end to make dur consistent
def mut8(genotype, start_index, end_index):
    """inserts randomly chosen fragment to a different position in the same chromosome genotype

    Args:
        genotype ((int, int)[]): list of tuples (pitch, dur) representing genotype of a chromosome
        start_index (int): index where fragment begins
        end_index (int): index where fragment ends
    
    Returns:
        (int, int)[]: returns new genotype with mutated fragment
    """
    old_genotype_dur = sum([d for (_, d) in genotype])
    fragment = genotype[start_index:end_index]
    new_genotype = []
    for elem in reversed(fragment):
        genotype.insert(start_index, elem)

    # trim end of genotype
    total_dur = 0
    for i, (ed, dur) in enumerate(genotype):
        total_dur += dur
        if total_dur >= old_genotype_dur:
            new_genotype.append((ed, dur - (total_dur - old_genotype_dur)))
            break
        else:
            new_genotype.append((ed, dur))

    if sum([d for (_, d) in new_genotype]) != old_genotype_dur:
        print 'mut8 duration error'

    return new_genotype


def get_random_start_end(len_genotype):
    """gets a random start and end type for genotype
    used to get start and end times for fragments, used in mutations
    
    Args:
        len_genotype (int): length of genotype list
    
    Returns:
        (int, int): tuple of two integers representing randomly generated (start_index, end_index)
    """
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
    """mutates chromosome in one of many ways locally
    
    Args:
        chromosome ((int, (int, int)[])): Chromosome is a tuple of (fitness, genotype).  Genotype is a list of tuples (pitch, dur) representing music to be played
        d (int): Length, in durks, representing total length of the full song
        prob_local (float, optional): Defaults to .5.  Probability that any mutation will happen
    
    Returns:
        (int, int)[]: returns new randomly mutated genotype
    """
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

    if random.random() < prob_local:
        (start_index, end_index) = get_random_start_end(len_genotype)
        genotype = mut8(genotype, start_index, end_index)

    return genotype


# returns mutated chromosome
def mutate(chromosome, d, prob_local=.5):
    """given chromosome, returns mutated chromosome
    calls local_mutation
    
    Args:
        chromosome ((int, (int, int)[])): Chromosome is a tuple of (fitness, genotype).  Genotype is a list of tuples (pitch, dur) representing music to be played
        d (int): Length, in durks, representing total length of the full song
        prob_local (float, optional): Defaults to .5.  Probability that any mutation will happen
    
    Returns:
        (int, int)[]: a genotype, which is a list of tuples (pitch, dur)
    """
    genotype = local_mutation(chromosome, d, prob_local)
    return genotype
