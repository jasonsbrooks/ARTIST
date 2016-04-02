'''
preliminary ngram model
'''


import re
import sys
import random
import math
from collections import defaultdict


#really generate_word
# keeps generating words until # is randomly generated, then returns the whole sentence
def generate_sentence_bi():
    sentence = "#"
    current = ''
    while current is not '#':
        if current == '': current = '#'
        current = generate_word_bi(current)
        sentence += " " + current
    return sentence


#really generate_phon
#given a word, randomly generates and returns the next word using the bigram model
def generate_word_bi(word):
    # generate a random float between 0 and 1
    # we will use this float to probabilistically select a 'bin'
    # corresponding to a word in our bigram model
    rand = random.uniform(0,1)
    # go through each possible second word
    for following in bigram[word]:
        # subtract this word's probability from rand
        rand -= bigram[word][following]
        # as soon as we 'cross over' zero we have found the word for that bin
        if rand < 0.0: return following
    return following


# keeps generating words until # is randomly generated, then returns the whole sentence
def generate_sentence_tri():
    sentence = "# #"
    # keeps track of immediately prior symbol
    current = ''
    # keeps track of immediately prior pair of symbols
    context = ("#","#")
    while current is not '#':
        if current == '': context = ('#','#')
        current = generate_word_tri(context)
        context = (context[1],current)
        sentence += " " + current
    return sentence

#given a pair of words, randomly generates and returns the next word using the trigram model
def generate_word_tri(pair):
    # generate a random float between 0 and 1
    # we will use this float to probabilistically select a 'bin'
    # corresponding to a word in our bigram model
    rand = random.uniform(0,1)
    # go through each possible second word
    for following in trigram[pair]:
        # subtract this word's probability from rand
        rand -= trigram[pair][following]
        # as soon as we 'cross over' zero we have found the word for that bin
        if rand < 0.0: return following
    return following


def main():
    #Command line setup
    N = sys.argv[2] #this will be 2 for bigram, or 3 for trigram.  Otherwise, error will be returned (I don't handle genreal case).
    #smoothFlag will be turned to True if smoothing must occur
    #smoothing must occur in the case that we have 4 command line args (including this file name)
    smoothFlag = False 
    if len(sys.argv) == 3:
        smoothFlag = False
    elif len(sys.argv) == 4:
        smoothFlag = True
    else:
        print "command line error!"


    #list of phonemes
    #used for add-1 smoothing, to make sure every combination of sonorities has
    #at least a value of 1
    #must add # placeholder to phonList
    #Note that we assume that all combinations of phonemes must be used in order to fully impement 
    #add one smoothing.  This turns out to be the same thing as simply using all the phonemes
    #present in our word_transcriptions file, but we listed the phonemes here from
    #problem set 1 to be safe.
    phonList = ['#', 'AA', 'AE', 'AH', 'AO', 'AW', 'AY',
                 'B', 'CH', 'D', 'DH', 'EH', 'ER',
                 'EY', 'F', 'G', 'HH', 'IH', 'IY', 'JH',
                 'K', 'L', 'M', 'N', 'NG', 'OW', 'OY',
                 'P', 'R', 'S', 'SH', 'T', 'TH', 'UH',
                 'UW', 'V', 'W', 'Y', 'Z', 'ZH']

    #The bigram case
    if int(N) == 2:
        # open file for training the bigram model
        language = open(sys.argv[1])

        # defaultdict takes a lambda function as an argument and uses it to set the default value for every key
        # this makes it easy to build up the dictionaries without checking for each key's existence
        counts = defaultdict(lambda: 0)
        bicounts = defaultdict(lambda: defaultdict(lambda: 0))

        # this loops through all the data and stores counts
        for line in language:
            # we have to have a way to begin and end each line
            line = line.strip()
            words = re.split(r' ', line)
            words = words[1:]  # gets rid of actual word

            #add #'s surrounding the word itself
            words.insert(0, '#')
            words.append('#')

            # we go through each position and keep track of word and word pair counts
            for i in range(len(words)-1):
                counts[words[i]] = counts[words[i]] + 1
                bicounts[words[i]][words[i+1]] = bicounts[words[i]][words[i+1]] + 1

        language.close()

        bigram = defaultdict(lambda: {})

        #this is the add-1 smoothing implementation
        #we modify bicounts and counts so that bigram initialization works below.
        if smoothFlag:
            for word1 in phonList:
                for word2 in phonList:
                    bicounts[word1][word2] += 1
                    counts[word1] += 1

        # this loops through all word pairs and computes relative frequency estimates
        for word1 in counts:
            for word2 in bicounts[word1]:
                bigram[word1][word2] = float(bicounts[word1][word2])/float(counts[word1])
                # print "P(" + word2 + " | " + word1 + ")\tis\t" + str(bigram[word1][word2])

        # print 25 random words using the bigram
        if not(smoothFlag):
            for i in range(25):
            	listSentence = list(generate_sentence_bi())#[2:-2]#parses out the '#' anchors from the output
            	print ''.join(listSentence)
        else: #or prints probabilities and perplexity (part 2)
            test_file = open(sys.argv[3])
            lineCount = 0 #number of lines, used to caluculate divisor for perplexity
            log_sum = 0 #used to calculate perplexity
            totalPhones = 0

            for line in test_file:
                lineSum = 0 #temp variable used to store the sum of this word in the line
                lineCount += 1 #incrememnt lineCount
                line = '# '+ line.strip() + ' #' #get rid of newline
                print line,

                lineList = line.split(' ')
                totalPhones += len(lineList) - 2
                # print lineList

                for index in range(len(lineList)-1):
                    # print bigram[words[index]][words[index+1]]
                    tempProb = math.log(bigram[lineList[index]][lineList[index+1]],2)
                    # print tempProb
                    lineSum += tempProb
                    log_sum += tempProb

                #now, print out probability of this word
                print '\t' + str(math.pow(2,lineSum))
            # print lineCount + totalPhones
            print "Perplexity: " + str(math.pow(2,log_sum*((-1.0)/(lineCount + totalPhones))))

            #lineCount + len(phonList)-1)

    #The trigram case
    elif int(N) == 3:
        # open file for training the trigram model
        language = open(sys.argv[1]) 

        # defaultdict takes a lambda function as an argument and uses it to set the default value for every key
        # this makes it easy to build up the dictionaries without checking for each key's existence
        bicounts = defaultdict(lambda:0)
        tricounts = defaultdict(lambda:defaultdict(lambda:0))

        # this loops through all the data and stores counts
        for line in language:
            # we have to have a way to begin and end each line
            line = line.strip()
            words = re.split(r' ',line)
            words = words[1:] #gets rid of actual word

            #add #'s surrounding the word itself
            words.insert(0,'#')
            words.insert(0,'#')
            words.append('#')

            # we go through each position and keep track of word and word pair counts
            for i in range(len(words)-2):
                # create a tuple of two words that defines the context for the trigram 
                pair = (words[i],words[i+1])
                # increment the count for this pair of words
                bicounts[pair] = bicounts[pair] + 1
                # increment the count for this pair of words followed by the next word in the sequence
                tricounts[pair][words[i+2]] = tricounts[pair][words[i+2]] + 1
                
        language.close()

        trigram = defaultdict(lambda:{})

        #this is the add-1 smoothing implementation
        #we modify bicounts and counts so that bigram initialization works below.
        #Note that this will take O(len(phonList)^3) time to run
        if smoothFlag:
            for word1 in phonList:
                for word2 in phonList:
                    for word3 in phonList:
                        pair = (word1,word2)
                        bicounts[pair] += 1
                        tricounts[pair][word3] += 1

        # this loops through all word pairs and computes relative frequency estimates
        for pair in tricounts:
            for word in tricounts[pair]:
                trigram[pair][word] = float(tricounts[pair][word])/float(bicounts[pair])
                #print "P(" + word2 + " | " + word1 + ")\tis\t" + str(bigram[word1][word2])

        # print 20 random 'sentences' using the bigram
        if not(smoothFlag):
            for i in range(25):
                listSentence = list(generate_sentence_tri())#[4:-2] #parses out the '#' anchors from the output
                print ''.join(listSentence)
        else: #or prints probabilities and perplexity (part 2)
            test_file = open(sys.argv[3])
            lineCount = 0 #number of lines, used to caluculate divisor for perplexity
            log_sum = 0 #used to calculate perplexity
            totalPhones = 0 #the total number of phonemes

            #iterate through each line in the test_file, which correpsonds to a word.
            for line in test_file:
                lineSum = 0 #temp variable used to store the sum of this word in the line
                lineCount += 1 #incrememnt lineCount
                line = '# # '+ line.strip() + ' #' #get rid of newline
                print line,

                lineList = line.split(' ')
                totalPhones += len(lineList) - 3
                # print lineList

                for index in range(len(lineList)-2):
                    pair = (lineList[index],lineList[index+1])
                    # print bigram[words[index]][words[index+1]]
                    tempProb = math.log(trigram[pair][lineList[index+2]],2)
                    # print tempProb
                    lineSum += tempProb
                    log_sum += tempProb

                #now, print out probability of this word
                print '\t' + str(math.pow(2,lineSum))
            # print lineCount + totalPhones
            print "Perplexity: " + str(math.pow(2,log_sum*((-1.0)/(lineCount + totalPhones))))

            #lineCount + len(phonList)-1)
    #An error has occured
    else:
        print('Input must be N=2 or N=3!')
