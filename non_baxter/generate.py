import ngram,ga
from optparse import OptionParser

TWELVE_BAR_BLUES = [1,1,1,1,4,4,1,1,5,4,1,1]

def main():
    parser = OptionParser()

    parser.add_option("-n", "--ngram", dest="ngram",default="undefined")
    parser.add_option("-g", "--ga", dest="ga", type="int", default=-1)

    (options, args) = parser.parse_args()

    ngram_output = []
    if options.ngram != "undefined":
        ngram_output = ngram.generate(options.ngram,TWELVE_BAR_BLUES)
        print ngram_output

    if options.ga != -1:
        ga.run(ngram_output=ngram_output,num_iter=options.ga)
    else:
        # just output the ngram midi
        ga.create_midi_file((0,ngram_output), TWELVE_BAR_BLUES)

if __name__ == '__main__':
    main()
