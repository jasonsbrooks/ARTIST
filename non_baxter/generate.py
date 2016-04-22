import ngram,ga
from optparse import OptionParser

TWELVE_BAR_BLUES = [1,1,1,1,4,4,1,1,5,4,1,1]

def main():
    parser = OptionParser()

    parser.add_option("-n", "--ngram", dest="ngram")
    parser.add_option("-g", "--ga", dest="ga", type="int", default=800)

    (options, args) = parser.parse_args()

    ngram_output = []
    if not (options.__dict__["ngram"] is None):
        ngram_output = ngram.generate(options.ngram,TWELVE_BAR_BLUES)
        print ngram_output

    if not (options.__dict__["ga"] is None):
        ga.run(ngram_output=ngram_output,num_iter=options.ga)
    else:
        # just output the ngram midi
        ga.create_midi_file((0,ngram_output), ga.create_chord_progression())

if __name__ == '__main__':
    main()