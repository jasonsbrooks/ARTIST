import ngram,ga
from optparse import OptionParser

def main():
    parser = OptionParser()

    parser.add_option("-n", "--ngram", dest="ngram")
    parser.add_option("-g", "--ga", dest="ga", type="int", default=800)

    (options, args) = parser.parse_args()

    ngram_output = []
    if not (options.__dict__[ngram] is None):
        ngram_output = ngram.generate()

    if not (options.__dict__[ga] is None):
        ga.ga(num_iter=options.ga,ngram_output=ngram_output)

if __name__ == '__main__':
    main()