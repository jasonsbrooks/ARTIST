import ngram,ga
from optparse import OptionParser

TWELVE_BAR_BLUES = [1,1,1,1,4,4,1,1,5,4,1,1]

def main():
    parser = OptionParser()

    parser.add_option("-n", "--ngram", dest="ngram",default="undefined")
    parser.add_option("-g", "--ga", dest="ga", type="int", default=800)

    (options, args) = parser.parse_args()

    ga.run(ngram_generate=(lambda: ngram.generate(options.ngram,TWELVE_BAR_BLUES)),num_iter=options.ga)

if __name__ == '__main__':
    main()
