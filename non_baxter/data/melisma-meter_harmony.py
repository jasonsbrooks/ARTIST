#!/usr/bin/env python
"""
Run the Melisma harmonic analyzer on a series of midi files 

Effectively runs:
$ mftext/mftext MIDIFILE | meter/meter | harmony/harmony > OUTFILE

Usage:
./melisma-meter_harmony.py MIDI_DIRECTORY PROCESS_POOL_SIZE
"""

import sys,os,fnmatch
from multiprocessing import Pool
from subprocess import Popen,PIPE

MELISMA_ROOT = "../lib/melisma2003/"

def cat_err(filename):
	with open(filename) as f:

		# print out the last five lines
		for line in f.readlines()[-5:]:
			sys.stderr.write(line)
		sys.stderr.write("\n")

def run(filename):
	fp,ext = os.path.splitext(filename)
	kfilepath = fp + '.k'
	kfileerr = fp + '.err'

	kfile = open(kfilepath,'w')
	kerr = open(kfileerr,'w')

	mftext = Popen([MELISMA_ROOT + 'mftext/mftext', filename],stdout=PIPE,stderr=kerr)
	meter = Popen([MELISMA_ROOT + 'meter/meter'],stdin=mftext.stdout,stdout=PIPE,stderr=kerr)
	harmony = Popen([MELISMA_ROOT + 'harmony/harmony'],stdin=meter.stdout,stdout=kfile,stderr=kerr)
	harmony.wait()

	# a VERY rough measure of success / error
	if kfile.tell() > 0:
		sys.stdout.write("success " + kfilepath + "\n")
	else:
		sys.stderr.write("error " + kfilepath + "\n")
		cat_err(kfileerr)
	
	kfile.close()
	kerr.close()

def main():
	if len(sys.argv) >= 3:
		midiDirectory = sys.argv[1]
		poolSize = int(sys.argv[2])
	else:
		print __doc__
		sys.exit(1)

	midi_files = []

	# all midi files
	for (root, dirnames, filenames) in os.walk(midiDirectory):
		for filename in fnmatch.filter(filenames,'*.mid'):
			midi_files.append(os.path.abspath(os.path.join(root, filename)))

	# create and run the process pool
	p = Pool(poolSize)
	p.map(run,midi_files)


if __name__ == "__main__":
	main()
	pass
