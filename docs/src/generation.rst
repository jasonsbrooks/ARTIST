Generating Music
================

Finally, we are ready to generate music. If you wish to directly
generate MIDI files, run:

::

    python generate.py -g GA_ITER -n NGRAM_DIR

in the ``src/artist_generator`` directory, where ``GA_ITER`` is the
number of iterations of the genetic algorithm you wish to run and
``NGRAM_DIR`` is the location of the previously generated model files.
Upon completion of the generation process, a MIDI file will be saved in
your current directory.

If your looking to generates multiple MIDI files, I would refer you to
the ``utils/gen_many.sh`` Bash script.

To dump all generated databases to files, take a look at
``utils/dump.sh``.

For more information on the ngram model, see

.. toctree::
	:maxdepth: 4

	artist_generator/ngram

For more information on the genetic algorithm, see

.. toctree::
	:maxdepth: 4

	artist_generator/ga

