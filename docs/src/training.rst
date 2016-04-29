Training the Trigram Model
==========================

Create the Databases
--------------------

::

    cd ARTIST/src/artist_generator/

Now, we need to set a password for the ``postgres`` user:

::

    sudo -u postgres psql

In the ``psql`` prompt that appears, type ``\password`` and then enter
the password ``postgres`` twice.

Now, to create a database pool of size ``NUM``, run:

::

    sudo -u postgres python -m db.reset create NUM

Parse MIDI into Database
------------------------

Just run:

::

    sudo -u postgres python -m db.midi_store -t NUM

where ``NUM`` is the number of databases you created before. This is
also the number of database worker processes that will be run. This will
take on the order of 1 hr on an 8-core machine.

Harmonic Analysis
-----------------

Run:

::

    sudo -u postgres python -m analyze.chords -t NUM

where ``NUM`` is the number of databases, as before. This process will
take a LONG time to complete (24-36 hours on 8-core machine). The
process can be interrupted at any point without losing progress -- each
song records whether or not it has been analyzed yet.

Training the Trigram Model
--------------------------

Run:

::

    sudo -u postgres python -m ngram.train -o OUTDIR -t NUM

where ``OUTDIR`` is where you wish to save the models and ``NUM`` is the
number of databases, as before. This should complete is < 15 min.
