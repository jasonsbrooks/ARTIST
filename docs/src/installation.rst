Installation on Ubuntu
======================

Install System Packages
-----------------------

::

    sudo apt-get update
    sudo apt-get install -y git python-dev python-pip postgresql postgresql-contrib python-psycopg2 libpq-dev

    sudo apt-get build-dep python imaging
    sudo apt-get install -y libjpeg8 libjpeg62-dev

Clone Repository
----------------

::

    git clone git@github.com:jasonsbrooks/ARTIST.git
    cd ARTIST

Install Python Packages using PIP
---------------------------------

::

    sudo -H pip install -r requirements.txt

Install Python-MIDI Package
---------------------------

::

    cd ../
    git clone git@github.com:vishnubob/python-midi.git
    cd python-midi
    sudo -H setup.py install
    cd ../
    sudo -H rm -rf python-midi/

Setup Python Path
-----------------

If you wish to run any scripts without ROS / Baxter setup (ie, the ML
pipeline), you'll need to first configure the Python path appropriately,
so that Python can find the ``artist_generator`` and
``artist_performer`` packages.

To do so, simply source the appropriate shell script under our
``setup/`` folder. For example, if you use Bash, run:

::

    source setup/setup.bash
