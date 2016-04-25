# Installation on Ubuntu

## Install System Packages

```
sudo apt-get update
sudo apt-get install -y git python-dev python-pip postgresql postgresql-contrib python-psycopg2 libpq-dev

sudo apt-get build-dep python imaging
sudo apt-get install -y libjpeg8 libjpeg62-dev
```

## Clone Repository

```
git clone git@github.com:jasonsbrooks/ARTIST.git
cd ARTIST
```

## Install Python Packages using PIP

```
sudo -H pip install -r requirements.txt
```

## Install Python-MIDI Package

```
cd ../
git clone git@github.com:vishnubob/python-midi.git
cd python-midi
sudo -H setup.py install
cd ../
sudo -H rm -rf python-midi/
```


## Create the Databases

```
cd ARTIST/src/artist_generator/
```

Now, we need to set a password for the `postgres` user:

```
sudo -u postgres psql
```

In the `psql` prompt that appears, type `\password` and then enter the password `postgres` twice.

Now, to create a database pool of size `NUM`, run:

```
sudo -u postgres python -m db.reset create NUM
```

## Parse MIDI into Database

Just run:

```
sudo -u postgres python -m db.midi_store -t NUM
```

where `NUM` is the number of databases you created before. This is also the number of database worker processes that will be run.

## Harmonic Analysis

Run:

```
sudo -u postgres python -m analyze.chords -t NUM
```

where `NUM` is the number of databases, as before. This process will take a LONG time to complete.