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

## Setup Python Path

If you wish to run any scripts without ROS / Baxter setup (ie, the ML pipeline), you'll need to first configure the Python path appropriately, so that Python can find the `artist_generator` and `artist_performer` packages.

To do so, simply source the appropriate shell script under our `setup/` folder. For example, if you use Bash, run:

```
source setup/setup.bash
```

# Data Processing to Train the Trigram Model

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

where `NUM` is the number of databases you created before. This is also the number of database worker processes that will be run. This will take on the order of 1 hr on an 8-core machine.

## Harmonic Analysis

Run:

```
sudo -u postgres python -m analyze.chords -t NUM
```

where `NUM` is the number of databases, as before. This process will take a LONG time to complete (24-36 hours on 8-core machine). The process can be interrupted at any point without losing progress -- each song records whether or not it has been analyzed yet.

## Training the Trigram Model

Run:

```
sudo -u postgres python -m ngram.train -o OUTDIR -t NUM
```

where `OUTDIR` is where you wish to save the models and `NUM` is the number of databases, as before. This should complete is < 15 min.

# Generating Music using the Trigram / GA

Finally, we are ready to generate music. If you wish to directly generate MIDI files, run:

```
python generate.py -g GA_ITER -n NGRAM_DIR
```

in the `src/artist_generator` directory, where `GA_ITER` is the number of iterations of the genetic algorithm you wish to run and `NGRAM_DIR` is the location of the previously generated model files. Upon completion of the generation process, a MIDI file will be saved in your current directory.

If your looking to generates multiple MIDI files, I would refer you to the `utils/gen_many.sh` Bash script. 

To dump all generated databases to files, take a look at `utils/dump.sh`.

# Using Baxter

First, you'll obviously need to set up Baxter and your ROS workspace. Documentation can be found [here](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter).

After running the `./baxter.sh` script and `catkin make` in the root of your `ros_ws`, you should be good to go! We've written a series of simple launch scripts for deploying the key pieces of our projects.

## Training

To train Baxter as to the location of the Xylophone keys, run the `learner.launch` script, as:

```
roslaunch baxter_artist learner.launch
```

## Improv

Now, to combine it all together, we run the `improv.launch` script:

```
roslaunch baxter_artist improv.launch
```

This runs the generation process (assuming you've already created the model files) and then plays the generated music.

## Scripts

Any of the scripts can obviously be run directly. Refer to those for documentation and usage. Especially useful is the `joint_position_keyboard.py` for precise control of joints when training Baxter as to the position of the keys. The `set_neutral.py` script is useful if you simply want to move Baxter's arms to a front-facing neutral position.

There is also a `performer.launch` script to play a pre-generated piece of music.
