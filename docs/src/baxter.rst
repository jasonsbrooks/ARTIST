Using Baxter
============

First, you'll obviously need to set up Baxter and your ROS workspace.
Documentation can be found
`here <http://sdk.rethinkrobotics.com/wiki/Hello_Baxter>`__.

After running the ``./baxter.sh`` script and ``catkin make`` in the root
of your ``ros_ws``, you should be good to go! We've written a series of
simple launch scripts for deploying the key pieces of our projects.

Training
--------

To train Baxter as to the location of the Xylophone keys, run the
``learner.launch`` script, as:

::

    roslaunch baxter_artist learner.launch

Improv
------

Now, to combine it all together, we run the ``improv.launch`` script:

::

    roslaunch baxter_artist improv.launch

This runs the generation process (assuming you've already created the
model files) and then plays the generated music.

Scripts
-------

Any of the scripts can obviously be run directly. Refer to those for
documentation and usage. Especially useful is the
``joint_position_keyboard.py`` for precise control of joints when
training Baxter as to the position of the keys. The ``set_neutral.py``
script is useful if you simply want to move Baxter's arms to a
front-facing neutral position.

There is also a ``performer.launch`` script to play a pre-generated
piece of music.
