# pyturtlebot

This is a python library which abstracts the usage of Turtlebot2's to a simple Python API. This library is primarily used for HackTheFuture (http://hackthefuture.org).

## First Time Setup

First you will need a Turtlebot2 with ROS Hydro installed and with all the turtlebot debs installed:

    $ sudo apt-get install ros-hydro-turtlebot ros-hydro-freenect-launch

You will also need the `ipython-notebook` debs:

    $ sudo apt-get install ipython-notebook

Once you have all of the turtlebot debs installed, you will need to build the `pyturtlebot` package. Start by cloning the source:

    $ git clone https://github.com/wjwwood/pyturtlebot
    $ cd pyturtlebot

Next build the package after `source`'ing the environment setup file for hydro:

    $ source /opt/ros/hydro/setup.bash
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

### HtF specific setup

You should get the ip address of the turtlebot and set the `ROS_HOSTNAME` in the `~/.bashrc`. You can get the ip address with `ipconfig`:

    $ ifconfig

Then set it in the `~/.bashrc` file like this (it should be modified if it already exists at the end of the file):

    ROS_HOSTNAME=x.x.x.x

Remember to open a new terminal or source the `~/.bashrc` in the current terminal after changing the `~/.bashrc` file:

    $ source ~/.bashrc

## Running the demo

First you will need to start the turtlebot2 software, I have provided a launch file in this package for convenience:

    $ source ~/pyturtlebot/build/devel/setup.bash
    $ roslaunch pyturtlebot pyturtlebot.launch

This will launch all of the ROS programs required to have the turtlebot up and running. If you get some errors about unloading nodelets, ROS may have been starting on boot. Stop it so you can run it locally by running `sudo service ros stop` before invoking the above `roslaunch` command.

Next run the ipython notebook in another terminal:

    $ source ~/pyturtlebot/build/devel/setup.bash
    $ ipython notebook --pylab=inline --port=5555 --ip=* --notebook-dir=~/pyturtlebot/notebooks

## Using the demo

Point your browser at the turtlebot's ip, port `5555`:

    http://x.x.x.x:5555

This should bring you to a page where you can open existing notebooks or create new ones. After creating a new notebook or selecting one of the existing ones you should be brought to a new page where you can program the turtlebot using the ipython notebook. You can write code in each of the blocks and execute the blocks by hitting <kbd>SHIFT</kbd>+<kbd>ENTER</kbd>.

## Example Program

```python
# -*- coding: utf-8 -*-
# <nbformat>2</nbformat>

# <codecell>

from pyturtlebot import get_robot

robot = get_robot()

# You can print messages to the screen with say
robot.say('Robot is ready!')

# <codecell>

# You can move the robot using the move(linear, angular) function
# The linear velocity is in meters per second and the angular velocity is in radians per second
# The move command stops after 1/2 second, so keep the robot moving you need to send command repeatedly
robot.move(0, 1.1)  # This call returns immediately
# But you can wait for any number of seconds before doing something else
robot.wait(0.6)
robot.move(0, -1.1)

# <codecell>

# You can also tell the robot to move a certain distance or turn to a certain angle
# move_distance takes a distance in meters and a speed
robot.say('Warning!!! The robot is about to move forward!!!')
robot.wait(5)
robot.move_distance(0.25, 0.5)  # This should take about 1/2 second to finish
# turn angle takes a number of degress (radians) and an angular speed
robot.turn_angle(radians(180), radians(180))  # This should take about a second
# Then drive back and turn to face the same way you started!
robot.move_distance(0.25, 0.5)
robot.turn_angle(radians(-180), radians(180))

# <codecell>

# You can run a function when the bumper is pressed
def on_bumper():
    robot.say('OUCH!')
    # This will stop the robot
    robot.stop()

# Now when you press the bumper you should get a message saying 'OUCH!' and the robot should stop
robot.on_bumper = on_bumper

# <codecell>

# If your robot gets out of control and you have to pick it up
# You must reset the wheel drop safety, like this:
robot.reset_movement()

# <codecell>
```
