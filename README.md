# pyturtlebot

This is a python library which abstracts the usage of Turtlebot2's to a simple Python API. This library is primarily used for HackTheFuture (http://hackthefuture.org).

## First Time Setup

First you will need a Turtlebot2 with ROS Hydro installed and with all the turtlebot debs installed:

    $ sudo apt-get install ros-hydro-turtlebot

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

    $ ipconfig

Then set it in the `~/.bashrc` file like this (it should be modified if it already exists at the end of the file):

    ROS_HOSTNAME=x.x.x.x

Remember to open a new terminal or source the `~/.bashrc` in the current terminal after changing the `~/.bashrc` file:

    $ source ~/.bashrc

## Running the demo

First you will need to start the turtlebot2 software, I have provided a launch file in this package for convenience:

    $ roslaunch pyturtlebot pyturtlebot.launch

This will launch all of the ROS programs required to have the turtlebot up and running. Next run the ipython notebook:

    $ ipython notebook --pylab --port=5555 --ip=* --notebook-dir=~/pyturtlebot/notebooks

## Using the demo

Point your browser at the turtlebot's ip, port `5555`:

    http://x.x.x.x:5555

This should bring you to a page where you can open existing notebooks or create new ones. After creating a new notebook or selecting one of the existing ones you should be brought to a new page where you can program the turtlebot using the ipython notebook. You can write code in each of the blocks and execute the blocks by hitting `shift+enter`.

