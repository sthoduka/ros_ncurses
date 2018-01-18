An ncurses-based program to show output of some ROS tools.

Dependencies
------------
ncurses

ROS

Compile
-------
    mkdir build
    cd build
    cmake ..
    make

Run
---
   ./ros_ncurses

Usage
-----
    n     runs rosnode list continuously
    t     runs rostopic list continously
    s     runs rosservice list continously
    i     ros<X> info (where X = node, topic or service depending on if you're in rosnode, rostopic or rosservice list mode)
    j     scroll down, for example to select a certain node, topic or service
    k     scroll up
    q     quit
    /     start search
    Esc   remove search filter
    Enter persist search filter

Future
------
* `rostopic echo`
* select and run `rosnode info` on nodes listed in rostopic info
* select and run `rostopic info` on topics listed in rosnode info


Motivation
----------
When things don't work, I find myself using commands like `rostopic list`, `rostopic list  | grep some_string`, `rosnode info`, `rostopic echo`, etc. very often for debugging purposes. This program is supposed to make it easier to do that with a lot less typing.
