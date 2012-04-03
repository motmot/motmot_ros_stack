****************
motmot_ros_stack
****************

Motmot camera utilities for the Robot Operating System.

Installation
************

(These instructions are modeled after the `ROS kinect package
<http://www.ros.org/wiki/kinect>`_.) These instructions were tested on
Ubuntu Lucid (10.04) and Ubuntu Precise (12.04).

 0. `Install motmot <http://code.astraw.com/projects/motmot/download.html#id4>`_.

 1. Get the rosinstall tool::

      sudo apt-get install python-stdeb
      pypi-install rosinstall

 2. `Install ROS Electric Desktop-Full from source
    <http://www.ros.org/wiki/electric/Installation/Ubuntu/Source>`_ to
    ``~/ros``.

 3. Get the motmot_ros_stack.rosinstall file from github and save to a
    local directory::

      cd /tmp
      wget https://raw.github.com/motmot/motmot_ros_stack/master/motmot_ros_stack.rosinstall

 4. Dowload motmot_ros_stack into ``~/motmot_ros_stack``::

      cd /tmp
      rosinstall ~/motmot_ros_stack ~/ros motmot_ros_stack.rosinstall

 5. Build motmot_ros_stack::

      . ~/motmot_ros_stack/setup.sh
      rosmake motmot_ros_stack --rosdep-install
