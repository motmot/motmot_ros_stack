camiface_ros
============

This is a standard ROS package, written in plain C and C++ and
including the usual manifest.xml to support rosmake. It has three
programs:

* camiface_ros: publishes images from libcamiface

* gl_view: an OpenGL based image viewer that seems faster and more
  robust than the OpenCV viewers that ROS seems to advertise. Supports
  scaling and Bayer demosaicing on the video card so your CPU doesn't
  take the hit.

* raw_info: prints information about an image topic

See the mainpage.dox file for more information.
