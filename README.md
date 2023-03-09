# Lecture 7: Motion Planning with MoveIt! and ROS

- lecture7.yaml is inside the doc folder. A second order was added to this file.
- The Python code has been rewritten to fix a few bugs.


## Service Calls from Callbacks

Service calls from callback functions can result in a deadlock. This is because the callback function is waiting for the service to return, but the service is waiting for the callback function to return. This is a common problem in ROS, and there are a few ways to solve it.

One way is to use a callback group for the callback and a different callback group for the service. This is the approach we will take in the provided project (see competition_interface.py). Read about this approach  [here](https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html). The example code in the link is for C++ and Python.