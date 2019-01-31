The CMake file does not work out of the box, because you will need a few 
external libraries (all available on ROS) if you want to use the SPA solver for 
closing loops:

CSparse
eigen
sba


Alternatively, you can just not compile Tutorial2.
