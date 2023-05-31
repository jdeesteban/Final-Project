
When building the package for the first time you may found this error:
CMake Error at /home/panda/catkin_ws/src/robot_localization/CMakeLists.txt:38 (find_package):
  By not providing "FindGeographicLib.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "GeographicLib", but CMake did not find one.

To solve it you must install the GeographicLib library by doing the 3 following commands:
 $sudo apr-get install ros-noetic-geographic-*
 $sudo apr-get install geographiclib-*
 $sudo apr-get install libgeographic-*

you can get more information (if you understand chinese) in the following link:
https://blog.csdn.net/weixin_44583856/article/details/122716587

Also is needed have the serial package:
 $sudo apr-get install ros-noetic-serial-*
 
 In the serial_reader node you must indicate the proper serial port to have a proper performace of the node.
