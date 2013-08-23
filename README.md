#To create an Eclipse Project
- catkin_make --force-cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8 . -G"Eclipse CDT4 - Unix Makefiles"

#Needed Includes Path Eclipse Project
- /usr/include/c++/4.6
- /usr/include/c++/4.6/x86_64-linux-gnu
- /usr/include/c++/4.6/backward
- /usr/lib/gcc/x86_64-linux-gnu/4.6.1/include
- /usr/lib/gcc/x86_64-linux-gnu/4.6.1/include-fixed
- /usr/include/x86_64-linux-gnu

#To compile in catkin
- cd ~/catkin_ws
- catkin_make
