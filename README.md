op-space-control
========

Implementation of an operationnal space controller

In order to parse URDFs, compile the dependencies in the following manner : 

In op-space-control folder

    mkdir libraries/install
    cd libraries

    # Clone and compile KrisLibrary
    git clone git@github.com:jmainpri/KrisLibrary.git
    git checkout linux_control
    make

    # You can set the install path to any you want (e.g /usr/local)
    export OSC_DEP_INSTALL_PATH=`pwd`/install 
    export CMAKE_PREFIX_PATH=$OSC_INSTALL_PATH:$CMAKE_PREFIX_PATH
    
    # Set the following for runtime
    export LD_LIBRARY_PATH=$OSC_INSTALL_PATH/lib:$LD_LIBRARY_PATH

Then :

    hg clone https://bitbucket.org/osrf/urdfdom_headers
    mkdir urdfdom_headers/build && cd urdfdom_headers/build
    cmake -DCMAKE_INSTALL_PREFIX:PATH=$OSC_DEP_INSTALL_PATH ..
    make install
    cd ../..

    git clone https://github.com/ros/console_bridge.git
    mkdir console_bridge/build && cd console_bridge/build
    cmake -DCMAKE_INSTALL_PREFIX:PATH=$OSC_DEP_INSTALL_PATH .. 
    make install
    cd ../..

    hg clone https://bitbucket.org/osrf/urdfdom
    mkdir urdfdom/build && cd urdfdom/build
    cmake -DCMAKE_INSTALL_PREFIX:PATH=$OSC_DEP_INSTALL_PATH ..
    make install
    cd ../..

If you encounter an error about visual_array or collision_array missing in the last step, you probably have ROS Groovy installed and sourced. Try removing it from your ~/.bashrc or ~/.zshrc (you may need to log out and log back in). Personally, I had to resort to manually deleting certain headers from the ROS source tree. 

Let me know if you get hung up at this stage: jmainpri@wpi.edu. After the above steps are complete, op-space-control should automatically detect the installed packages. Simply run cmake again and recompile.

You can then build from source:

    mkdir build
    cmake ..
    make
    cd ../bin

Run program:

      ./robotTest

This will run a tutorial program that is helpful if you are also looking at the code located at:

      robotKin/test/robotTest.cpp

