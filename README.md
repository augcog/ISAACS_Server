# isaacs_server

ROS package that acts as a server for the Immersive Semi-Autonomous Aerial Command System (ISAACS) platform.

## Setup

1. Please install ROS by following [this link](http://wiki.ros.org/melodic/Installation)
2. Create a catkin workspace. [(read more about catkin workspaces here)](http://wiki.ros.org/catkin/workspaces) 
    * `mkdir catkin_ws`
    * `mkdir catkin_ws/src`
    * `cd catkin_ws/src`
    * `catkin_init_workspace` (this should create a "build" and "devel" directory in catkin_ws)
3. from within catkin_ws/src, run `git clone -b operator https://github.com/immersive-command-system/isaacs_server.git` (Or you can use the ssh uri). The "-b operator" argument ensures you clone the operator branch, which contains our most up-to-date (and probably buggy) code.
4. `cd ..`
5. `catkin_make`
6. `source devel/setup.bash`

## Getting Started

* **src/** contains all source code and python executables
    * **src/operator.py** is the main ROS node running on the server that defines all services that can be called by the VR interface node.
    * **src/drone.py** defines the drone abstract class, and subclasses for each drone that we have implemented. Currently, this is just DjiMatrice. Currently, only add_drone has been implemented (with some error checking still needed). Additional functions need to be added.
    * **src/service_test/** contains various tests for all service calls. These are used for testing and debugging our services without needing the VR interface. Tests should include both proper service calls, and service calls with bad inputs. For bad inputs, error messages should be back-propagated to the node that called the service, explaining what error occurred (ex: "Drone type Pixhawk has not been implemented yet")
* **srv/** contains all service type definitions.
    * **To add a new service type**, create a new file named [service_name].srv (within the isaacs_server/srv/ directory). srv files are simple text files that have two parts: a request and response. The 2 parts are separated by a "---". Each part describes the fields (parameters) for the request/response. To describe a field, simply include the field type and field name, with one field per line. See srv/add_drone.srv for an example. 
    * Before we can actually use the service type we just defined, we must make sure the srv file is turned into source code. To do this, closely follow [this link](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv). You'll need to make changes to isaacs_server/CMakeLists.txt (not catkin_ws/src/CMakeLists.txt) and isaacs_server/package.xml. Following section 4.1 should be enough, but read through the whole thing at least once to understand how messages and services work.
    * Don't forget to run `catkin_make` and `source devel/setup.bash` from catkin_ws/ to compile the package with the new service types. This must be done before you can use these new service types.
