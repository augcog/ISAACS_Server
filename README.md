# isaacs_server

ROS package that acts as a server for the Immersive Semi-Autonomous Aerial Command System (ISAACS) platform.

## Setup

1. Please install ROS by following [this link](http://wiki.ros.org/melodic/Installation)
2. Create a catkin workspace. [(read more about catkin workspaces here)](http://wiki.ros.org/catkin/workspaces) 
    * `mkdir catkin_ws`
    * `mkdir catkin_ws/src`
    * `cd catkin_ws/src`
    * `catkin_init_workspace` (this should create a "build" and "devel" directory in catkin_ws)
3. from within catkin_ws/src, run `git clone https://github.com/immersive-command-system/isaacs_server.git` (Or you can use the ssh uri)
4. `cd ..`
5. `catkin_make`
6. `source devel/setup.bash`
