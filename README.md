# isaacs_server

ROS package that acts as a server for the Immersive Semi-Autonomous Aerial Command System (ISAACS) platform.

## Setup

1. Please install ROS by following [this link](http://wiki.ros.org/melodic/Installation)
2. Install rosbridgelib by following [this link](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge). You don't need to run it yet.
3. Create a catkin workspace. [(read more about catkin workspaces here)](http://wiki.ros.org/catkin/workspaces) 
    * `mkdir catkin_ws`
    * `mkdir catkin_ws/src`
    * `cd catkin_ws/src`
    * `catkin_init_workspace` (this should create a "build" and "devel" directory in catkin_ws)
4. from within catkin_ws/src, run `git clone -b operator https://github.com/immersive-command-system/isaacs_server.git` (Or you can use the ssh uri). The "-b operator" argument ensures you clone the operator branch, which contains our most up-to-date (and probably buggy) code.
5. `cd ..`
6. `catkin_make`
7. `source devel/setup.bash` (this needs to be run every time)
   * Alternatively for this to be done automatically, you can add the following line into `~/.bashrc`: `source ~/catkin_ws/devel/setup.bash`
   * `source ~/.bashrc`

You are now all set and can run the server node.

8. To start the server, run `roslaunch rosbridge_server rosbridge_websocket.launch`. To edit the parameters, run `roslaunch rosbridge_server rosbridge_websocket.launch {arg}:={value}` (for example, `roslaunch rosbridge_server rosbridge_websocket.launch unregister_timeout:= 600`). To test locally, you should 
9. To run the operator node, run `python3 operator.py` from isaacs_server/src in a new terminal window.
10. You can now make service calls. These will eventually be made from the VR interface, but you can also test our services by running `python3 service_test/[test_name].py` from isaacs_server/src in a new terminal window.


Note that for roslibpy to work, the host IP should be the ROS master that is running rosbridge (multi-network) or roscore (local network). All ros connections are currently made to a static IP address (that probably isn't yours), so you will have to change all IP addresses to 'localhost' to run it locally, or to your static IP if you set up port forwarding. If you're confused about port forwarding, ask Kyle for more info.

Host IP can be set in either `constants.py` or with the `--ip [ip]` argument via command line.


## Getting Started

Make sure you `git pull` to ensure you have the latest code before you make any changes.




* **src/** contains all source code and python executables
    * **src/operator.py** is the main ROS node running on the server that defines all services that can be called by the VR interface node.
    * **src/drone.py** defines the drone abstract class, and subclasses for each drone that we have implemented. Currently, this is just DjiMatrice. Currently, only add_drone has been implemented (with some error checking still needed). Additional functions need to be added.
    * **src/djimatrice.py** implements the drone abstract class, using the Dji SDK. All abstract methods have been implemented from drone.py except shutdown since DJI SDK does not have a built in service call for shutdown.
    * **src/mavros.py** implements the drone abstract class, using the mavros SDK. All abstract methods have been implemented from drone.py except get_speed.
    * **src/service_test/** contains various tests for all service calls. These are used for testing and debugging our services without needing the VR interface. Tests should include both proper service calls, and service calls with bad inputs. For bad inputs, error messages should be back-propagated to the node that called the service, explaining what error occurred (ex: "Drone type Pixhawk has not been implemented yet")
srv* **    * **src/test.py** Primary driver for all tests. Tests have been written for both MAVROS and DJI_SDK drone classes and their corresponding methods. Ensure you are conncted to the AWS server in order to run the py.test
/** contains all service type definitions.
    * **To add a new service type**, create a new file named [service_name].srv (within the isaacs_server/srv/ directory). srv files are simple text files that have two parts: a request and response. The 2 parts are separated by a "---". Each part describes the fields (parameters) for the request/response. To describe a field, simply include the field type and field name, with one field per line. See srv/add_drone.srv for an example. 
    * Before we can actually use the service type we just defined, we must make sure the srv file is turned into source code. To do this, closely follow [this link](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv). You'll need to make changes to isaacs_server/CMakeLists.txt (not catkin_ws/src/CMakeLists.txt) and isaacs_server/package.xml. Following section 4.1 should be enough, but read through the whole thing at least once to understand how messages and services work.
    * Don't forget to run `catkin_make` and `source devel/setup.bash` from catkin_ws/ to compile the package with the new service types. This must be done before you can use these new service types.
    
## Using Roslipby

You'll need to `pip install roslibpy` before you can use it. Make sure you're using python3, as python2 will run into errors. You might have to pip3 install roslibpy if python2 is your default python version, and python3 file_name.py to run it with python3.

`import roslibpy` at the top of the python file.

Create a connection to ROS master:  
    `client = roslibpy.Ros(host='ip', port=9090)` #where ip is either 'localhost' or the static IP of the ROS master you are trying to connect to.  
    `client.run()`. 

Creating a service:   
`service = roslibpy.Service(client, '/add_drone', 'isaacs_server/add_drone')` #where /add_drone is the name of the service, and isaacs_server/add_drone is the service type defined by add_drone.srv in the isaacs_server package.  
`service.advertise(handler)` #where handler is a function that is run upon the service being called. 

Calling a service:  
`service = roslibpy.Service(client, '/add_drone', 'isaacs_server/add_drone')` #where /add_drone is the name of the service, and isaacs_server/add_drone is the service type defined by add_drone.srv in the isaacs_server package.  
`request = roslibpy.ServiceRequest({'ip': "some ip", "port": 9090, "drone_type":"DjiMatrice"})` #a dictionary of field names (defined in the service type) to inputs.  
`result = service.call(request)` #calls the service, with inputs defined by request, and sets result to the callback.

## Setup Port Forwarding
1. Find your router ip address.
2. Type in ip address as url of browser
3. Enter router credentials (most likely *user: admin* and *pass: admin* if it has not been setup previously)
4. Search for *Virtual Server* or *Port Forwarding* tab.  Will probably be located under advanced settings.
5. Add a port forwarding service with the local port you plan to use.


