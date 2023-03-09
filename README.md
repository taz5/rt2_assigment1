# RT2 README for the entire assignment including all branches

## Main Branch
The main branch features a basic execution of the ros package in which the simulation takes place via a simple VRep created in coppeliasim.
You can execute the process with the command:

                   roslaunch rt2_assignment1 vrepsimulation.launch
                   
                   
The package consists of 4 nodes namely the go_to_point, user_interface which can be viewed in the scripts folder and the position_service and state_machine which can be viewed in the src folder.


## Code Execution

After launching the application, the user interface will appear, allowing control of the robot using input 1 to start and 0 to stop.

To launch CoppeliaSim, open Terminal2 and execute the necessary ROS-side processes. Once the simulation environment is launched, the command "ROS LOADING SUCCESS" will appear in the Coppelia launch terminal. Navigate to the Vrep directory and launch the ./Coppeliasim.sh file.

When CoppeliaSim is open, import the scene file "pioneerVrepforROS.ttt" from the repository by selecting "open scene" in CoppeliaSim. Once the scene is loaded, click on the play button to begin the simulation. On the ROS side, input is required to control the robot. The scene is developed and controlled with a Lua script, linked to ROS using threaded scripts.


## Nodes
**go_to_point**:
The go_to_point node provides a service to initiate or cease movement of the robot towards a specific point in the environment. The service allows the robot to be directed towards a particular (x,y) position and angle (theta).

**user_interface**:
The user_interface node enables control of the robot using the start and stop attributes, and utilizes the service implemented in the state_machine node.

**position_service**:
The PositionServer node implements a random Position Server, providing random values for x, y, and theta within the minimum and maximum limits.

**state_machine**:
The state_machine node implements a service to start or stop the robot and calls the other two services for driving the robot.



## Action Branch

ROS Actions facilitate communication between a client and server, utilizing a specified protocol and ROS topics to send goal messages from the client to the server. Clients can cancel goals using the action client, which enables the return of the result message, final goal state, and server status.

ROS services utilize a simplified service description language, "srv," building upon the ROS message format to enable request/response communication between nodes. Service descriptions are stored in .srv files within the srv/ subdirectory of a package. Examples include command.srv, position.srv, and Randomposition.srv.

After describing the complete package architecture, the package execution process follows. To launch the package, enter the command "roslaunch rt2_assignment1 sim.launch." Once launched, Gazebo and nodes will begin operating based on user input received through the interface. The implementation of the action server enables preemption of the robot goal, meaning the robot can be stopped at the last point after action server implementation from the given package.

## Jupyter Notebook

The Jupyter Notebook is the original web application for creating and sharing computational documents. It offers a simple, streamlined, document-centric experience.

After performing the exact operation for Action branch, do the following in the second terminal

                   jupyter notebook --allow-root
                   

Using the Jupyter notebook, I will make the interface for the user more appealing.


## ROS2

To begin setting up the ROS workspace, first source the terminal by running the ROS source file. Then, proceed to set the environment using the ROS script in one terminal. For the ROS2 environment, source the ROS2 environment with the source script file and clone the developed package into it. Finally, source the ROS and ROS2 scripts together in one terminal using a script. Once sourcing is complete, clone the package in a specific workspace to proceed with the ROS1_bridge.

To clone the ROS1_bridge package, use the command "git clone https://github.com/ros2/ros1_bridge.git" and clone it inside the ROS2 workspace. Then, build the workspace using the command "colcon build --symlink-install --packages-skip ros1_bridge". The compilation of the ROS bridge is based on the mapping rules contained in the package. After this, source the ROS1_bridge terminal by running "colcon build --packages-select ros1_bridge --cmake-force-configure".

Once the package is built, launch the file using the following commands:

1. In Terminal 1, launch "roslaunch rt2_assignment1 ros_ros2_bridge.launch"
2. In Terminal 2, run "ros2 run ros1_bridge dynamic_bridge"
3. In Terminal 3, launch "ros2 launch rt2_assignment1 sim_container.py" for both ROS1 and ROS2.
