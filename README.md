# 180 Final Project | Team Erick, Celeste, Ismael

Final Project for CSE 180. Using ROS2, C++ and gazebo.

To do list:

- ~~get robot to move around environment (probably gonna make it move along a predetermined path around the environment)~~

- ~~find pillar (probably by comparing local cost map to global cost map)~~

- ~~collision prevention~~

## Running this project

### Prerequisites

Using aptitude or pip (where applicable), install the following packages:

- Ubuntu 20.04
- colcon
- ros2-foxy-*
- Gazebo
- Stefano Carpin's [MRTP](https://github.com/stefanocarpin/MRTP) package, downloaded, built and installed in a memorable place (`~`)

Note: Installing Ros2 will take a significant amount of time.

### Building

`git clone git@github.com:leamsifigs/180project.git`  

You will need to clone our project into a memorable directory. `~` is recommended.

```bash
cd ~/180project/src

colcon build

. install/local_setup.bash
```

You should be good to go!  

You will need to [source your ros enviornment variables](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html#sourcing-the-setup-script) as well!

It is recommended that you follow instrustions also linked in Prof. Carpin's page.

Note: if if you're using VScode remote connection, accept the prompt to define your cmake directory. and that will be the `cmakelists.txt` inside `src/project`. this will ensure you don't get a ton of red squigles everywhere!

### Running

Open two new terminal windows. If you haven't sourced ROS2 files and directories, do so now.

In one window run the following if you want run without the extra pillar:  
```bash
cd ~/MRTP/MRTP
. install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
ros2 launch gazeboenvs tb3_simulation.launch.py
```  

or this if you want to try it with a added 10th pillar:

```bash
cd ~/MRTP/MRTP
. install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/MRTP/MRTP/src/gazeboenvs/models
ros2 launch gazeboenvs tb3_simulation.launch.py
```

This will spawn Gazebo and a complex enviornment. This should take 1-3 minutes to open, depending on the weather. Once it is open completely, you may continue.

Finally, in your second terminal enter the following the following:
```bash
cd ~180project/src
. install/local_setup.bash
ros2 run project seeker > output.txt
```  

we recommend piping the output to a file to make it easier to parse.  