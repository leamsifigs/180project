# 180project
Final Project for CSE 180. Using ROS2, C++ and gazebo.

To do list:
    get robot to move around environment (probably gonna make it move along a predetermined path around the environment)
    find pillar (probably by comparing local cost map to global cost map)
    collision prevention
    


## To setup enviornment

`git clone` to a directory

`cd src` cd into src folder

`colcon build`  build the dang thing

`. install/local_setup.bash` source our stuff

and you should be good to go. I hope the .gitignore works correctly!

if if you're using vscode, you'll want to accept the prompt to define your cmake directory. and that will be the cmakelists.txt inside `src/project`. this will ensure you don't get a ton of red squigles everywhere!
