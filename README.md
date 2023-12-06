# Turtlebot3-Aruco-Autodocking
A simple turtlebot 3 Aruco Autodocking Algorithm

I just have created the file named Simple_Docking.py. The environmentis just a fork for given link:
https://github.com/mahimaarora2208/Autonomous-ArUco-Marker-Detection

to see this program work

    cd ~/Your_catkin_environment_name/src
    git clone https://github.com/zeidk/rwa3_group.git
    cd ~/Your_catkin_environment_name_ws
    rosdep install --from-paths ./src --ignore-packages-from-source -y
    catkin build
    source ~/Your_catkin_environment_name_ws/devel/setup.bash
    Try roscd rwa3_group to make sure the package can be found.

The Gazebo world used in this package uses some custom models (ArUco markers) that can be found in the models folder. Tell Gazebo where to find these models by adding the following in your .bashrc (adapt this path to your case):

    export GAZEBO_MODEL_PATH=/home/zeid/enpm809e_ws/src/rwa3_kootbally/models:$GAZEBO_MODEL_PATH

We need the Waffle model for this assignment. Make sure you have the following line in .bashrc:

    export TURTLEBOT3_MODEL=waffle

Start the environment to make sure the ArUco markers appear in the environment.

    roslaunch rwa3_group rwa3.launch
After these steps its expected that you may see 3 Arcuo marker and your turtlebot. Now you just need to rotate the Aruco Marker Towards your robot. For some reason its turned away in the begining.

for docking algorithm write

rosrun rwa3_group Simple_Docking.py 

