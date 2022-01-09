# T.I.P.S.Y.: Tiago Is Pouring Shots, yay!
## Developers: Tiphaine Caillibotte, Geraldine Dewulf, Lorenzo Montalto
## Useful ROS commands:
### Fixing stuff (like Aruco markers):
- roscd tiago_gazebo/models
- grep -RiIl 'sdf' | xargs sed -i 's/package/model/g'
### Running the whole thing (one terminal per instruction)
- roslaunch tipsy tipsy_launch.launch (rosprolog service and spawning Tiago into the world with the map)
- rosrun tipsy motion_node (launching the motion node)
- rosrun tipsy table_to_coordinates (launching the node that asks for drinks and provides coordinates)
- rostopic echo /coordinates_topic (if you want to listen to the topic)
### Opening the world
- roscore
- rosrun gazebo_ros gazebo /home/user/simulation_ws/src/tiago_simulation/tiago_gazebo/worlds/bar.world
### Manual control
- rosrun key_teleop key_teleop.py
## Useful Git commands:
- To clone the repository into your computer: git clone https://github.com/Montie97/T.I.P.S.Y..git
- To see the status of your local repository wrt the one in git: git status
- To add a new file to the repository: git add {name of the file}
- To add all the files into the git repository: git add -A
- To commit (with a commit message): git commit -m "commit message"
- To actually upload committed file to git: git push
- To retrieve files from the repository: git pull
- To change the branch you are working on: git checkout {name of the branch}
