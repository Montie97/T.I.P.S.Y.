# T.I.P.S.Y.: Tiago Is Pouring Shots, yay!
## Developers: Tiphaine Caillibotte, Geraldine Dewulf, Lorenzo Montalto
## Useful ROS commands:
### Fixing stuff (like Aruco markers):
- roscd tiago_gazebo/models
- grep -RiIl 'sdf' | xargs sed -i 's/package/model/g'
### Spawning Tiago in the world
- roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true world:=project gzpose:=-"x -0.5 -y -1.5 -z 0.0 -R 0.0 -P 0.0 -Y 1.5"
### Opening the world
- roscore
- rosrun gazebo_ros gazebo /home/user/simulation_ws/src/tiago_simulation/tiago_gazebo/worlds/project.world
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
