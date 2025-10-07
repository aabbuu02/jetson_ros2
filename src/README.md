# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###
This is the repository for grid based anscer agv simulation and complete software required for the AGV

### How do I get set up? ###
clone the repository and catkin_make it inside a workspace


### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact

### For making robot up for navigation###

### launch anscer_gazebo package ###
roslaunch anscer_gazebo anscer_gazebo_qr.launch

###for launching graph_server,it has to launched with qr_map.graphml config file###
roslaunch graph_server graph_server.launch graph_file:=`rospack find graph_server`/config/qr_map.graphml

### for launching qr_mission_scheduler ###
rosrun qr_mission_scheduler qr_mission_scheduler_server

### for launching qr_navigation node ###
rosrun qr_navigation qr_navigation_node 

### for initialising robot ###
rostopic pub /robot_initialized std_msgs/Bool "data: true"

### for starting mission ###
### goal_id ,goalType,source,destination   has to be given ###
rostopic pub /mission_server/goal qr_mission_scheduler/mission_schedulerActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: '3'    
goal:
  goalType:
    goalType: 'CHARGING'   
    source: 1.0           
    destination: 14.0"     


