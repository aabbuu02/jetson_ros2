# 🚀 Zeus ROS2 Quick Test Commands

## 1. Environment Setup
```bash
cd /root/zeus_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. Package Testing (No Hardware Required)

### Test Message Packages:
```bash
ros2 interface show anscer_msgs/msg/PGVPose
ros2 interface show graph_msgs/msg/Graph
ros2 interface show graph_creator_msgs/msg/GraphGeneratorConfig
```

### Test Individual Nodes:
```bash
# Graph Creator (can run without hardware)
ros2 run graph_creator graph_creator_node

# Global Planner (can run without hardware)
ros2 run global_planner path_graph_planner_node

# Robot Pose Publisher (can run without hardware)
ros2 run robot_pose_publisher robot_pose_publisher
```

### Test Launch Files:
```bash
# Emergency Handler (safe to test)
ros2 launch emergency_handler emergency_handler.launch.py

# Tag Monitor (safe to test)
ros2 launch tag_monitor tag_monitor.launch.py
```

## 3. Topic/Service Testing

### List Available Topics:
```bash
ros2 topic list
```

### List Available Services:
```bash
ros2 service list
```

### Monitor Topics:
```bash
ros2 topic echo /graph
ros2 topic echo /cmd_vel
```

## 4. Simulation Testing

### Run Graph Creator with Visualization:
```bash
ros2 run graph_creator graph_creator_node &
ros2 run rviz2 rviz2
```

### Test Navigation Stack (without robot):
```bash
ros2 launch anscer_agv_bringup zeus_bringup.launch.py
```

## 5. Hardware-Specific Testing (Requires Robot)

### Robot Controller:
```bash
ros2 launch acr_robot_controller acr_robot_controller.launch.py
```

### Lift Action:
```bash
ros2 launch lift_action lift_action.launch.py
```

### Reader Modules:
```bash
ros2 launch reader_modules shelf_reader.launch.py
ros2 launch reader_modules pgv_reader.launch.py
```

## 6. Debugging Commands

### Check Node Status:
```bash
ros2 node list
ros2 node info /node_name
```

### Check Package Info:
```bash
ros2 pkg list | grep zeus
ros2 pkg executables package_name
```

### Monitor System Resources:
```bash
htop
ros2 topic hz /topic_name
```

## 7. Performance Testing

### CPU Usage:
```bash
ros2 run resource_monitor resource_monitor
```

### Memory Usage:
```bash
ros2 run memory_monitor memory_monitor
```

## 8. Integration Testing

### Full System Test (without hardware):
```bash
# Terminal 1: Start core services
ros2 launch anscer_agv_bringup zeus_bringup.launch.py

# Terminal 2: Start navigation
ros2 launch global_planner global_planner.launch.py

# Terminal 3: Start graph management
ros2 launch graph_creator graph_creator.launch.py
```

## 9. Troubleshooting

### Check Logs:
```bash
ros2 log level /node_name DEBUG
ros2 log info
```

### Restart Nodes:
```bash
ros2 lifecycle set /node_name configure
ros2 lifecycle set /node_name activate
```

### Kill All Nodes:
```bash
ros2 daemon stop
ros2 daemon start
```
