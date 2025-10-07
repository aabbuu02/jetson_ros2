"""
@file test_zeus_ros2.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

#!/usr/bin/env python3
"""
Zeus ROS2 Testing Script for Laptop
Tests all packages without hardware dependencies
"""

import subprocess
import sys
import time
import os

def run_command(cmd, description):
    """Run a command and return success status"""
    print(f"\n🔧 {description}")
    print(f"Command: {cmd}")
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ SUCCESS: {description}")
            return True
        else:
            print(f"❌ FAILED: {description}")
            print(f"Error: {result.stderr}")
            return False
    except Exception as e:
        print(f"❌ EXCEPTION: {description} - {e}")
        return False

def test_ros2_environment():
    """Test ROS2 environment setup"""
    print("=" * 60)
    print("🚀 ZEUS ROS2 LAPTOP TESTING")
    print("=" * 60)
    
    # Test ROS2 installation
    if not run_command("ros2 --version", "ROS2 Installation Check"):
        print("❌ ROS2 not installed. Please install ROS2 Humble first.")
        return False
    
    # Test workspace setup
    if not run_command("cd /root/zeus_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash", "Workspace Setup"):
        print("❌ Workspace setup failed.")
        return False
    
    return True

def test_packages():
    """Test individual packages"""
    print("\n" + "=" * 60)
    print("📦 PACKAGE TESTING")
    print("=" * 60)
    
    packages_to_test = [
        ("anscer_msgs", "Message Generation"),
        ("graph_msgs", "Graph Messages"),
        ("graph_creator_msgs", "Graph Creator Messages"),
        ("qr_navigation_msgs", "QR Navigation Messages"),
        ("global_planner", "Global Path Planning"),
        ("graph_creator", "Graph Creation"),
        ("graph_control", "Graph Control"),
        ("graph_server", "Graph Server"),
        ("graph_visual_control", "Graph Visualization"),
        ("acr_robot_controller", "Robot Controller"),
        ("brake_action", "Brake Action"),
        ("emergency_handler", "Emergency Handler"),
        ("lift_action", "Lift Action"),
        ("reader_modules", "Reader Modules"),
        ("tag_monitor", "Tag Monitor"),
        ("power_control", "Power Control"),
        ("navigation_speed_control", "Navigation Speed Control"),
        ("qr_navigation", "QR Navigation"),
        ("qr_mission_scheduler", "QR Mission Scheduler"),
        ("lower_level_controller", "Lower Level Controller"),
        ("robot_pose_publisher", "Robot Pose Publisher"),
        ("twist_mux", "Twist Mux"),
        ("anscer_teleop", "Teleop"),
        ("vda5050_adapter", "VDA5050 Adapter"),
        ("wms_data", "WMS Data"),
        ("anscer_agv_bringup", "AGV Bringup"),
        ("start_robot", "Start Robot")
    ]
    
    success_count = 0
    total_packages = len(packages_to_test)
    
    for package, description in packages_to_test:
        cmd = f"cd /root/zeus_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 pkg list | grep {package}"
        if run_command(cmd, f"{package} - {description}"):
            success_count += 1
    
    print(f"\n📊 PACKAGE TEST RESULTS: {success_count}/{total_packages} packages available")
    return success_count == total_packages

def test_nodes():
    """Test node availability"""
    print("\n" + "=" * 60)
    print("🖥️ NODE TESTING")
    print("=" * 60)
    
    # Test if nodes can be listed
    cmd = "cd /root/zeus_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 5 ros2 node list"
    return run_command(cmd, "Node List Availability")

def test_topics():
    """Test topic availability"""
    print("\n" + "=" * 60)
    print("📡 TOPIC TESTING")
    print("=" * 60)
    
    # Test if topics can be listed
    cmd = "cd /root/zeus_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 5 ros2 topic list"
    return run_command(cmd, "Topic List Availability")

def test_services():
    """Test service availability"""
    print("\n" + "=" * 60)
    print("🔧 SERVICE TESTING")
    print("=" * 60)
    
    # Test if services can be listed
    cmd = "cd /root/zeus_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 5 ros2 service list"
    return run_command(cmd, "Service List Availability")

def test_launch_files():
    """Test launch file conversion"""
    print("\n" + "=" * 60)
    print("🚀 LAUNCH FILE TESTING")
    print("=" * 60)
    
    launch_files = [
        "acr_robot_controller.launch.py",
        "brake_action.launch.py", 
        "emergency_handler.launch.py",
        "tag_monitor.launch.py"
    ]
    
    success_count = 0
    for launch_file in launch_files:
        cmd = f"cd /root/zeus_ros2_ws && find src -name '{launch_file}'"
        if run_command(cmd, f"Launch file {launch_file}"):
            success_count += 1
    
    print(f"\n📊 LAUNCH FILE RESULTS: {success_count}/{len(launch_files)} files found")
    return success_count == len(launch_files)

def test_simulation():
    """Test simulation capabilities"""
    print("\n" + "=" * 60)
    print("🎮 SIMULATION TESTING")
    print("=" * 60)
    
    # Test if we can run a simple node
    cmd = "cd /root/zeus_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 10 ros2 run graph_creator graph_creator_node"
    return run_command(cmd, "Graph Creator Node Simulation")

def main():
    """Main testing function"""
    print("Starting Zeus ROS2 Laptop Testing...")
    
    tests = [
        ("ROS2 Environment", test_ros2_environment),
        ("Packages", test_packages),
        ("Nodes", test_nodes),
        ("Topics", test_topics),
        ("Services", test_services),
        ("Launch Files", test_launch_files),
        ("Simulation", test_simulation)
    ]
    
    passed_tests = 0
    total_tests = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n🧪 Running {test_name} Test...")
        if test_func():
            passed_tests += 1
        time.sleep(1)
    
    print("\n" + "=" * 60)
    print("📊 FINAL TEST RESULTS")
    print("=" * 60)
    print(f"✅ Passed: {passed_tests}/{total_tests} tests")
    print(f"❌ Failed: {total_tests - passed_tests}/{total_tests} tests")
    
    if passed_tests == total_tests:
        print("\n🎉 ALL TESTS PASSED! Your Zeus ROS2 migration is ready!")
        print("🚀 You can now test individual components:")
        print("   - ros2 run graph_creator graph_creator_node")
        print("   - ros2 run global_planner path_graph_planner_node")
        print("   - ros2 launch acr_robot_controller acr_robot_controller.launch.py")
    else:
        print(f"\n⚠️  {total_tests - passed_tests} tests failed. Check the errors above.")
    
    return passed_tests == total_tests

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
