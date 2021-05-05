#!/bin/bash

if [ -n "$1" ]; then
    dronesCount=$1
else
    dronesCount=1
    echo "Drones count not specified. Running with 1 drone"
fi


roscore &

# run_simulation
/home/eyal/Projects/LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh &

sleep 7s

source /home/eyal/Projects/AirSim/ros/devel/setup.bash;

# run_airsim_ros_bridge
roslaunch airsim_ros_pkgs airsim_all.launch &

for (( i=1; i<=$dronesCount; i++ ));
do
    echo "spawn $i"
    sleep 2s
    roslaunch drones_controller drone.launch droneId:=$i &
done


sleep 5s


# run_test_script
python3 /home/eyal/Projects/AirSim/ros/src/drones_controller/scripts/test_drones_paths.py &

# run_ticker
python3 /home/eyal/Projects/AirSim/ros/src/drones_controller/scripts/ticker_node.py &

# run_referee
python3 /home/eyal/Projects/AirSim/ros/src/drones_controller/scripts/patrol_referee_node.py &

# run_markers_spawner
python3 /home/eyal/Projects/AirSim/PythonClient/computer_vision/marker_spawner_node.py &




# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
        
    echo "Killing..."

    ps aux | grep -i 'airsim_node' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'drones_controller_node' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'blob_detector_node' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'pd_position_controller_simple_node' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'drone.launch' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'drones_controller' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'LandscapeMountains' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'test_drones_paths.py' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'ticker_node.py' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'patrol_referee_node.py' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i 'marker_spawner_node.py' | awk '{print $2}' | xargs kill -2

    ps aux | grep -i 'AirSim' | awk '{print $2}' | xargs kill -2
    ps aux | grep -i noetic | awk '{print $2}' | xargs kill -2
    ps aux | grep -i ros_workspaces | awk '{print $2}' | xargs kill -2

    sleep 3s

    ps aux | grep -i 'airsim_node' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'drones_controller_node' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'blob_detector_node' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'pd_position_controller_simple_node' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'drone.launch' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'drones_controller' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'LandscapeMountains' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'test_drones_paths.py' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'ticker_node.py' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'patrol_referee_node.py' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i 'marker_spawner_node.py' | awk '{print $2}' | xargs kill -9

    ps aux | grep -i 'AirSim' | awk '{print $2}' | xargs kill -9
    ps aux | grep -i noetic | awk '{print $2}' | xargs kill -9
    ps aux | grep -i ros_workspaces | awk '{print $2}' | xargs kill -9

    exit
}

any_key='a'
until [ $any_key == 'c' -o $any_key == 'q' -o $any_key == 'Q' ]; do
   read any_key   
done
