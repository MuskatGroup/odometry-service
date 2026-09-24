set -e
source /opt/ros/humble/setup.bash
cp -r /repo /work && cd /work/ros2_ws
colcon build 2>&1 | tail -2
source install/setup.bash
export PYTHONPATH=/work/src:$PYTHONPATH
python3 /smoke/gen_bag.py
ros2 launch odometry_node estimator.launch.py config:=$PWD/src/odometry_node/config/example.json use_sim_time:=true > /tmp/node.log 2>&1 &
python3 /smoke/listener.py > /tmp/listener.log 2>&1 &
sleep 4
ros2 bag play /tmp/bag --clock 2>&1 | tail -2
sleep 2
python3 /smoke/analyze.py
echo "--- node log tail ---"; tail -5 /tmp/node.log; echo "--- listener log ---"; tail -3 /tmp/listener.log
