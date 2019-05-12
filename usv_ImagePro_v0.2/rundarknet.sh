source devel/setup.bash
echo "run darknet..."
roslaunch darknet_ros yolo_v3-tiny_xx.launch
sleep 1
wait
exit 0
