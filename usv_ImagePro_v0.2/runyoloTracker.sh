source devel/setup.bash
echo "run  YOLO tracker on GPU..."
roslaunch yolo_tracker runYoloTracker.launch
sleep 1
wait
exit 0
