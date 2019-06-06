#zhi qi dong tracker cheng xu
source devel/setup.bash
echo "run  YOLO tracker on GPU..."
roslaunch yolo_tracker yolotracker.launch
sleep 1
wait
exit 0
