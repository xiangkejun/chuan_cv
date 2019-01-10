source devel/setup.bash
echo "run gazebo..."
roslaunch vmrc_gazebo vmrc.launch 
roslaunch wamv_gazebo localization_example.launch
sleep 1
wait
exit 0


