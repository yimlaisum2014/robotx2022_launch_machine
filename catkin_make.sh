#! /bin/bash
#Catkin ignore

# Install Arm packages
echo "Installing ROS packages for the Interbotix Arm..."
rm ./ROS/catkin_ws/src/interbotix_ros_core/interbotix_ros_xseries/CATKIN_IGNORE
rm ./ROS/catkin_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/CATKIN_IGNORE
rm ./ROS/catkin_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/CATKIN_IGNORE
rm ./ROS/catkin_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/CATKIN_IGNORE

echo "Setup udev..."
cp ./ROS/catkin_ws/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d/
udevadm control --reload-rules &&  udevadm trigger

# cd ~/GMU_NCTU_DualArm_Medical
# rosdep install --from-paths src --ignore-src -r -y
# --------------------------------

# python3 openCV 
# catkin_make --pkg vision_opencv -C ./ROS/catkin_ws \
#    -DCMAKE_BUILD_TYPE=Release \
#    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
#    -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
#    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

# python3 tf
catkin_make --pkg geometry2 -C ./ROS/catkin_ws \
	    --cmake-args \
           -DCMAKE_BUILD_TYPE=Release \
           -DPYTHON_EXECUTABLE=/usr/bin/python3 \
           -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
           -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

catkin_make -C ./ROS/catkin_ws            