FROM rcywongaa/remy_test:moveit
SHELL ["bash", "-c"]
COPY dev_ws/robot_controller /dev_ws/robot_controller/
COPY dev_ws/robot_msgs /dev_ws/robot_msgs/
COPY dev_ws/robot_planner /dev_ws/robot_planner/
COPY dev_ws/trajectory_file_reader /dev_ws/trajectory_file_reader/
WORKDIR /dev_ws
RUN source /opt/ros/foxy/setup.bash && colcon build

