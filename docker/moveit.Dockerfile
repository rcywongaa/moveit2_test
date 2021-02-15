FROM rcywongaa/remy_test:base
SHELL ["bash", "-c"]
WORKDIR /moveit2_tmp
RUN source /opt/ros/foxy/setup.bash && colcon build --symlink-install --merge-install --install-base /opt/ros/foxy/

