FROM ros:foxy
SHELL ["bash", "-c"]
COPY ./dev_ws/moveit2 /moveit2_tmp
WORKDIR /moveit2_tmp
RUN source /opt/ros/foxy/setup.bash && apt update && rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y

