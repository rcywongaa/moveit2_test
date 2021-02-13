import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.time import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy


from robot_msgs.msg import PointTrajectory

class PointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('PointTrajectoryPublisher')
        self.get_logger().info("PointTrajectoryPublisher created!")

        latching_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
        )

        self.publisher = self.create_publisher(
                PointTrajectory,
                self.declare_and_get_parameter("trajectory_topic").string_value,
                qos_profile=latching_qos)
        trajectory = PointTrajectory()
        filename = self.declare_and_get_parameter("filename").string_value
        with open(filename) as f:
           for cnt, line in enumerate(f):
               entries = line.split(' ')
               point = Point()
               point.x = float(entries[0])
               point.y = float(entries[1])
               point.z = float(entries[2])
               trajectory.points.append(point)
               trajectory.time_from_start.append(Duration(seconds=float(entries[3])).to_msg())
        self.publisher.publish(trajectory)
        self.get_logger().info("Trajectory published!")

    def declare_and_get_parameter(self, parameter_name, default_value=None):
        self.declare_parameter(parameter_name, default_value)
        return self.get_parameter(parameter_name).get_parameter_value()

def main(args=None):
    rclpy.init(args=args)
    publisher = PointTrajectoryPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
