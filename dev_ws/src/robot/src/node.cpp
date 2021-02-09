#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <functional>
#include <memory>

using std::placeholders::_1;

using PointStamped = geometry_msgs::msg::PointStamped;

class RobotNode : public rclcpp::Node
{
    public:
        RobotNode() :
            Node("RobotNode")
        {
            std::string ee_position_topic_param = "ee_position_topic";
            this->declare_parameter(ee_position_topic_param);
            sub = create_subscription<PointStamped>(
                    get_parameter(ee_position_topic_param).as_string(),
                    1, std::bind(&RobotNode::callback, this, _1));
        }

        void callback(const PointStamped::SharedPtr msg)
        {
            ;
        }

    private:
        rclcpp::Subscription<PointStamped>::SharedPtr sub;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
}
