#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(std::make_unique<robot::ControlCore>(shared_from_this(), this->get_logger())) {}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
