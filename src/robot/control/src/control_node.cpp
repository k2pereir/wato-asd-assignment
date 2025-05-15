#include "control_node.hpp"

ControlNode::ControlNode() : Node("control") {}

void ControlNode::initControlCore()
{
  control_ = std::make_unique<robot::ControlCore>(shared_from_this(), this->get_logger());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  node->initControlCore();   
  rclcpp::spin(node); 
  rclcpp::shutdown();
  return 0;
}
