#include "rclcpp/rclcpp.hpp"
#include "dwa_planner/dwa_planner_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dwa_planner::DWAPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
