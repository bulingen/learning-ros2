#include "rclcpp/rclcpp.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
  NumberCounterNode() : Node("number_counter")
  {
  }

private:
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}