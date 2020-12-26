#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
  NumberCounterNode() : Node("number_counter"), counter_(0)
  {
      subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
          "number",
          10,
          std::bind(&NumberCounterNode::handleNumberMessage, this, std::placeholders::_1) 
      );
      RCLCPP_INFO(this->get_logger(), "Number counter has been initialized");
  }

private:
    void handleNumberMessage(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        int received_number = msg->data;
        RCLCPP_INFO(this->get_logger(), "counter: %d, received: %d", counter_, received_number);
        counter_ ++;
    }
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    int counter_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}