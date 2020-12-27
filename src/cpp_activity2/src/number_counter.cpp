#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


class NumberCounterNode : public rclcpp::Node
{
public:
  NumberCounterNode() : Node("number_counter"), counter_(0)
  {
      publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
      subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
          "number",
          10,
          std::bind(&NumberCounterNode::handleNumberMessage, this, std::placeholders::_1) 
      );
      server_ = this->create_service<example_interfaces::srv::SetBool>(
          "reset_counter",
          std::bind(&NumberCounterNode::handleResetCounterRequest, this, _1, _2)
      );
      RCLCPP_INFO(this->get_logger(), "Number counter has been initialized");
  }

private:
    void handleNumberMessage(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        int received_number = msg->data;
        RCLCPP_INFO(this->get_logger(), "counter: %d, received: %d", counter_, received_number);
        counter_ ++;
        auto newMessage = example_interfaces::msg::Int64();
        newMessage.data = counter_;
        publisher_->publish(newMessage);
    }

    void handleResetCounterRequest(
        const example_interfaces::srv::SetBool::Request::SharedPtr request,
        const example_interfaces::srv::SetBool::Response::SharedPtr response
    )
    {
        bool should_reset = request->data;
        if (should_reset)
        {
          counter_ = 0;
          RCLCPP_INFO(this->get_logger(), "Counter was reset");
          response->success = true;
          response->message = "Counter was reset";
        }
        else 
        {
          RCLCPP_INFO(this->get_logger(), "Pass true to reset counter");
          response->success = false;
          response->message = "Pass true to reset counter";
        }
    }

    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
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