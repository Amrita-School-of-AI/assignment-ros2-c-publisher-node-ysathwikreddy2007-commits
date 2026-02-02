#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode()
        : Node("publisher_node"), count_(0)
    {
        // Create the publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/counter", 10);

        // Create the timer (500 ms)
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&PublisherNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        std_msgs::msg::String msg;
        msg.data = "Count: " + std::to_string(count_);

        RCLCPP_INFO(this->get_logger(),
                    "Publishing: '%s'",
                    msg.data.c_str());

        publisher_->publish(msg);
        count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}

