#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class IrcamRepublisher : public rclcpp::Node {
public:
  IrcamRepublisher() : Node("ircam_republisher")
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/ircam_test_topic",
        rclcpp::SensorDataQoS(),
        std::bind(&IrcamRepublisher::callback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/ircam/compressed",
        rclcpp::SensorDataQoS());
  }

private:
  void callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    pub_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IrcamRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
