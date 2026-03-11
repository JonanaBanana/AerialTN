#include "ircam/ircam_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions();
    options.use_intra_process_comms(false); // standalone = inter-process

    try {
        auto node = std::make_shared<ir_v4l2_camera::IrCameraNode>(options);
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}