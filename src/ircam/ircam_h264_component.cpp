#include "ircam/ircam_h264_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace ir_v4l2_camera {

// thin alias for the node. Forward the nodeoptions
class IrCameraH264Component : public IrcamH264Republisher {
public:
    explicit IrCameraH264Component(const rclcpp::NodeOptions& options) : IrcamH264Republisher(options) {}
};

} // namespace ir_v4l2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(ir_v4l2_camera::IrCameraH264Component)