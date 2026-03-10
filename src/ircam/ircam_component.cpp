#include "ircam/ircam_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace ir_v4l2_camera {

// thin alias for the node. Forward the nodeoptions
class IrCameraComponent : public IrCameraNode {
public:
    explicit IrCameraComponent(const rclcpp::NodeOptions& options) : IrCameraNode(options) {}
};

} // namespace ir_v4l2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(ir_v4l2_camera::IrCameraComponent)