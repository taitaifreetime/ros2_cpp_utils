#ifndef ROS2_UTILS_RVIZ_HPP
#define ROS2_UTILS_RVIZ_HPP

#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ros2_cpp_utils{
namespace rviz{


    const static int RED = 1;
    const static int ORANGE = 2;
    const static int YELLOW = 3;
    const static int GREEN = 4;
    const static int BLUE_GREEN = 5;
    const static int BLUE = 6;
    const static int PURPLE = 7;
    const static int PINK = 8;
    const static int BEIGE = 9;
    const static int BLACK = 10;
    const static int WHITE = 11;

    /**
     * @brief Get the Color object
     * 
     * @param code above color list
     * @param a 
     * @return std_msgs::msg::ColorRGBA 
     */
    std_msgs::msg::ColorRGBA getColor(const int code, const double& a)
    {
        std_msgs::msg::ColorRGBA color;
        color.a = a;
        switch (code){
        case RED:
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            break;
        case ORANGE:
            color.r = 1.0;
            color.g = 0.5;
            color.b = 0.0;
            break;
        case YELLOW:
            color.r = 1.0;
            color.g = 1.0;
            color.b = 0.0;
            break;
        case GREEN:
            color.r = 0.3;
            color.g = 1.0;
            color.b = 0.3;
            break;
        case BLUE_GREEN:
            color.r = 0.0;
            color.g = 0.5;
            color.b = 0.5;
            break;
        case BLUE:
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            break;
        case PURPLE:
            color.r = 0.5;
            color.g = 0.0;
            color.b = 0.5;
            break;
        case PINK:
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.5;
            break;
        case BEIGE:
            color.r = 0.96;
            color.g = 0.96;
            color.b = 0.86;
            break;
        case BLACK:
            color.r = 0.0;
            color.g = 0.0;
            color.b = 0.0;
            break;
        case WHITE:
            color.r = 1.0;
            color.g = 1.0;
            color.b = 1.0;
            break;
        default:
            color.r = 1.0;
            color.g = 1.0;
            color.b = 1.0;
            break;
        }
        return color;
    }

    /**
     * @brief Get the Header object
     * 
     * @param stamp 
     * @param frame 
     * @return std_msgs::msg::Header 
     */
    std_msgs::msg::Header getHeader(const rclcpp::Time& stamp, const std::string& frame)
    {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = frame;
        return header;
    }
}
}

namespace visualization_msgs{
namespace msg{
    
    class MarkerTemplate : public Marker
    {
        public:
            /**
             * @brief Construct a new Marker Template object
             * 
             * @param header 
             * @param type namespace visualization_msgs::msg::Marker::
             * @param color namespace ros2_cpp_utils::rviz::
             * @param lifetime rclcpp::Duration::from_seconds()
             * @param action namespace visualization_msgs::msg::Marker::
             */
            MarkerTemplate(
                const std_msgs::msg::Header& header, 
                const int type, 
                const int color=ros2_cpp_utils::rviz::RED, 
                const rclcpp::Duration& lifetime=rclcpp::Duration::from_seconds(1.0), 
                const int action=visualization_msgs::msg::Marker::ADD
            ){
                this->header = header;
                this->lifetime = lifetime;
                this->action = action;
                this->type = type;
                this->color = ros2_cpp_utils::rviz::getColor(color, 1.0);
                this->mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
            }
    };

}
}

#endif // ROS2_UTILS_RVIZ_HPP