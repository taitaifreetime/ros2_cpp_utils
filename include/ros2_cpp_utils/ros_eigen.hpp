#ifndef ROS2_UTILS_ROS_EIGEN_HPP
#define ROS2_UTILS_ROS_EIGEN_HPP

#include <Eigen/Dense>

namespace ros2_cpp_utils{
namespace ros_eigen{

    /**
     * @brief ros msg to Eigen::Vector2f
     * 
     * @tparam Msg 
     * @param ros_msg 
     * @return std::enable_if<
     * std::is_same<decltype(Msg::x), double>::value &&
     * std::is_same<decltype(Msg::y), double>::value,
     * Eigen::Vector2f>::type 
     */
    template <typename Msg>
    typename std::enable_if<
        std::is_same<decltype(Msg::x), double>::value &&
        std::is_same<decltype(Msg::y), double>::value,
        Eigen::Vector2f>::type
    toEigenVector2(const Msg& ros_msg) {return Eigen::Vector2f(ros_msg.x, ros_msg.y);}
        
    /**
     * @brief ros msg to Eigen::Vector3f
     * 
     * @tparam Msg 
     * @param ros_msg 
     * @return std::enable_if<
     * std::is_same<decltype(Msg::x), double>::value &&
     * std::is_same<decltype(Msg::y), double>::value &&
     * std::is_same<decltype(Msg::z), double>::value,
     * Eigen::Vector3f>::type 
     */
    template <typename Msg>
    typename std::enable_if<
        std::is_same<decltype(Msg::x), double>::value &&
        std::is_same<decltype(Msg::y), double>::value &&
        std::is_same<decltype(Msg::z), double>::value,
        Eigen::Vector3f>::type
    toEigenVector3(const Msg& ros_msg) {return Eigen::Vector3f(ros_msg.x, ros_msg.y, ros_msg.z);}


    /**
     * @brief ros msg to Eigen::Quaterniond
     * 
     * @tparam Msg 
     * @param ros_msg 
     * @return std::enable_if<
     * std::is_same<decltype(Msg::x), double>::value &&
     * std::is_same<decltype(Msg::y), double>::value &&
     * std::is_same<decltype(Msg::z), double>::value &&
     * std::is_same<decltype(Msg::w), double>::value,
     * Eigen::Quaterniond>::type 
     */
    template <typename Msg>
    typename std::enable_if<
        std::is_same<decltype(Msg::x), double>::value &&
        std::is_same<decltype(Msg::y), double>::value &&
        std::is_same<decltype(Msg::z), double>::value &&
        std::is_same<decltype(Msg::w), double>::value,
        Eigen::Quaterniond>::type
    toQuaternion(const Msg& ros_msg) {return Eigen::Quaterniond(ros_msg.w, ros_msg.x, ros_msg.y, ros_msg.z);}


    /**
     * @brief Eigen::Vector3f to ros msg
     * 
     * @tparam Msg 
     * @param vec3 
     * @return std::enable_if<
     * std::is_same<decltype(Msg::x), double>::value &&
     * std::is_same<decltype(Msg::y), double>::value &&
     * std::is_same<decltype(Msg::z), double>::value,
     * Msg>::type 
     */
    template <typename Msg>
    typename std::enable_if<
        std::is_same<decltype(Msg::x), double>::value &&
        std::is_same<decltype(Msg::y), double>::value &&
        std::is_same<decltype(Msg::z), double>::value,
        Msg>::type
    toMsg(const Eigen::Vector3f& vec3)
    {
        Msg msg;
        msg.x = vec3[0];
        msg.y = vec3[1];
        msg.z = vec3[2];
        return msg;
    }


    /**
     * @brief Eigen::Quaterniond to ros quaternion msg
     * 
     * @tparam Msg 
     * @param quat 
     * @return std::enable_if<
     * std::is_same<decltype(Msg::x), double>::value &&
     * std::is_same<decltype(Msg::y), double>::value &&
     * std::is_same<decltype(Msg::z), double>::value &&
     * std::is_same<decltype(Msg::w), double>::value,
     * Msg>::type 
     */
    template <typename Msg>
    typename std::enable_if<
        std::is_same<decltype(Msg::x), double>::value &&
        std::is_same<decltype(Msg::y), double>::value &&
        std::is_same<decltype(Msg::z), double>::value &&
        std::is_same<decltype(Msg::w), double>::value,
        Msg>::type
    toMsg(const Eigen::Quaterniond& quat)
    {
        Msg msg;
        msg.x = quat.w();
        msg.y = quat.x();
        msg.z = quat.y();
        msg.w = quat.z();
        return msg;
    }

}
}

#endif // ROS2_UTILS_ROS_EIGEN_HPP