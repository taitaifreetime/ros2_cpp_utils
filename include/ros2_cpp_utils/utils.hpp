#ifndef ROS_PARAM_HPP
#define ROS_PARAM_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace ros2_cpp_utils{
namespace utils{

    /**
     * @brief Get the Ros Param object
     * 
     * @tparam T 
     * @param node 
     * @param name 
     * @param default_value 
     * @param description optional
     * @param additional_constraints optional
     * @param read_only optional
     * @return T 
     */
    template <typename T>
    T getRosParam(
        rclcpp::Node* node, 
        const std::string& name, 
        const T& default_value, 
        const std::string& description="", 
        const std::string& additional_constraints="", 
        const bool read_only=false
    ){
        T param;
        auto config = rcl_interfaces::msg::ParameterDescriptor{};
        config.description = description;
        config.additional_constraints = additional_constraints;
        config.read_only = read_only;
        node->declare_parameter(name, default_value, config);
        node->get_parameter(name, param);

        return param;
    }


    /**
     * @brief Get the QoS object
     * set topic name for each qos content
     * 
     * @param node 
     * @param reliability 
     * @param history 
     * @param durability 
     * @param depth 
     * @return rclcpp::QoS 
     */
    rclcpp::QoS getQoS(
        rclcpp::Node* node, 
        const std::string& reliability="reliability", 
        const std::string& history="history", 
        const std::string& durability="durability", 
        const std::string& depth="depth"
    ){
        rmw_qos_profile_t qos_config = rmw_qos_profile_default;

        std::string reliability_config = getRosParam<std::string>(node, reliability, "reliable");
        if ("reliable" == reliability_config) qos_config.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        else if ("best_effort" == reliability_config) qos_config.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        else if ("system_default" == reliability_config) qos_config.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;

        std::string history_config = getRosParam<std::string>(node, history, "keep_last");
        if ("keep_last" == history_config) qos_config.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        if ("keep_all" == history_config) qos_config.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        else if ("system_default" == history_config) qos_config.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;

        std::string durability_config = getRosParam<std::string>(node, durability, "volatile");
        if ("volatile" == durability_config) qos_config.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE;
        else if ("transient_local" == durability_config) qos_config.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        else if ("system_default" == durability_config) qos_config.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;

        int depth_config = getRosParam<int>(node, depth, 10);

        rclcpp::QoS qos_profile(rclcpp::KeepLast(depth_config), qos_config);
    
        return qos_profile;
    }

}
}

#endif // ROS_PARAM_HPP