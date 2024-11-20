#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <Eigen/Dense>
#include "ros2_cpp_utils/ros_eigen.hpp"

class TestNode : public rclcpp::Node {
public:
    TestNode() : Node("test_node") {
        // ROSのログでノードが開始したことを出力
        RCLCPP_INFO(this->get_logger(), "TestNode has started!");

        // geometry_msgs::msg::Point メッセージを生成
        geometry_msgs::msg::Vector3 point_msg;
        point_msg.x = 1.0;
        point_msg.y = 2.0;
        point_msg.z = 3.0;

        geometry_msgs::msg::Quaternion quaternion_msg;
        quaternion_msg.x = 1.0;
        quaternion_msg.y = 2.0;
        quaternion_msg.z = 3.0;
        quaternion_msg.w = 4.0;

        // Eigenベクトルに変換
        using namespace ros2_cpp_utils::ros_eigen;
        Eigen::Vector3f eigen_vec3 = toEigenVector3(point_msg);
        Eigen::Vector2f eigen_vec2 = toEigenVector2(point_msg);
        Eigen::Quaterniond quat = toQuaternion(quaternion_msg);

        geometry_msgs::msg::Vector3 vec = toMsg<geometry_msgs::msg::Vector3>(eigen_vec3);

        Eigen::Quaterniond q(1.0, 0.0, 0.0,0.0);
        geometry_msgs::msg::Quaternion quat_msg = toMsg<geometry_msgs::msg::Quaternion>(q);

        // 結果を出力
        RCLCPP_INFO(this->get_logger(), "Original Vector3: x = %f, y = %f, z = %f", point_msg.x, point_msg.y, point_msg.z);
        RCLCPP_INFO(this->get_logger(), "Converted to Eigen::Vector3f: x = %f, y = %f, z = %f", eigen_vec3.x(), eigen_vec3.y(), eigen_vec3.z());
        RCLCPP_INFO(this->get_logger(), "Converted to Eigen::Vector2f (x, y only): x = %f, y = %f", eigen_vec2.x(), eigen_vec2.y());
        RCLCPP_INFO(this->get_logger(), "Original Quaternion: x = %f, y = %f, z = %f, w = %f", quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w);
        RCLCPP_INFO(this->get_logger(), "Converted to Eigen::Quaterniond: x = %f, y = %f, z = %f, w = %f", quat.x(), quat.y(), quat.z(), quat.w());
        RCLCPP_INFO(this->get_logger(), "Converted back to geometry_msgs::msg::Vector3: x = %f, y = %f, z = %f", vec.x, vec.y, vec.z);
        RCLCPP_INFO(this->get_logger(), "Converted back to geometry_msgs::msg::Quaternion: x = %f, y = %f, z = %f, w = %f", quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
    }
};

int main(int argc, char** argv) {
    // ROS 2 ノードの初期化
    rclcpp::init(argc, argv);

    // ノードの作成と実行
    auto node = std::make_shared<TestNode>();
    rclcpp::spin(node);

    // ノードの終了
    rclcpp::shutdown();
    return 0;
}
