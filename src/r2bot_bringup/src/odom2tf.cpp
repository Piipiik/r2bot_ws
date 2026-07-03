#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomTopic2TF : public rclcpp::Node {
public:
    OdomTopic2TF(std::string name) : Node(name) {
        // 创建 odom 话题订阅者，使用传感器数据的 QoS (SensorDataQoS)
        odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            std::bind(&OdomTopic2TF::odom_callback_, this, std::placeholders::_1));
        
        // 创建一个 tf2_ros::TransformBroadcaster 用于广播坐标变换
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 回调函数，处理接收到的 odom 消息，并发布 tf
    void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;
        
        // 使用消息的时间戳和框架 ID
        transform.header = msg->header;
        transform.child_frame_id = msg->child_frame_id;

        // 设置平移 (Translation)
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        // 设置旋转 (Rotation/Orientation)
        transform.transform.rotation.x = msg->pose.pose.orientation.x;
        transform.transform.rotation.y = msg->pose.pose.orientation.y;
        transform.transform.rotation.z = msg->pose.pose.orientation.z;
        transform.transform.rotation.w = msg->pose.pose.orientation.w;

        // 广播坐标变换信息
        tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTopic2TF>("odom_topic_2_tf");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}