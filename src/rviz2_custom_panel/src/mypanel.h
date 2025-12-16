#pragma once
 
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <QVBoxLayout>
#include <QPushButton>
#include <QGridLayout>
#include <QLabel>
#include <QTimer>
 
namespace rviz2_custom_panel{
 
class Rviz2Panel: public rviz_common::Panel{
    Q_OBJECT
public:
    Rviz2Panel(QWidget* parent = nullptr);
    void CreateNode();
    void battHandler(const std_msgs::msg::Float32::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr cmd_pub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr batt_sub;
    float batt_value;
    QLabel* batt_label;
    QTimer* ros_spin_timer;
};
 
}