#include "mypanel.h"

void rviz2_custom_panel::Rviz2Panel::CreateNode()
{
    node = rclcpp::Node::make_shared("fcu_command");
    cmd_pub = node->create_publisher<std_msgs::msg::Int16>("command", 100);
    batt_sub = node->create_subscription<std_msgs::msg::Float32>(
    "batt_now_001", 100, std::bind(&Rviz2Panel::battHandler, this, std::placeholders::_1));

}

void rviz2_custom_panel::Rviz2Panel::battHandler(const std_msgs::msg::Float32::SharedPtr msg)
{
    float batt_value = msg->data;
    batt_label->setText(QString("battery: %1").arg(batt_value));
    // RCLCPP_INFO(node->get_logger(),"battery: %f", batt_value);
}

rviz2_custom_panel::Rviz2Panel::Rviz2Panel(QWidget *parent)
{
    // 创建主布局
    QVBoxLayout* main_layout = new QVBoxLayout;
    
    // 创建标题标签
    QLabel* label = new QLabel("FanciSwarm Control Panel");
    label->setAlignment(Qt::AlignCenter);
    
    batt_label = new QLabel("battary : 0.0");
    batt_label->setAlignment(Qt::AlignCenter);
    
    // 创建网格布局用于放置按钮
    QGridLayout* grid_layout = new QGridLayout;

    QPushButton* button_arm = new QPushButton(QString("解锁"));
    QPushButton* button_disarm = new QPushButton(QString("上锁"));
    QPushButton* button_takeoff = new QPushButton(QString("起飞"));
    QPushButton* button_land = new QPushButton(QString("降落"));

    QPushButton* button_position1 = new QPushButton(QString("位置1"));
    QPushButton* button_position2 = new QPushButton(QString("位置2"));
    QPushButton* button_position3 = new QPushButton(QString("位置3"));
    QPushButton* button_origin = new QPushButton(QString("原点"));

    QPushButton* button_circle = new QPushButton(QString("绕圆"));
    QPushButton* button_waypoints = new QPushButton(QString("多点"));
    QPushButton* button_track = new QPushButton(QString("追踪"));
    QPushButton* button_stop = new QPushButton(QString("停止"));

    grid_layout->addWidget(button_arm, 0, 1);
    grid_layout->addWidget(button_disarm, 0, 2);
    grid_layout->addWidget(button_takeoff, 0, 3);
    grid_layout->addWidget(button_land, 0, 4);
    
    grid_layout->addWidget(button_position1, 1, 1);
    grid_layout->addWidget(button_position2, 1, 2);
    grid_layout->addWidget(button_position3, 1, 3);
    grid_layout->addWidget(button_origin, 1, 4);

    grid_layout->addWidget(button_circle, 2, 1);
    grid_layout->addWidget(button_waypoints, 2, 2);
    grid_layout->addWidget(button_track, 2, 3);
    grid_layout->addWidget(button_stop, 2, 4);

    CreateNode();

    connect(button_arm, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 1;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"解锁");
    });

    connect(button_disarm, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 2;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"上锁");
    });

    connect(button_takeoff, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 3;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"起飞");
    });

    connect(button_land, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 4;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"降落");
    });

    connect(button_position1, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 7;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"位置1");
    });

    connect(button_position2, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 8;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"位置2");
    });

    connect(button_position3, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 9;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"位置3");
    });

    connect(button_origin, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 10;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"原点");
    });


    connect(button_circle, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 5;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"绕圆");
    });


    connect(button_waypoints, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 0;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"多点");
    });

    connect(button_track, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 6;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"追踪");
    });

    connect(button_stop, &QPushButton::clicked, [this]() {
        std_msgs::msg::Int16 msg;
        msg.data = 6;
        cmd_pub->publish(msg);
        RCLCPP_INFO(node->get_logger(),"停止");
    });
    
    // 添加组件到主布局
    main_layout->addWidget(label);
    main_layout->addWidget(batt_label);
    main_layout->addLayout(grid_layout);
    
    // 设置主布局
    setLayout(main_layout);

    // 创建并启动ROS spin定时器
    ros_spin_timer = new QTimer(this);
    connect(ros_spin_timer, &QTimer::timeout, [this]() {
        rclcpp::spin_some(this->node);
    });
    ros_spin_timer->start(1000); // 每100ms执行一次
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_custom_panel::Rviz2Panel, rviz_common::Panel)