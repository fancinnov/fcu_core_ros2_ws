#include <stdio.h>
#include <string.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>

static char buf[16] = {0};
static std_msgs::msg::Int16 cmd;

int main(int argc, char **argv) {

  rclcpp::init(argc,argv);
  auto node = rclcpp::Node::make_shared("fcu_command");

  auto command = node->create_publisher<std_msgs::msg::Int16>("command", 100);

  while (rclcpp::ok()) {
    // 获取从键盘输入的数据
    RCLCPP_INFO(node->get_logger(),"请输入指令：");
    ssize_t size = read(STDIN_FILENO, buf, sizeof(buf));
    if(size>0){
      if(size!=2){
        RCLCPP_INFO(node->get_logger(),"指令错误！");
        continue;
      }
    }else{
      RCLCPP_INFO(node->get_logger(),"禁用指令");
      rclcpp::shutdown();
      return 0;
    }
    switch(buf[0]){
      case 'p':
        RCLCPP_INFO(node->get_logger(),"执行路径");
        cmd.data=0;
        command->publish(cmd);
        break;
      case 'a':
        RCLCPP_INFO(node->get_logger(),"解锁");
        cmd.data=1;
        command->publish(cmd);
        break;
      case 'd':
        RCLCPP_INFO(node->get_logger(),"锁定");
        cmd.data=2;
        command->publish(cmd);
        break;
      case 't':
        RCLCPP_INFO(node->get_logger(),"起飞");
        cmd.data=3;
        command->publish(cmd);
        break;
      case 'l':
        RCLCPP_INFO(node->get_logger(),"降落");
        cmd.data=4;
        command->publish(cmd);
        break;
      case 'r':
        RCLCPP_INFO(node->get_logger(),"运行");
        cmd.data=5;
        command->publish(cmd);
        break;
      case 's':
        RCLCPP_INFO(node->get_logger(),"停止");
        cmd.data=6;
        command->publish(cmd);
        break;
      case '1':
        RCLCPP_INFO(node->get_logger(),"位置点1");
        cmd.data=7;
        command->publish(cmd);
        break;
      case '2':
        RCLCPP_INFO(node->get_logger(),"位置点2");
        cmd.data=8;
        command->publish(cmd);
        break;
      case '3':
        RCLCPP_INFO(node->get_logger(),"位置点3");
        cmd.data=9;
        command->publish(cmd);
        break;
      case '4':
        RCLCPP_INFO(node->get_logger(),"位置点4");
        cmd.data=10;
        command->publish(cmd);
        break;
      case 'q':
        RCLCPP_INFO(node->get_logger(),"前向追踪");
        cmd.data=1011;
        command->publish(cmd);
        break;
      case 'w':
        RCLCPP_INFO(node->get_logger(),"下视追踪");
        cmd.data=1012;
        command->publish(cmd);
        break;
      case 'e':
        RCLCPP_INFO(node->get_logger(),"停止追踪");
        cmd.data=1013;
        command->publish(cmd);
        break;
      default:
        RCLCPP_INFO(node->get_logger(),"非法指令！");
        break;
    }
  }

  return 0;
}
