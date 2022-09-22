#include "rclcpp/rclcpp.hpp"
#include "message_interfaces/msg/control.hpp"

class Publisher : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    Publisher(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s，发布话题为:/control_command.", name.c_str());
        // 创建发布者
        subscribe_and_publish_publisher_ = this->create_publisher<message_interfaces::msg::Control>("control_command", 10);
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Publisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        message_interfaces::msg::Control message;
        message.v_c = "256";
        message.w_c = "1";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "v_c: '%s'，w_c: '%s'", message.v_c.c_str(),message.w_c.c_str());
        // 发布消息
        subscribe_and_publish_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者指针
    rclcpp::Publisher<message_interfaces::msg::Control>::SharedPtr subscribe_and_publish_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个的节点*/
    auto node = std::make_shared<Publisher>("control_publisher");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}