#include <cstdio>
#include <memory>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "keyboard_msgs/msg/key.hpp"
#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"


class TalonSRXController : public rclcpp::Node
{
public:
    TalonSRXController()
        : Node("talon_srx_controller")
    {
        talSRX = new TalonSRX(0);
        talSRX->EnableCurrentLimit(true);
        talSRX->ConfigPeakCurrentLimit(10);

        subscription_ = this->create_subscription<keyboard_msgs::msg::Key>(    // CHANGE
      "keydown", 10, std::bind(&TalonSRXController::key_callback, this, std::placeholders::_1));
    }

    ~TalonSRXController()
    {
        delete talSRX;
    }

private:
    void key_callback(const keyboard_msgs::msg::Key & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "%d", msg.code);
        if (msg.code == 97)
        {
            RCLCPP_INFO(this->get_logger(), "%d", msg.code);
            talSRX->Set(ControlMode::PercentOutput, .1);
        }
        else
        {
            talSRX->Set(ControlMode::PercentOutput, 0.0);
        }
    }

    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr subscription_;
    TalonSRX* talSRX;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalonSRXController>());
    rclcpp::shutdown();
    return 0;
}
