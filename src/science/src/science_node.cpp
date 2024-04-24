#include <cstdio>
#include <memory>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
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

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&TalonSRXController::joy_callback, this, std::placeholders::_1));
    }

    ~TalonSRXController()
    {
        delete talSRX;
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
        for(int i = 0; i < msg->buttons.size(); i++) {
          std::cout << msg->buttons[i] << ' ';
        } 
        if (msg->buttons[0] == 1)
        {
            talSRX->Set(ControlMode::PercentOutput, .1);
        }
        else
        {
            talSRX->Set(ControlMode::PercentOutput, 0.0);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    TalonSRX* talSRX;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalonSRXController>());
    rclcpp::shutdown();
    return 0;
}
