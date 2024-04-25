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
        this->declare_parameter("speed", .1);
        this->declare_parameter("differential", 0.0);
        leftLiftMotor = new TalonSRX(7);
        leftLiftMotor->EnableCurrentLimit(true);
        leftLiftMotor->ConfigPeakCurrentLimit(10);
        rightLiftMotor = new TalonSRX(8);
        rightLiftMotor->EnableCurrentLimit(true);
        rightLiftMotor->ConfigPeakCurrentLimit(10);

        subscription_ = this->create_subscription<keyboard_msgs::msg::Key>(    // CHANGE
      "keydown", 10, std::bind(&TalonSRXController::key_callback, this, std::placeholders::_1));
    }

    ~TalonSRXController()
    {
        delete leftLiftMotor;
        delete rightLiftMotor;
    }

private:
    void key_callback(const keyboard_msgs::msg::Key & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "%d", msg.code);
        if (msg.code == msg.KEY_A)
        {
            RCLCPP_INFO(this->get_logger(), "%d", msg.code);
            RCLCPP_INFO(this->get_logger(), "Current Left Speed: %f", this->get_parameter("speed").as_double()+this->get_parameter("differential").as_double());
            RCLCPP_INFO(this->get_logger(), "Current Right Speed: %f", this->get_parameter("speed").as_double()-this->get_parameter("differential").as_double());
            leftLiftMotor->Set(ControlMode::PercentOutput, this->get_parameter("speed").as_double()+this->get_parameter("differential").as_double());
            rightLiftMotor->Set(ControlMode::PercentOutput, this->get_parameter("speed").as_double()-this->get_parameter("differential").as_double());

        }
        else
        {
            leftLiftMotor->Set(ControlMode::PercentOutput, 0.0);
        }
    }

    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr subscription_;
    TalonSRX* leftLiftMotor;
    TalonSRX* rightLiftMotor;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalonSRXController>());
    rclcpp::shutdown();
    return 0;
}
