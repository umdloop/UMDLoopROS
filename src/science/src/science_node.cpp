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
        this->declare_parameter("speed", .4);
        this->declare_parameter("left", 0.1);
        this->declare_parameter("right", 0.0);
        leftLiftMotor = new TalonSRX(7);
        leftLiftMotor->EnableCurrentLimit(true);
        leftLiftMotor->ConfigVoltageCompSaturation(24);
        leftLiftMotor->EnableVoltageCompensation(true);
        leftLiftMotor->ConfigPeakCurrentLimit(20);
        rightLiftMotor = new TalonSRX(8);
        rightLiftMotor->EnableCurrentLimit(true);
        rightLiftMotor->ConfigPeakCurrentLimit(20);
        rightLiftMotor->ConfigVoltageCompSaturation(24);
        rightLiftMotor->EnableVoltageCompensation(true);

        subscription1_ = this->create_subscription<keyboard_msgs::msg::Key>(    // CHANGE
      "keydown", 10, std::bind(&TalonSRXController::keydown_callback, this, std::placeholders::_1));
      subscription2_ = this->create_subscription<keyboard_msgs::msg::Key>(    // CHANGE
      "keyup", 10, std::bind(&TalonSRXController::keyup_callback, this, std::placeholders::_1));
    }

    ~TalonSRXController()
    {
        delete leftLiftMotor;
        delete rightLiftMotor;
    }

private:
    void keydown_callback(const keyboard_msgs::msg::Key & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "%d", msg.code);
        if (msg.code == msg.KEY_U)
        {
            unmanaged::Unmanaged::FeedEnable(100);
            RCLCPP_INFO(this->get_logger(), "%d", msg.code);
            RCLCPP_INFO(this->get_logger(), "Current Left Speed: %f", this->get_parameter("speed").as_double()+this->get_parameter("left").as_double());
            RCLCPP_INFO(this->get_logger(), "Current Right Speed: %f", this->get_parameter("speed").as_double()+this->get_parameter("right").as_double());
            leftLiftMotor->Set(ControlMode::PercentOutput, this->get_parameter("speed").as_double()+this->get_parameter("left").as_double());
            rightLiftMotor->Set(ControlMode::PercentOutput, this->get_parameter("speed").as_double()+this->get_parameter("right").as_double());

        }
        else if (msg.code == msg.KEY_I) {
            unmanaged::Unmanaged::FeedEnable(100);
            RCLCPP_INFO(this->get_logger(), "%d", msg.code);
            RCLCPP_INFO(this->get_logger(), "Current Left Speed: %f", this->get_parameter("speed").as_double()+this->get_parameter("left").as_double()+.1);
            RCLCPP_INFO(this->get_logger(), "Current Right Speed: %f", this->get_parameter("speed").as_double()-this->get_parameter("right").as_double());
            leftLiftMotor->Set(ControlMode::PercentOutput, -1*this->get_parameter("speed").as_double()+this->get_parameter("left").as_double()-.31);
            rightLiftMotor->Set(ControlMode::PercentOutput, -1*this->get_parameter("speed").as_double()+this->get_parameter("right").as_double()-.1);

        }
        else
        {
            leftLiftMotor->Set(ControlMode::PercentOutput, 0.0);
            rightLiftMotor->Set(ControlMode::PercentOutput, 0.0);
        }
    }

    void keyup_callback(const keyboard_msgs::msg::Key & msg) const
    {
            leftLiftMotor->Set(ControlMode::PercentOutput, 0.0);
            rightLiftMotor->Set(ControlMode::PercentOutput, 0.0);
    }

    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr subscription1_;
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr subscription2_;
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
