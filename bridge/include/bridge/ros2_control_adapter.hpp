#pragma once

#include "bridge/robot_io.hpp"

#include <robot_msgs/types.hpp>

namespace bridge
{

class ros2_adapter
{
public:
    explicit ros2_adapter(IRobotIO& backend) : backend_(&backend) {}

    void ReadState(robot_msgs::LowState& state) const { backend_->ReadState(state); }

    void WriteCommand(const robot_msgs::LowCmd& cmd) { backend_->WriteCommand(cmd); }

    [[nodiscard]] std::uint32_t NumMotors() const { return backend_->NumMotors(); }

    static void SetMotorTorque(robot_msgs::LowCmd& cmd, std::uint32_t index, float tau)
    {
        if (index < robot_msgs::kMaxMotors)
        {
            cmd.motors[index].mode = robot_msgs::kMotorModeTorque;
            cmd.motors[index].tau = tau;
        }
    }

    static void SetMotorPd(robot_msgs::LowCmd& cmd, std::uint32_t index, float q, float dq, float kp,
                           float kd, float tau_ff = 0.0f)
    {
        if (index < robot_msgs::kMaxMotors)
        {
            cmd.motors[index].mode = robot_msgs::kMotorModePd;
            cmd.motors[index].q = q;
            cmd.motors[index].dq = dq;
            cmd.motors[index].kp = kp;
            cmd.motors[index].kd = kd;
            cmd.motors[index].tau = tau_ff;
        }
    }

private:
    IRobotIO* backend_ = nullptr;
};

}  // namespace bridge
