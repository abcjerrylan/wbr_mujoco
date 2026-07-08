#pragma once

#include <mujoco_interface/robot_interface.hpp>
#include <mujoco_interface/types.hpp>
#include <robot_msgs/types.hpp>

namespace bridge
{

inline void ToRobotMsgs(const mujoco_interface::RobotState& src, robot_msgs::LowState& dst)
{
    dst.num_motors = src.num_motors;
    dst.time = src.time;
    for (int i = 0; i < 4; ++i)
    {
        dst.imu.quat[i] = src.imu.quat[i];
    }
    for (int i = 0; i < 3; ++i)
    {
        dst.imu.gyro[i] = src.imu.gyro[i];
        dst.imu.accel[i] = src.imu.accel[i];
    }
    for (std::uint32_t i = 0; i < src.num_motors && i < robot_msgs::kMaxMotors; ++i)
    {
        dst.motors[i].q = src.motors[i].q;
        dst.motors[i].dq = src.motors[i].dq;
        dst.motors[i].tau_est = src.motors[i].tau_est;
    }
}

inline void FromRobotMsgs(const robot_msgs::LowCmd& src, mujoco_interface::RobotCommand& dst)
{
    dst.num_motors = src.num_motors;
    for (std::uint32_t i = 0; i < src.num_motors && i < mujoco_interface::kMaxMotors; ++i)
    {
        dst.motors[i].q = src.motors[i].q;
        dst.motors[i].dq = src.motors[i].dq;
        dst.motors[i].tau = src.motors[i].tau;
        dst.motors[i].kp = src.motors[i].kp;
        dst.motors[i].kd = src.motors[i].kd;
        dst.motors[i].mode = src.motors[i].mode;
    }
}

class mj_adapter
{
public:
    explicit mj_adapter(mujoco_interface::RobotInterface& robot) : robot_(&robot) {}

    void ReadState(robot_msgs::LowState& state) const
    {
        mujoco_interface::RobotState native{};
        robot_->ReadState(native);
        ToRobotMsgs(native, state);
    }

    void WriteCommand(const robot_msgs::LowCmd& cmd)
    {
        mujoco_interface::RobotCommand native{};
        FromRobotMsgs(cmd, native);
        robot_->WriteCommand(native);
    }

    [[nodiscard]] std::uint32_t NumMotors() const { return robot_->NumMotors(); }

private:
    mujoco_interface::RobotInterface* robot_ = nullptr;
};

}  // namespace bridge
