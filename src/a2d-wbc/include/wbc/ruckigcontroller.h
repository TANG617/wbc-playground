#pragma once

#include <ruckig/ruckig.hpp>
#include <array>
#include <iostream>

/*
author:tangyong
date:2025.10.29
des:rukig封装，平滑处理
*/

template <size_t DOF>
class RuckigController
{
private:
    ruckig::Ruckig<DOF> m_ruckigTool;
    ruckig::InputParameter<DOF> m_inputPara;
    ruckig::OutputParameter<DOF> m_outputPara;

    void setCurrentState(const std::array<double, DOF> &currentPos,
                         const std::array<double, DOF> &currentVel,
                         const std::array<double, DOF> &currentAcc)
    {
        m_inputPara.current_position = currentPos;
        m_inputPara.current_velocity = currentVel;
        m_inputPara.current_acceleration = currentAcc;
    }

   

public:
    // control_dt，控制周期（单位：秒）
    RuckigController(
        const std::array<double, DOF> &maxVelocity,
        const std::array<double, DOF> &maxAcceleration,
        const std::array<double, DOF> &maxJerk,
        double control_dt) : m_ruckigTool(control_dt)
    {
        // 设置速度、加速度和加加速度限制
        m_inputPara.max_velocity = maxVelocity;
        m_inputPara.max_acceleration = maxAcceleration;
        m_inputPara.max_jerk = maxJerk;

        m_inputPara.current_position.fill(0.0);
        m_inputPara.current_velocity.fill(0.0);
        m_inputPara.current_acceleration.fill(0.0);

        m_inputPara.target_position = m_inputPara.current_position;
        m_inputPara.target_velocity.fill(0.0);
        m_inputPara.target_acceleration.fill(0.0);
    }

    void setTarget(const std::array<double, DOF> &targetPos)
    {
        m_inputPara.target_position = targetPos;
        m_inputPara.target_velocity.fill(0.0);
        m_inputPara.target_acceleration.fill(0.0);
    }

    void setCurrentState(const std::array<double, DOF> &currentPos)
    {
        m_inputPara.current_position = currentPos;
        m_inputPara.current_velocity.fill(0.0);
        m_inputPara.current_acceleration.fill(0.0);
    }

    // 生成下一周期轨迹
    bool update(std::array<double, DOF> &outPos, std::array<double, DOF> &outVel)
    {
       const ruckig::Result result = m_ruckigTool.update(m_inputPara, m_outputPara);

        if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
        {          
            std::cerr << "Ruckig Error,code: " << static_cast<int>(result) << std::endl;
            return false;
        }
    
        outPos = m_outputPara.new_position;
        outVel = m_outputPara.new_velocity;
   
        m_outputPara.pass_to_input(m_inputPara);
        return true;   
     }
};
