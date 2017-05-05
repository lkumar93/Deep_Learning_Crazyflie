#pragma once

#include <ros/ros.h>

class PID
{
public:
    PID(
        float kp,
        float kd,
        float ki,
        float minOutput,
        float maxOutput,
        float integratorMin,
        float integratorMax,
        const std::string& name)
        : m_kp(kp)
        , m_kd(kd)
        , m_ki(ki)
        , m_minOutput(minOutput)
        , m_maxOutput(maxOutput)
        , m_integratorMin(integratorMin)
        , m_integratorMax(integratorMax)
        , m_integral(0)
        , m_previousError(0)
        , m_previousTime(ros::Time::now())
	, m_dError(0.0)
    {
    }

    void reset()
    {   m_integral = 0;
        m_previousError = 0;
	m_dError = 0;
        m_previousTime = ros::Time::now();
    }

    float velocity()
    {
	return m_dError;
    }

    void setIntegral(float integral)
    {
        m_integral = integral;
    }

    void setKp(float kp_val)
    {
        m_kp = kp_val;
    }

    void setKd(float kd_val)
    {
        m_kd = kd_val;
    }

    void setKi(float ki_val)
    {
        m_ki = ki_val;
    }

    float ki() const
    {
        return m_ki;
    }

    float kp() const
    {
        return m_kp;
    }

    float kd() const
    {
        return m_kd;
    }

    float update(float value, float targetValue)
    {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        float error = targetValue - value;
        m_integral += error * dt;
        m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
        float p = m_kp * error;
        float d = 0;
        if (dt > 0)
        {
	    m_dError = (error - m_previousError) / dt ;
            d = m_kd * m_dError;
        }
        float i = m_ki * m_integral;
        float output = p + d + i;
        m_previousError = error;
        m_previousTime = time;
        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_integral;
    float m_previousError;
    float m_dError;
    ros::Time m_previousTime;
};
