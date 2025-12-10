
#pragma once

#include <algorithm>
#include <array>
#include <iostream>

namespace hemisphere
{
/**
 * @brief A class that implements a PID controller.
 */
    class PID
    {
    public:
        /**
         * @brief Default constructor, initializes all member variables to zero.
         */
        PID() : _kp(0), _ki(0), _kd(0), _max(0), _min(0), _prev_error(0), _integral(0), _dt(0) {}

        /**
         * @brief Constructor with parameters.
         * @param kp Proportional gain.
         * @param ki Integral gain.
         * @param kd Derivative gain
         * @param dt Time step.
         * @param max Maximum output value.
         * @param min Minimum output value.
         */
        PID(float kp, float ki, float kd, float dt, float max, float min)
                : _kp(kp), _ki(ki), _kd(kd), _max(max), _min(min), _prev_error(0), _integral(0), _dt(dt)
        {
        }

        /**
         * @brief Computes the PID control output.
         * @param error The current error value.
         * @return The PID control output.
         */
        float compute(float error, float dt)
        {
            _dt = dt;
            if (_dt == 0.0) {
                std::cerr << "[PID] dt not set, returning 0." << std::endl;
                return 0.0;
            }
            _integral += error * _dt;
            float derivative = (error - _prev_error) / _dt;
            float output = _kp * error + _ki * _integral + _kd * derivative;
            _prev_error = error;
            return std::clamp(output, _min, _max);
        }

        /**
         * @brief Resets the PID controller's internal state.
         */
        void reset()
        {
            _prev_error = 0;
            _integral = 0;
        }

        /**
         * @brief Sets the PID controller's tunings.
         * @param kp Proportional gain.
         * @param ki Integral gain.
         * @param kd Derivative gain.
         */
        void setTunings(float kp, float ki, float kd)
        {
            _kp = kp;
            _ki = ki;
            _kd = kd;
        }

        /**
         * @brief Gets the PID controller's tunings.
         * @return kp Proportional gain, ki Integral gain, kd Derivative gain.
         */
        std::array<float, 3> getTunings() { return std::array<float, 3>{_kp, _ki, _kd}; }

        /**
         * @brief Sets the time step.
         * @param dt Time step.
         */
        void setTimeStep(float dt) { _dt = dt; }

        /**
         * @brief Sets the output limits for the PID controller.
         * @param min Minimum output value.
         * @param max Maximum output value.
         */
        void setOutputLimits(float min, float max)
        {
            _min = min;
            _max = max;
        }

        // Getters for PID member variables

        float kp() const { return _kp; }   ///< @brief Returns the proportional gain.
        float ki() const { return _ki; }   ///< @brief Returns the integral gain.
        float kd() const { return _kd; }   ///< @brief Returns the derivative gain.
        float dt() const { return _dt; }   ///< @brief Returns the time step.
        float max() const { return _max; } ///< @brief Returns the maximum output value.
        float min() const
        {
            return _min;
        } ///< @brief Returns the minimum output value.
        ///
        // Setters for PID member variables
        void kp(float kp) { _kp = kp; }     ///< @brief Sets the proportional gain.
        void ki(float ki) { _ki = ki; }     ///< @brief Sets the integral gain.
        void kd(float kd) { _kd = kd; }     ///< @brief Sets the derivative gain.
        void dt(float dt) { _dt = dt; }     ///< @brief Sets the time step.
        void max(float max) { _max = max; } ///< @brief Sets the maximum output value.
        void min(float min) { _min = min; } ///< @brief Sets the minimum output value.

    private:
        float _kp, _ki, _kd;          ///< Proportional, integral, and derivative gains.                   ///< Time step.
        float _max, _min;             ///< Maximum and minimum output values.
        float _prev_error, _integral; ///< Internal state variables.
        float _dt;
    };
}
