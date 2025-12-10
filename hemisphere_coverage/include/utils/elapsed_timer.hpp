// Minimal elapsed timer utility to avoid external dependencies.
#pragma once

#include <chrono>

namespace hemisphere
{
    class ElapsedTimer
    {
    public:
        ElapsedTimer() : start_time_(Clock::now()) {}

        void start() { start_time_ = Clock::now(); }

        std::chrono::duration<double> elapsedSec() const
        {
            return std::chrono::duration<double>(Clock::now() - start_time_);
        }

    private:
        using Clock = std::chrono::steady_clock;
        Clock::time_point start_time_;
    };
}
