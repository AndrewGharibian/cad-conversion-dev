// =============================================================================
// stage_timer.hpp — progress output for the pipeline stages.
// =============================================================================
#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <utility>

// Construct one at the start of a stage: it prints the stage name right away
// (flushed, so progress shows up live on big models) and how long the stage
// took when it goes out of scope.
class StageTimer
{
public:
    explicit StageTimer(std::string name) : name_(std::move(name)), start_(std::chrono::steady_clock::now())
    {
        std::cout << "  -> " << name_ << "..." << std::endl;
    }
    ~StageTimer()
    {
        std::cout << "     " << name_ << ": " << Seconds() << " s" << std::endl;
    }
    double Seconds() const
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    }

private:
    std::string name_;
    std::chrono::steady_clock::time_point start_;
};
