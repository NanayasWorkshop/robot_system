#pragma once

#include <chrono>
#include <unordered_map>
#include <string>

namespace delta {

/**
 * High-resolution timer for performance measurement
 * Automatically accumulates time when destroyed (RAII pattern)
 */
class ScopedTimer {
private:
    double& time_accumulator_;
    std::chrono::high_resolution_clock::time_point start_time_;
    
public:
    explicit ScopedTimer(double& accumulator) 
        : time_accumulator_(accumulator), 
          start_time_(std::chrono::high_resolution_clock::now()) {}
    
    ~ScopedTimer() {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time_);
        time_accumulator_ += duration.count() / 1000.0; // Convert to milliseconds
    }
    
    // Get elapsed time without destroying timer
    double get_elapsed_ms() const {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            current_time - start_time_);
        return duration.count() / 1000.0; // Convert to milliseconds
    }
};

/**
 * Simple high-resolution timer for one-off measurements
 */
class Timer {
private:
    std::chrono::high_resolution_clock::time_point start_time_;
    
public:
    Timer() : start_time_(std::chrono::high_resolution_clock::now()) {}
    
    void reset() {
        start_time_ = std::chrono::high_resolution_clock::now();
    }
    
    double elapsed_ms() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time_);
        return duration.count() / 1000.0; // Convert to milliseconds
    }
    
    double elapsed_us() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time_);
        return static_cast<double>(duration.count());
    }
    
    double elapsed_seconds() const {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time_);
        return duration.count() / 1000000.0; // Convert to seconds
    }
};

/**
 * Performance profiler for multiple named timers
 */
class Profiler {
private:
    std::unordered_map<std::string, double> timers_;
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> start_times_;
    
public:
    void start(const std::string& name) {
        start_times_[name] = std::chrono::high_resolution_clock::now();
    }
    
    void end(const std::string& name) {
        auto it = start_times_.find(name);
        if (it != start_times_.end()) {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - it->second);
            timers_[name] += duration.count() / 1000.0; // Accumulate in milliseconds
            start_times_.erase(it);
        }
    }
    
    double get_time(const std::string& name) const {
        auto it = timers_.find(name);
        return (it != timers_.end()) ? it->second : 0.0;
    }
    
    void reset() {
        timers_.clear();
        start_times_.clear();
    }
    
    void reset(const std::string& name) {
        timers_[name] = 0.0;
        start_times_.erase(name);
    }
    
    std::string get_report() const {
        std::string report = "Performance Report:\n";
        for (const auto& [name, time] : timers_) {
            report += "  " + name + ": " + std::to_string(time) + " ms\n";
        }
        return report;
    }
};

} // namespace delta