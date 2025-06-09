#ifndef DELTA_CORE_TIMING_HPP
#define DELTA_CORE_TIMING_HPP

#include <chrono>

namespace delta {

/**
 * RAII Timer for automatic timing measurement
 * Usage:
 *   double time_ms = 0.0;
 *   {
 *       ScopedTimer timer(time_ms);
 *       // ... code to time ...
 *   }  // time_ms automatically set when scope exits
 */
class ScopedTimer {
private:
    double& result_ms;
    std::chrono::high_resolution_clock::time_point start_time;

public:
    explicit ScopedTimer(double& result) 
        : result_ms(result), start_time(std::chrono::high_resolution_clock::now()) {}
    
    ~ScopedTimer() {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        result_ms = duration.count() / 1000.0;  // Convert to milliseconds
    }
    
    // Prevent copying
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
};

} // namespace delta

#endif // DELTA_CORE_TIMING_HPP