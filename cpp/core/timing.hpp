#ifndef DELTA_CORE_TIMING_HPP
#define DELTA_CORE_TIMING_HPP

#include <chrono>

namespace delta {

/**
 * RAII Timer for automatic timing measurement with high precision
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
        
        // FIX: Use double-precision milliseconds directly - no truncation
        auto duration = std::chrono::duration<double, std::milli>(end_time - start_time);
        result_ms = duration.count();  // Already in milliseconds with full precision
    }
    
    // Prevent copying
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
};

} // namespace delta

#endif // DELTA_CORE_TIMING_HPP