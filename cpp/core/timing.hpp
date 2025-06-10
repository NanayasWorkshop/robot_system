#ifndef DELTA_CORE_TIMING_HPP
#define DELTA_CORE_TIMING_HPP

#include <chrono>
#include <iostream>
#include <iomanip>

namespace delta {

/**
 * DEBUG Timer - shows exactly what's happening (no static variables)
 */
class ScopedTimer {
private:
    double& result_ms;
    std::chrono::high_resolution_clock::time_point start_time;

public:
    explicit ScopedTimer(double& result) 
        : result_ms(result) {
        
        std::cout << "[TIMER] CONSTRUCTOR" << std::endl;
        std::cout << "[TIMER] Input variable address: " << &result_ms << std::endl;
        std::cout << "[TIMER] Input variable value: " << result_ms << std::endl;
        
        start_time = std::chrono::high_resolution_clock::now();
        
        std::cout << "[TIMER] Started timing" << std::endl;
    }
    
    ~ScopedTimer() {
        std::cout << "[TIMER] DESTRUCTOR" << std::endl;
        std::cout << "[TIMER] Variable address: " << &result_ms << std::endl;
        std::cout << "[TIMER] Variable value before: " << result_ms << std::endl;
        
        auto end_time = std::chrono::high_resolution_clock::now();
        
        // Calculate duration
        auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
        double calculated_ms = static_cast<double>(duration_ns.count()) / 1000000.0;
        
        std::cout << "[TIMER] Nanoseconds measured: " << duration_ns.count() << std::endl;
        std::cout << "[TIMER] Calculated ms: " << std::fixed << std::setprecision(6) << calculated_ms << std::endl;
        
        // Check if the variable is still valid
        std::cout << "[TIMER] About to assign..." << std::endl;
        
        result_ms = calculated_ms;
        
        std::cout << "[TIMER] Assignment complete" << std::endl;
        std::cout << "[TIMER] Variable value after: " << result_ms << std::endl;
        std::cout << "[TIMER] DESTRUCTOR COMPLETE" << std::endl;
    }
    
    // Prevent copying
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
};

} // namespace delta

#endif // DELTA_CORE_TIMING_HPP