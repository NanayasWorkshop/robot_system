#ifndef DELTA_CORE_TIMING_HPP
#define DELTA_CORE_TIMING_HPP

#include <chrono>

namespace delta {

class ScopedTimer {
private:
    double& result_ms;
    std::chrono::high_resolution_clock::time_point start_time;

public:
    explicit ScopedTimer(double& result) 
        : result_ms(result), start_time(std::chrono::high_resolution_clock::now()) {
        result_ms = 0.0;
    }
    
    ~ScopedTimer() noexcept {
        try {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
            result_ms = static_cast<double>(duration_ns.count()) / 1000000.0;
        } catch (...) {
            result_ms = 0.0;
        }
    }
    
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
};

}

#endif