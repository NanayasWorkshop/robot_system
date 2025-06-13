#pragma once

#include <string>
#include "../core/timing.hpp"

namespace delta {

/**
 * Bridge Result Template for error handling
 * Shared between all bridge implementations
 */
template<typename T>
struct BridgeResult {
    T data;
    bool success;
    std::string error_message;
    double computation_time_ms;
    
    // Success constructor
    BridgeResult(const T& result_data, double time_ms) 
        : data(result_data), success(true), computation_time_ms(time_ms) {}
    
    // Error constructor
    BridgeResult(const std::string& error, double time_ms) 
        : success(false), error_message(error), computation_time_ms(time_ms) {}
};

} // namespace delta