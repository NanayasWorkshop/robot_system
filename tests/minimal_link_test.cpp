#include <iostream>
#include "core/timing.hpp"

using namespace delta;

int main() {
    std::cout << "🧪 Library linking test..." << std::endl;
    
    // Test ScopedTimer
    double test_time = 0.0;
    {
        ScopedTimer timer(test_time);
        // Do some minimal work
        for (int i = 0; i < 1000; ++i) {
            volatile int x = i * i;
        }
    }
    
    std::cout << "✅ ScopedTimer test: " << test_time << " ms" << std::endl;
    std::cout << "✅ Library linking successful!" << std::endl;
    
    return 0;
}
