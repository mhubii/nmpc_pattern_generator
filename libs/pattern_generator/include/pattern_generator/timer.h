#ifndef TIMER_H_
#define TIME_H_

#include <chrono>

// Usage
// Timer(START);
// double elapsed_time = Timer(STOP);

// Speed benchmarking :).
enum {
    START,
    STOP
};

auto Timer(int set) -> double {

    double msec;

    // Set a timer.
    switch (set)
    {
        case START:
            static auto t1 = std::chrono::high_resolution_clock::now();
            msec = 0.;
            break;
        case STOP:
            static auto t2 = std::chrono::high_resolution_clock::now();
            msec = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            break;
        default:
            break;
    }
    return msec;
}

#endif
