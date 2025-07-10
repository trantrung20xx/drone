#include "software_timer.h"

volatile int32_t timer2_count = 0;
volatile int32_t timer2_flag  = 0;

void SetupTimer2(int32_t duration) {
    timer2_count = duration / 10; // Convert milliseconds to 10ms intervals
    timer2_flag  = 0;             // Reset the flag
}

void timer2Run() {
    if (timer2_count > 0) {
        timer2_count--;
    }
    if (timer2_count <= 0) {
        timer2_flag  = 1; // Set the flag when the timer reaches zero
        timer2_count = 0; // Prevent underflow
    }
}
