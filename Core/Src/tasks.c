#include "tasks.h"

void ToggleLED(void) {
    // This function toggles the state of the LED connected to GPIO pin PC13
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
