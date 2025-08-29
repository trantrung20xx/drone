#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"
#include "scheduler.h"

#define STABLE_STATE GPIO_PIN_RESET // Stable state for button pressed
#define GPIO_PULLx   GPIO_PULLUP    // Pull up

// Threshold for button pressed
#define DOUBLE_CLICK      400
#define SHORT_PRESS       400
#define LONG_PRESS        3000
#define TIME_THRESHOLT(x) ((x) / TICKS)

typedef enum
{
    BUTTON_RELEASED       = 0,
    BUTTON_PRESSED        = 1,
    BUTTON_LONG_PRESSED   = 2,
    BUTTON_DOUBLE_CLICKED = 3,
} ButtonState_t;

typedef struct
{
    ButtonState_t state;            // Current state of the button (pressed, released, etc.)
    GPIO_TypeDef *GPIOx;            // GPIO port of the button
    uint16_t      GPIO_Pin;         // GPIO pin number of the button
    uint8_t       filter1;          // Filter states for debouncing
    uint8_t       filter2;          // Filter states for debouncing
    uint8_t       filter3;          // Filter states for debouncing
    int8_t        counter;          // Counter for double-click detection
    uint32_t      start_press_time; // Timestamp of the first button press
    uint32_t      last_press_time;  // Timestamp of the last button press
} Button_t;

void BTNx_Init(Button_t *button, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void BTNx_Handler(Button_t *button);
void BTNx_Handler_Callback(Button_t *button);

// Callbacks for button events
void BTNx_OnPressed(Button_t *button);
void BTNx_OnReleased(Button_t *button);
void BTNx_OnLongPressed(Button_t *button);
void BTNx_OnDoubleClicked(Button_t *button);

#endif // INC_BUTTON_H_
