#include "button.h"

__weak void BTNx_OnPressed(Button_t *button) { (void)button; }
__weak void BTNx_OnReleased(Button_t *button) { (void)button; }
__weak void BTNx_OnLongPressed(Button_t *button) { (void)button; }
__weak void BTNx_OnDoubleClicked(Button_t *button) { (void)button; }

void BTNx_Init(Button_t *button, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    button->state            = BUTTON_RELEASED;
    button->GPIOx            = GPIOx;
    button->GPIO_Pin         = GPIO_Pin;
    button->filter1          = 0;
    button->filter2          = 0;
    button->filter3          = 0;
    button->counter          = 0;
    button->start_press_time = 0;
    button->last_press_time  = 0;

    // Initialize the GPIO pin for the button
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = GPIO_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull             = GPIO_PULLx;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW; // Low speed for button input
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void BTNx_Handler(Button_t *button)
{
    // Read the current state of the button
    uint8_t current_state = HAL_GPIO_ReadPin(button->GPIOx, button->GPIO_Pin);
    // Update the filter states
    button->filter1 = button->filter2;
    button->filter2 = button->filter3;
    button->filter3 = current_state;

    // Check if the button state is released
    if (current_state != STABLE_STATE && button->filter3 != button->filter2)
    {
        button->counter++; // Increment the number of buttons
        // Button released
        if (button->state != BUTTON_RELEASED)
        {
            button->state = BUTTON_RELEASED;
        }
    }

    // Check if the button is pressed
    if (current_state == STABLE_STATE && button->filter1 == button->filter2 &&
        button->filter2 == button->filter3)
    {

        button->start_press_time += TICKS;
        uint32_t time_diff = button->start_press_time - button->last_press_time;

        if (time_diff > TIME_THRESHOLT(LONG_PRESS) && button->counter == 0)
        {
            // Button long pressed
            if (button->state != BUTTON_LONG_PRESSED)
            {
                button->state   = BUTTON_LONG_PRESSED;
                button->counter = -1; // Reset counter after long press
                // button->counter will be incremented to 0 after the button is released
                button->last_press_time = button->start_press_time;
            }
        }
        else if (time_diff > TIME_THRESHOLT(SHORT_PRESS) && button->counter == 1)
        {
            // Button short pressed
            if (button->state != BUTTON_PRESSED)
            {
                button->state           = BUTTON_PRESSED;
                button->counter         = 0; // Reset counter after single press
                button->last_press_time = button->start_press_time;
            }
        }
        else if (time_diff <= TIME_THRESHOLT(DOUBLE_CLICK) && button->counter >= 2)
        {
            // Button double-clicked
            if (button->state != BUTTON_DOUBLE_CLICKED)
            {
                button->state           = BUTTON_DOUBLE_CLICKED;
                button->counter         = 0; // Reset counter after double click
                button->last_press_time = button->start_press_time;
            }
        }
    }
}

void BTNx_Handler_Callback(Button_t *button)
{
    BTNx_Handler(button);
    switch (button->state)
    {
        case BUTTON_LONG_PRESSED:
            BTNx_OnLongPressed(button);
            break;
        case BUTTON_PRESSED:
            BTNx_OnPressed(button);
            break;
        case BUTTON_DOUBLE_CLICKED:
            BTNx_OnDoubleClicked(button);
            break;
        case BUTTON_RELEASED:
            BTNx_OnReleased(button);
            break;
        default:
            break;
    }
}
