#include "Engineer_Limit.h"
#include "stm32h7xx_hal.h"

#define ENGINEER_LIMIT_DEBOUNCE_COUNT 3U

typedef struct {
    uint8_t counter;
    uint8_t active;
} Engineer_Limit_Channel_t;

static Engineer_Limit_Channel_t lift_bottom;
static Engineer_Limit_Channel_t transverse_zero;
static Engineer_Limit_Channel_t leadscrew_up;
static Engineer_Limit_Channel_t leadscrew_down;

static void Engineer_Limit_Debounce(Engineer_Limit_Channel_t *channel, GPIO_PinState pin_state)
{
    if (pin_state == GPIO_PIN_RESET) {
        if (channel->counter < ENGINEER_LIMIT_DEBOUNCE_COUNT) {
            channel->counter++;
        }
        channel->active = (channel->counter >= ENGINEER_LIMIT_DEBOUNCE_COUNT) ? 1U : 0U;
    } else {
        channel->counter = 0U;
        channel->active = 0U;
    }
}

void Engineer_Limit_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // 沿用老车接线：PB4 为图传横移零位，PD7 为图传抬升底部，均为低电平有效。
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOD, &gpio);

    // 沿用老车接线：PB11 为丝杠上限位，PD10 为丝杠下限位，均为低电平有效。
    gpio.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOD, &gpio);

    lift_bottom.counter = 0U;
    lift_bottom.active = 0U;
    transverse_zero.counter = 0U;
    transverse_zero.active = 0U;
    leadscrew_up.counter = 0U;
    leadscrew_up.active = 0U;
    leadscrew_down.counter = 0U;
    leadscrew_down.active = 0U;
}

void Engineer_Limit_Update(void)
{
    Engineer_Limit_Debounce(&transverse_zero, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4));
    Engineer_Limit_Debounce(&lift_bottom, HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7));
    Engineer_Limit_Debounce(&leadscrew_up, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11));
    Engineer_Limit_Debounce(&leadscrew_down, HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10));
}

uint8_t Engineer_Limit_Lift_Bottom(void)
{
    return lift_bottom.active;
}

uint8_t Engineer_Limit_Transverse_Zero(void)
{
    return transverse_zero.active;
}

uint8_t Engineer_Limit_LeadScrew_Up(void)
{
    return leadscrew_up.active;
}

uint8_t Engineer_Limit_LeadScrew_Down(void)
{
    return leadscrew_down.active;
}
