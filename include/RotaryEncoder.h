#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include "main.h"
#include "DWT_Delay.h"

#ifndef ENCODER_POLLING_TICK_DELAY
#define ENCODER_POLLING_TICK_DELAY 1000
#endif

#ifndef BUTTON_HOLD_TIMEOUT_MS
#define BUTTON_HOLD_TIMEOUT_MS 1000
#endif

#ifndef BUTTON_CLICK_TIMEOUT_MS
#define BUTTON_CLICK_TIMEOUT_MS 1000
#endif

typedef struct RotaryEncoderButton {
    GPIO_TypeDef *port;
    uint32_t pin;
    uint32_t clickTimer;
    uint16_t clickCount;
    bool isPressed;
    bool isReleased;
    bool isAtHold;
} RotaryEncoderButton;

typedef struct RotaryEncoder {
    TIM_TypeDef *TIMx;
    RotaryEncoderButton *button;
    uint32_t previousSystemTicks;
    uint32_t currentCounter;
    uint32_t previousCounter;
    int32_t turnValue;
    bool isTurnedRight;
    bool isTurnedLeft;
    bool isTurned;
} RotaryEncoder;


RotaryEncoder *initRotaryEncoder(TIM_TypeDef *TIMx, GPIO_TypeDef *buttonPort, uint32_t buttonPin);
RotaryEncoder *initRotaryEncoderWithoutButton(TIM_TypeDef *TIMx);

void pollRotaryEncoderStatus(RotaryEncoder *encoder);   // call this function continuously

int32_t getRotaryEncoderTurnCount(RotaryEncoder *encoder);
bool isRotaryEncoderTurnedRight(RotaryEncoder *encoder);  // return true when turning right, resets itself to false
bool isRotaryEncoderTurnedLeft(RotaryEncoder *encoder);  // returns true when turning left, resets itself to false
bool isRotaryEncoderTurned(RotaryEncoder *encoder);      // return true on any rotation, resets itself to false

bool isRotaryEncoderButtonPressed(RotaryEncoder *encoder);  // return true if button is pressed, resets itself to false
bool isRotaryEncoderButtonReleased(RotaryEncoder *encoder); // return true if button is released, resets itself to false
bool isRotaryEncoderButtonAtHold(RotaryEncoder *encoder);   // return true when button is at hold, resets itself to false
bool isRotaryEncoderButtonSingleClicked(RotaryEncoder *encoder);
bool isRotaryEncoderButtonDoubleClicked(RotaryEncoder *encoder);  // return true when button double clicked, resets click count to 0

void resetRotaryEncoder(RotaryEncoder *encoder);
void deleteRotaryEncoder(RotaryEncoder *encoder);