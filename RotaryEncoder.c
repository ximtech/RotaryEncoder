#include "RotaryEncoder.h"


RotaryEncoder *initRotaryEncoder(TIM_TypeDef *TIMx, GPIO_TypeDef *buttonPort, uint32_t buttonPin) {
    RotaryEncoder *encoder = malloc(sizeof(RotaryEncoder));
    if (encoder == NULL) return NULL;
    resetRotaryEncoder(encoder);
    encoder->TIMx = TIMx;

    if (buttonPort != NULL) {
        RotaryEncoderButton *encoderButton = malloc(sizeof(RotaryEncoderButton));
        if (encoderButton == NULL) {
            deleteRotaryEncoder(encoder);
            return NULL;
        }
        encoderButton->port = buttonPort;
        encoderButton->pin = buttonPin;
        encoderButton->clickTimer = 0;
        encoderButton->clickCount = 0;
        encoderButton->isPressed = false;
        encoderButton->isReleased = false;
        encoderButton->isAtHold = false;
        encoder->button = encoderButton;
    }

    dwtDelayInit();
    LL_TIM_SetCounter(TIMx, 0);
    LL_TIM_EnableCounter(TIMx);
    return encoder;
}

RotaryEncoder *initRotaryEncoderWithoutButton(TIM_TypeDef *TIMx) {
    return initRotaryEncoder(TIMx, NULL, 0);
}

void pollRotaryEncoderStatus(RotaryEncoder *encoder) {
    if ((getSystemTicks() - encoder->previousSystemTicks) > ENCODER_POLLING_TICK_DELAY) {   // check TIM status every ms
        encoder->currentCounter = LL_TIM_GetCounter(encoder->TIMx) >> 2;

        if (encoder->currentCounter != encoder->previousCounter) {  // protection against bounce
            if (encoder->currentCounter > encoder->previousCounter) {
                encoder->turnValue++;
                encoder->isTurnedRight = true;
                encoder->isTurnedLeft = false;
            } else {
                encoder->turnValue--;
                encoder->isTurnedLeft = true;
                encoder->isTurnedRight = false;
            }

            encoder->isTurned = true;
            encoder->previousCounter = encoder->currentCounter;
        }

        encoder->previousSystemTicks = getSystemTicks();
    }

    if (encoder->button != NULL) { // check button state if enabled
        bool isButtonPressed = !LL_GPIO_IsInputPinSet(encoder->button->port, encoder->button->pin);

        if (!encoder->button->isPressed && isButtonPressed) { // button is pressed
            encoder->button->isPressed = true;
            encoder->button->isReleased = false;
            encoder->button->clickCount++;
            encoder->button->clickTimer = currentMilliSeconds();
        }

        if (encoder->button->isPressed && isButtonPressed) {    // button is at hold position
            if ((currentMilliSeconds() - encoder->button->clickTimer) >= BUTTON_HOLD_TIMEOUT_MS) {
                encoder->button->isAtHold = true;
            } else {
                encoder->button->isAtHold = false;
            }
        }

        if (encoder->button->isPressed && !isButtonPressed) {  // button is released
            encoder->button->isReleased = true;
            encoder->button->isPressed = false;
        }

        if (encoder->button->clickCount > 0 && (currentMilliSeconds() - encoder->button->clickTimer >= BUTTON_CLICK_TIMEOUT_MS)) {// click handling
            encoder->button->clickCount = 0;
        }
    }
}

int32_t getRotaryEncoderTurnCount(RotaryEncoder *encoder) {
    return encoder->turnValue;
}

bool isRotaryEncoderTurnedRight(RotaryEncoder *encoder) {
    bool isTurnedRight = encoder->isTurnedRight;
    encoder->isTurnedRight = false;
    return isTurnedRight;
}

bool isRotaryEncoderTurnedLeft(RotaryEncoder *encoder) {
    bool isTurnedLeft = encoder->isTurnedLeft;
    encoder->isTurnedLeft = false;
    return isTurnedLeft;
}

bool isRotaryEncoderTurned(RotaryEncoder *encoder) {
    bool isTurned = encoder->isTurned;
    encoder->isTurned = false;
    return isTurned;
}

bool isRotaryEncoderButtonPressed(RotaryEncoder *encoder) {
    if (encoder->button->isPressed) {
        encoder->button->isPressed = false;
        encoder->button->clickCount = 0;
        return true;
    }
    return false;
}

bool isRotaryEncoderButtonReleased(RotaryEncoder *encoder) {
    if (encoder->button->isReleased) {
        encoder->button->isReleased = false;
        encoder->button->clickCount = 0;
        return true;
    }
    return false;
}

bool isRotaryEncoderButtonAtHold(RotaryEncoder *encoder) {
    if (encoder->button->isAtHold) {
        encoder->button->isAtHold = false;
        return true;
    }
    return false;
}

bool isRotaryEncoderButtonSingleClicked(RotaryEncoder *encoder) {
    if (encoder->button->clickCount == 1 && !encoder->button->isAtHold) {
        encoder->button->isPressed = false;
        encoder->button->isReleased = false;
        return true;
    }
    return false;
}

bool isRotaryEncoderButtonDoubleClicked(RotaryEncoder *encoder) {
    if (encoder->button->clickCount == 2 && !encoder->button->isAtHold) {
        encoder->button->isPressed = false;
        encoder->button->isReleased = false;
        encoder->button->clickCount = 0;
        return true;
    }
    return false;
}

void resetRotaryEncoder(RotaryEncoder *encoder) {
    encoder->previousSystemTicks = 0;
    encoder->currentCounter = 0;
    encoder->previousCounter = 0;
    encoder->turnValue = 0;
    encoder->isTurnedRight = false;
    encoder->isTurnedLeft = false;
    encoder->isTurned = false;
}

void deleteRotaryEncoder(RotaryEncoder *encoder) {
    if (encoder != NULL) {
        free(encoder->button);
        free(encoder);
    }
}