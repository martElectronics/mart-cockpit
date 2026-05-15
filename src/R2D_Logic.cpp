#include "R2D_Logic.h"

R2D_Logic::R2D_Logic() : step(0), tAux(0), r2dState(false) {}

void R2D_Logic::init() {
    pinMode(PIN_START, INPUT_PULLUP);
    pinMode(PIN_TSON, INPUT);
    pinMode(PIN_TSON_EXT, INPUT);
    pinMode(PIN_BUZZ, OUTPUT);
    digitalWrite(PIN_BUZZ, LOW);
}

bool R2D_Logic::update(bool tson, bool startBtn, bool brakePressed) {
    // Control del tiempo del buzzer
    if (r2dState && (millis() - tAux) >= CFG_TIME_SOUND_R2D) {
        digitalWrite(PIN_BUZZ, LOW);
    }

    switch (step) {
        case 0:
            if (tson) step = 10;
            break;
        case 10:
            if (!tson) {
                step = 0;
            } else if (startBtn && brakePressed) {
                tAux = millis();
                digitalWrite(PIN_BUZZ, HIGH); // Suena el buzzer
                step = 20;
            }
            break;
        case 20:
            if (!tson) {
                step = 0;
                r2dState = false;
            } else {
                r2dState = true;
            }
            break;
    }
    return r2dState;
}

bool R2D_Logic::isReadyToDrive() {
    return r2dState;
}