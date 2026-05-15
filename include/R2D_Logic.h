#ifndef R2D_LOGIC_H
#define R2D_LOGIC_H

#include <Arduino.h>
#include "Config.h"

class R2D_Logic {
public:
    R2D_Logic();
    void init();
    bool update(bool tson, bool startBtn, bool brakePressed);
    bool isReadyToDrive();

private:
    int step;
    uint32_t tAux;
    bool r2dState;
};

#endif // R2D_LOGIC_H