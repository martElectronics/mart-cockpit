
#include <Arduino.h>



class APPS {

public:
    APPS(uint16_t rest1, uint16_t full1,
         uint16_t rest2, uint16_t full2,
         uint16_t vMin = 50, uint16_t vMax = 975,
         float maxDiffPct = 10.0,
         uint16_t minDelta = 50)
        : _rest1(rest1), _full1(full1),
          _rest2(rest2), _full2(full2),
          _vMin(vMin), _vMax(vMax),
          _maxDiffPct(maxDiffPct), _minDelta(minDelta) {}

    void update( uint16_t raw1 ,  uint16_t raw2) {
        _raw1 = raw1;
        _raw2 = raw2;

        _scaled1 = scale(_raw1, _rest1, _full1);
        _scaled2 = scale(_raw2, _rest2, _full2);

        checkImplausibility();
    }

    void debug() {
        unsigned long now = millis();
        if (now - _lastDebugTime >= 1000) {
            _lastDebugTime = now;

            Serial.println("==== APPS DEBUG INFO ====");
            Serial.print("Raw1: "); Serial.print(_raw1);
            Serial.print(" | Raw2: "); Serial.println(_raw2);

            Serial.print("Scaled1: "); Serial.print(_scaled1);
            Serial.print(" | Scaled2: "); Serial.println(_scaled2);

            Serial.print("Implausible Diff: ");
            Serial.println(_implausibleDiff ? "YES" : "NO");

            Serial.print("Short to VCC/GND: ");
            Serial.println(_shortToVccGnd ? "YES" : "NO");

            Serial.print("Signals Too Similar: ");
            Serial.println(_signalsTooSimilar ? "YES" : "NO");

            Serial.println("=========================\n");
        }
    }

    int getScaled1() const { return _scaled1; }
    int getScaled2() const { return _scaled2; }

    bool isImplausibleDiff() const { return _implausibleDiff; }
    bool isShortToVccGnd() const { return _shortToVccGnd; }
    bool isSignalsTooSimilar() const { return _signalsTooSimilar; }

private:
    int _rest1, _full1, _rest2, _full2;
    int _vMin, _vMax;
    float _maxDiffPct;
    int _minDelta;

    int _raw1 = 0, _raw2 = 0;
    int _scaled1 = 0, _scaled2 = 0;

    bool _implausibleDiff = false;
    bool _shortToVccGnd = false;
    bool _signalsTooSimilar = false;

    unsigned long _lastDebugTime = 0;

    int scale(uint16_t val, uint16_t minVal, uint16_t maxVal) {
        if (val < minVal) val = minVal;
        if (val > maxVal) val = maxVal;
        return map(val, minVal, maxVal, 0, 1000);
    }

    void checkImplausibility() {
        _implausibleDiff = abs(_scaled1 - _scaled2) > (_maxDiffPct / 100.0 * 1000);
        _shortToVccGnd = (_raw1 <= _vMin || _raw1 >= _vMax || _raw2 <= _vMin || _raw2 >= _vMax);
        _signalsTooSimilar = abs(_raw1 - _raw2) < _minDelta;
    }
};

