#pragma once

class Timer {
public:
    ~Timer() {
        delete[] _ignore;
    }
    Timer() {}
    Timer(unsigned long(*mil)(), void(*func[])(void), uint16_t* delays, uint8_t length) {
        _mil = mil;
        _funcArray = func;
        _delayArray = delays;
        _length = length;
        _mainFlag = (_mil == millis ? 1 : 0);
        _ignore = new int8_t[length];

        for (uint8_t i = 0; i < _length; i++)_ignore[i] = 0;
    }
    /*
        Timer(unsigned long(*mil)(), void(*func)(void), uint16_t* delays, uint8_t length) {
            _mil = mil;
            void(*q[])(void) = { func };
            _funcArray = q;
            _delayArray = delays;
            _length = length;
            _mainFlag = (_mil == millis ? 1 : 0);
            _ignore = new int8_t[length];

            _ignore[0] = 0;
        }
    */
    void begin(unsigned long(*mil)(), void(*func[])(void), uint16_t* delays, uint8_t length) {
        _mil = mil;
        _funcArray = func;
        _delayArray = delays;
        _length = length;
        _mainFlag = (_mil == millis ? 1 : 0);
        _ignore = new int8_t[length];

        for (uint8_t i = 0; i < _length; i++)_ignore[i] = 0;
    }

    void remove(uint8_t x) { _ignore[x] = 0; }
    void add(uint8_t x) { _ignore[x] = 1; }
    void addNext(uint8_t x) { _ignore[x] = 2; }
    void always(uint8_t x) { _ignore[x] = 3; }
    void alwaysNext(uint8_t x) { _ignore[x] = 4; }

    void tick() {   //нужно засутунь в loop()
        if (_mil() - _tmr >= (_mainFlag ? 1 : 1000)) {
            _tmr = _mil();
            _counter++;
            for (uint8_t i = 0; i < _length; i++) {
                if (_counter % _delayArray[i] == 0) {
                    if (_ignore[i] == 2) { _ignore[i] = 1; continue; }
                    if (_ignore[i] == 4) { _ignore[i] = 3; continue; }
                    if (_ignore[i] == 1 || _ignore[i] == 3) {
                        _funcArray[i]();
                        if (_ignore[i] == 1)_ignore[i] = 0;
                    }
                }
            }
        }
    }


private:
    bool _mainFlag;
    uint32_t _counter;
    uint8_t _length;
    int8_t* _ignore;

    uint32_t _tmr;
    unsigned long(*_mil)();
    void (**_funcArray)(void);
    uint16_t* _delayArray;
};
