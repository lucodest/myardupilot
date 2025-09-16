#include "LRCInput.h"

extern const AP_HAL::HAL& hal;

namespace ESP32 {

void LRCInput::init() {}

bool LRCInput::new_input() {
    auto ret = _fresh;
    _fresh = false;
    return ret;
}

uint8_t LRCInput::num_channels() {
    return RC_INPUT_NUM_CHANNELS;
}

uint16_t LRCInput::read(uint8_t chan) {
    if(chan >= RC_INPUT_NUM_CHANNELS) return 0;
    return _rc_values[chan];
}

uint8_t LRCInput::read(uint16_t* periods, uint8_t len) {
    if(len > RC_INPUT_NUM_CHANNELS) len = RC_INPUT_NUM_CHANNELS;

    for (uint8_t i = 0; i < len; i++){
        periods[i] = _rc_values[i];
    }

    return len;
}

//Periods must contain RC_INPUT_NUM_CHANNELS values
void LRCInput::write(uint16_t* periods) {

    for (uint8_t i = 0; i < RC_INPUT_NUM_CHANNELS; i++){
        _rc_values[i] = periods[i];
    }

    _last_update = AP_HAL::millis();
    _fresh = true;
}

}