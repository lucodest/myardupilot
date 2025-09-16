#pragma once

#include "AP_HAL_ESP32.h"
#include "Semaphores.h"

namespace ESP32 {

#define RC_INPUT_NUM_CHANNELS 6

class LRCInput : public AP_HAL::RCInput {
public:
    void init() override;
    bool  new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;
    virtual const char *protocol() const override { return "LProt"; }

    void write(uint16_t* periods);
    
private:

    uint16_t _rc_values[RC_INPUT_NUM_CHANNELS] = {0};
    uint64_t _last_update;
    Semaphore rcin_mutex;
    bool _fresh;
    //bool _init;

};

}