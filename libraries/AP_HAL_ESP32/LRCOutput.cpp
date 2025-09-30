#include "LRCOutput.h"

#include "lprot.h"

namespace ESP32 {

void LRCOutput::init() {}

void LRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t LRCOutput::get_freq(uint8_t chan) {
    return 200;
}

void LRCOutput::enable_ch(uint8_t chan)
{}

void LRCOutput::disable_ch(uint8_t chan)
{}

void LRCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan >= ARRAY_SIZE(value)) return;
    
    value[chan] = period_us;
    
    if(!_corked) publish();
}

uint16_t LRCOutput::read(uint8_t chan)
{
    if (chan >= ARRAY_SIZE(value)) return 0;

    return value[chan];
}

void LRCOutput::read(uint16_t* period_us, uint8_t len)
{
    if (len > ARRAY_SIZE(value)) len = ARRAY_SIZE(value);
    
    memcpy(period_us, value, len * sizeof(value[0]));
}

void LRCOutput::cork(void) {
    _corked = true;
}

void LRCOutput::push(void) {

    if (!_corked) {
        return;
    }

    publish();

    _corked = false;
}

void LRCOutput::publish() {
    LProt::RC_DATA_t rc;

    rc.chan1 = value[0];
    rc.chan2 = value[1];
    rc.chan3 = value[2];
    rc.chan4 = value[3];
    rc.chan5 = value[4];
    rc.chan6 = value[5];

    LProt::instance()->sendRcData(&rc);
}

}