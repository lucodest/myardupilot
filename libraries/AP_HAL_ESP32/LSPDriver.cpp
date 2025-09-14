#include "LSPDriver.h"
#include <AP_Math/AP_Math.h>
#include "lprot.h"

extern const AP_HAL::HAL& hal;

namespace ESP32 
{

LSPDriver* LSPDriver::refs[LSP_MAX_DRIVERS];
uint8_t LSPDriver::num_drivers = 0;

LSPDriver* LSPDriver::get(uint8_t i) {
    if(i >= LSP_MAX_DRIVERS) return NULL;
    return LSPDriver::refs[i];
}

void LSPDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {

    _readbuf.set_size(RX_BUF_SIZE);
    _writebuf.set_size(TX_BUF_SIZE);

    _initialized = true;
}

void LSPDriver::_end()
{
    if (_initialized) {
        _readbuf.set_size(0);
        _writebuf.set_size(0);
    }
    _initialized = false;
}

bool LSPDriver::is_initialized()
{
    return _initialized;
}

uint32_t LSPDriver::_available()
{
    if (!_initialized) {
        return 0;
    }
    return _readbuf.available();
}

bool LSPDriver::_discard_input()
{
    if (!_initialized) {
        return false;
    }

    _readbuf.clear();

    return true;
}

void IRAM_ATTR UARTDriver::_timer_tick(void)
{
    if (!_initialized) {
        return;
    }

    //Send multiplexed stream
    uint8_t buf[256];
    _write_mutex.take_blocking();
    uint32_t av = _writebuf.available();
    if(av > 0) {
        //Only send at most 256 bytes at a tick
        uint32_t read = av > 256 ? 256 : av;
        _writebuf.read(buf, read);
        LProt::instance()->sendMultiStream(stream_num, buf, read);
    }
    _write_mutex.give();
}

ssize_t LSPDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);

    return ret;
}

size_t LSPDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }

    _write_mutex.take_blocking();

    size_t ret = _writebuf.write(buffer, size);
    _write_mutex.give();
    return ret;
}

size_t LSPDriver::back_write(const uint8_t *buffer, size_t size) {
    if (!_initialized) {
        return 0;
    }

    size_t ret = _readbuf.write(buffer, size);

    return ret;
}

bool LSPDriver::tx_pending() {
    return (_writebuf.available() > 0);
}

uint32_t LSPDriver::txspace() {
    if (!_initialized) {
        return 0;
    }
    int result =  _writebuf.space();
    result -= TX_BUF_SIZE / 4;
    return MAX(result, 0);
}

void LSPDriver::_flush() {}

}