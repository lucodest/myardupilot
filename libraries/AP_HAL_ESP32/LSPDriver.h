#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>

namespace ESP32
{

#define LSP_MAX_DRIVERS 3

class LSPDriver : public AP_HAL::UARTDriver {
public:
    LSPDriver()
        : AP_HAL::UARTDriver()
    {
        _initialized = false;
        LSPDriver::refs[LSPDriver::num_drivers] = this;
        stream_num = LSPDriver::num_drivers;
        LSPDriver::num_drivers++;
    }

    static LSPDriver* get(uint8_t i);

    virtual ~LSPDriver() = default;
    /* Empty implementations of UARTDriver virtual methods */
    bool is_initialized() override;
    bool tx_pending() override;

    /* Empty implementations of Stream virtual methods */
    uint32_t txspace() override;

    void _timer_tick(void) override;

    size_t back_write(const uint8_t *buffer, size_t size);

private:
    const size_t TX_BUF_SIZE = 1024;
    const size_t RX_BUF_SIZE = 1024;

    static LSPDriver* refs[];
    static uint8_t num_drivers;

    bool _initialized;
    uint8_t stream_num;
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    Semaphore _write_mutex;

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t size) override WARN_IF_UNUSED;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;
};

}
