#pragma once

#include "AP_HAL_ESP32.h"

//My protocol

/* 
Packet format

byte 0 - Magic 0xE9
byte 1 - Packet type
byte 2 - Payload length n
bytes 3 - n Payload
bytes n+1 / n+2 - CRC32 only first 2 bytes

Packet types
Most significant 4 bits

0x2 - Multiplexed Stream data
    Next 4 bits are the stream id 0-15
    Payload is data to be written to the stream

0x3 - Sensor data
    Next 4 bits are the sensor type (see enum)
    Payload is sensor data structure
*/

namespace ESP32 
{

#define LPROT_MAGIC 0xE9

class LProt 
{
public:

    LProt(LProt &other) = delete;

    void operator=(const LProt &) = delete;

    static LProt* instance();

    typedef enum {
        STREAM = 2,
        SENSOR = 3
    } LPROT_PACKET_TYPE;

    typedef enum {
        //Barometer
        BARO = 1,
        //3-Axis Magnetometer (Compass)
        MAG = 2,

        IMU = 3
    } LPROT_SENSOR_TYPE;

    typedef struct PACKED {
        uint32_t pressure;  //In Pascal (Abs)
        uint16_t temperature;   //Temperature in 0.1 Celcius
    } BARO_DATA_t;

    typedef struct PACKED {
        //In miliGauss
        int16_t mag_x;
        int16_t mag_y;
        int16_t mag_z;
    } MAG_DATA_t;

    typedef struct PACKED {
        int16_t ax;
        int16_t ay;
        int16_t az;
        int16_t gx;
        int16_t gy;
        int16_t gz;
    } IMU_DATA_t;

    //Handle raw recived data, MUST be called by user
    void handleRx(uint8_t* data, uint16_t len);

    //Functions for sending frames, all are called by user
    void sendMultiStream(uint8_t id, uint8_t* data, uint8_t len);

protected:

    LProt();

    static LProt* singleton_;

private:
    uint32_t crc_errors = 0;
    bool inPacket = false;
    uint16_t pkt_index = 0;
    uint8_t *pkt_buffer;
    AP_HAL::UARTDriver *uart;

    void update_thread(void);

    inline void handleSensorData(uint8_t type, uint8_t* data, uint8_t len);
    void handlePayload(uint8_t type, uint8_t* data, uint8_t len);
    void sendFrame(uint8_t type, uint8_t* payload, uint8_t len);

    //Function for calculating crc, MUST be implemented by user
    uint16_t calculateCrc(uint8_t* data, uint16_t len);

    //Functions callbacks for handling frame types, all implemented by user
    void handleMultiStream(uint8_t id, uint8_t* data, uint8_t len);
    void handleBaroData(BARO_DATA_t data) {};
    void handleMagData(MAG_DATA_t data) {};
    void handleImuData(IMU_DATA_t data) {};

    //Handle raw data to transmitt, MUST be implemented by user
    void handleTx(uint8_t* data, uint16_t len);

    void handleError(uint8_t err, ...);
};

}