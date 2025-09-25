#include "lprot.h"
#include "LSPDriver.h"
#include "LRCInput.h"
#include "AP_Math/crc.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

using namespace ESP32;

LProt* LProt::singleton_ = nullptr;

LProt* LProt::instance() {
    if(singleton_ == nullptr) {
        singleton_ = new LProt();
    }

    return singleton_;
}

LProt::LProt() {
    pkt_buffer = new uint8_t[261];

    //Use serialmanager later
    uart = hal.serial(2);

    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&LProt::update_thread, void), "LProt", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0);
}

//Implement missing methods in AP_ExternalAHRS

void AP_ExternalAHRS::init(void) {
    LProt::instance();
}

int8_t AP_ExternalAHRS::get_port(AvailableSensor sensor) const
{
    return 2;
};

void AP_ExternalAHRS::update() {}

float AP_ExternalAHRS::get_IMU_rate(void) const {
    return 400;
}

//End

void LProt::update_thread(void) {
    uint8_t buf[128];

    uart->begin(115200, 512, 512);

    while(true) {
        uint32_t av = uart->available();
        if(av > 0) {
            av = av > 128 ? 128 : av;
            uart->read(buf, av);
            handleRx(buf, av);
        }

        hal.scheduler->delay(1);
    }
    
}

void LProt::handleTx(uint8_t* data, uint16_t len) {
    uart->write(data, len);
}

inline void LProt::handleSensorData(uint8_t type, uint8_t* data, uint8_t len) {
    switch (type)
    {
        case BARO:
            handleBaroData(*((BARO_DATA_t*) data));
            break;
        case MAG:
            handleMagData(*((MAG_DATA_t*) data));
            break;
        case IMU:
            handleImuData(*((IMU_DATA_t*) data));
            break;
        case RC:
            handleRcData(*((RC_DATA_t*) data));
            break;
        default:

            break;
    }
}

void LProt::handlePayload(uint8_t type, uint8_t* data, uint8_t len){
    switch (type >> 4)
    {
        case STREAM:
            handleMultiStream(type & 0xF, data, len);
            break;
        case SENSOR:
            handleSensorData(type & 0xF, data, len);
            break;
        default:

            break;
    }
}

uint16_t LProt::calculateCrc(uint8_t* data, uint16_t len) {
    uint32_t crc = crc_crc32(0xFFFFFFFF, data, len);
    uint8_t rem = len % 4;

    if(rem){
        uint8_t zr[] = {0x0, 0x0, 0x0};
        crc = crc_crc32(crc, zr, 4 - rem);
    }

    return crc;
}

void LProt::handleRx(uint8_t* data, uint16_t len){

    for(uint16_t i = 0; i < len; i++){
        pkt_buffer[pkt_index] = data[i];
        pkt_index++;

        //Start of frame recived
        if(inPacket){

            //Got lenght
            if(pkt_index > 2){
                uint8_t pkt_size = 5 + pkt_buffer[2];
                //Last byte of packet
                if(pkt_size == pkt_index){
                    //Debug
                    //handleError(2, pkt_buffer, pkt_size);

                    pkt_size -= 2;
                    uint16_t calc_crc = calculateCrc(pkt_buffer, pkt_size);
                    uint16_t pkt_crc = (pkt_buffer[pkt_size] << 8) | pkt_buffer[pkt_size+1];

                    //Packet OK
                    if(pkt_crc == calc_crc){
                        handlePayload(pkt_buffer[1], pkt_buffer + 3, pkt_buffer[2]);
                    //Crc error
                    }else{
                        hal.console->printf("crce\n");

                        handleError(1, calc_crc, pkt_crc);
                        crc_errors++;
                    }

                    pkt_index = 0;
                    inPacket = false;
                }
            }

        }else if(data[i] == LPROT_MAGIC){
            inPacket = true;
        }else{
            //Discard byte
            pkt_index--;
        }
    }
}

void LProt::sendFrame(uint8_t type, uint8_t* payload, uint8_t len) {
    uint8_t buffer[261];
    //Assemble frame
    buffer[0] = LPROT_MAGIC;
    buffer[1] = type;
    buffer[2] = len;
    //Copy payload
    uint16_t i = 0;
    for(; i < len; i++){
        buffer[3 + i] = payload[i];
    }
    //Total size
    i = 3 + len;
    //Calculate crc
    uint16_t crc = calculateCrc(buffer, i);
    buffer[i] = crc >> 8;
    buffer[i+1] = crc & 0xFF;

    handleTx(buffer, i + 2);
}

void LProt::sendMultiStream(uint8_t id, uint8_t* data, uint8_t len) {
    sendFrame((STREAM << 4) | (id & 0xf), data, len);
}

void LProt::handleMultiStream(uint8_t id, uint8_t* data, uint8_t len) {
    LSPDriver* vu = LSPDriver::get(id);
    if(vu != NULL) {
        vu->back_write(data, len);
    }
}

void LProt::handleRcData(RC_DATA_t data) {
    uint16_t periods[RC_INPUT_NUM_CHANNELS];
    //Find a better way pls
    periods[0] = data.chan1;
    periods[1] = data.chan2;
    periods[2] = data.chan3;
    periods[3] = data.chan4;
    periods[4] = data.chan5;
    periods[5] = data.chan6;

    ((LRCInput*) hal.rcin)->write(periods);
}

void LProt::handleBaroData(BARO_DATA_t data) {
    AP_ExternalAHRS::baro_data_message_t pkt;

    pkt.pressure_pa = data.pressure;
    pkt.temperature = data.temperature / 10.0f;

    AP::baro().handle_external(pkt);
}

void LProt::handleMagData(MAG_DATA_t data) {
    AP_ExternalAHRS::mag_data_message_t pkt;

    pkt.field.x = data.mag_x;
    pkt.field.y = data.mag_y;
    pkt.field.z = data.mag_z;

    AP::compass().handle_external(pkt);
}

#define LSB_PER_G 8192.0f

void LProt::handleImuData(IMU_DATA_t data) {
    AP_ExternalAHRS::ins_data_message_t pkt;

    pkt.accel.x = (data.ax / LSB_PER_G) * GRAVITY_MSS;
    pkt.accel.y = (data.ay / LSB_PER_G) * GRAVITY_MSS;
    pkt.accel.z = (data.az / LSB_PER_G) * GRAVITY_MSS;

    AP::ins().handle_external(pkt);
}