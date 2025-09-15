#include "lprot.h"
#include "LSPDriver.h"
#include "AP_Math/crc.h"

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

    uart->begin(115200, 512, 512);

    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&LProt::update_thread, void), "LProt", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0);
}

void LProt::update_thread(void) {
    uint8_t buf[128];

    while(true) {
        
        uint32_t av = uart->available();
        if(av > 0) {
            av = av > 128 : 128 : av;
            uart->read(buf, av);
            handleRx(buf, av);
        }
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