#pragma once

#include "AP_Airspeed_Backend.h"
#include <AP_UAVCAN/AP_UAVCAN.h>

class AP_Airspeed_UAVCAN : public AP_Airspeed_Backend {
public:
    AP_Airspeed_UAVCAN(AP_Airspeed & airspeed, uint8_t instance);
    ~AP_Airspeed_UAVCAN() override;

    static AP_Airspeed_Backend *probe(AP_Airspeed &airspeed, uint8_t instance);
//
//    void update() override;
//
    // This method is called from UAVCAN thread
    virtual void handle_airspeed_msg(float pressure, float temperature) override;

    bool register_uavcan_airspeed(uint8_t mgr, uint8_t node);


    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;
private:
    uint8_t _instance;
    float _pressure_sum;
    float _temperature_sum;
    uint64_t _last_timestamp;
    uint8_t _manager;

    uint32_t _last_sample_time_ms;
    uint32_t _press_count;
    uint32_t _temp_count;

    bool _initialized;

    AP_HAL::Semaphore *_sem_airspeed;
};
