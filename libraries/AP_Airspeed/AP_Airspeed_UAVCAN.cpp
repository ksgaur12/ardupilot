#include <AP_HAL/AP_HAL.h>

#include <GCS_MAVLink/GCS.h>

#if HAL_WITH_UAVCAN

#include "AP_Airspeed_UAVCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#if HAL_OS_POSIX_IO
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

extern const AP_HAL::HAL& hal;

#define debug_airspeed_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

/*
  constructor - registers instance at top Airspeed driver
 */
AP_Airspeed_UAVCAN::AP_Airspeed_UAVCAN(AP_Airspeed & airspeed, uint8_t instance) :
    AP_Airspeed_Backend(airspeed, instance)
{
    _sem_airspeed = hal.util->new_semaphore();
}

AP_Airspeed_UAVCAN::~AP_Airspeed_UAVCAN()
{
    if (!_initialized) {
        return;
    }

    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(_manager);
    if (ap_uavcan == nullptr) {
        return;
    }

    ap_uavcan->remove_airspeed_listener(this);
    delete _sem_airspeed;

    debug_airspeed_uavcan(2, "AP_Airspeed_UAVCAN destructed\n\r");
}

AP_Airspeed_Backend *AP_Airspeed_UAVCAN::probe(AP_Airspeed &airspeed, uint8_t instance)
{

    if (AP_BoardConfig_CAN::get_can_num_ifaces() == 0) {
        return nullptr;
    }


    AP_Airspeed_UAVCAN *sensor;
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }

        uint8_t freeairspeed = ap_uavcan->find_smallest_free_airspeed_node();
        if (freeairspeed == UINT8_MAX) {
            continue;
        }

        sensor = new AP_Airspeed_UAVCAN(airspeed, instance);
        if (sensor->register_uavcan_airspeed(i, freeairspeed)) {
            sensor->_instance = instance;
            debug_airspeed_uavcan(2, "AP_Airspeed_UAVCAN probed, drv: %d, node: %d\n\r", i, freeairspeed);
            return sensor;
        } else {
            delete sensor;
        }
    }

    return nullptr;
}

void AP_Airspeed_UAVCAN::handle_airspeed_msg(float pressure, float temperature)
{

    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _pressure_sum += pressure;
        _temperature_sum += temperature - C_TO_KELVIN;
        _press_count++;
        _temp_count++;
        _last_sample_time_ms = AP_HAL::millis();
        sem->give();
    }
}

//initialise the sensor
bool AP_Airspeed_UAVCAN::init()
{
    if(_initialized){
        return true;
    }

    return false;
}


// return the current differential_pressure in Pascal
bool AP_Airspeed_UAVCAN::get_differential_pressure(float &pressure)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    if (_sem_airspeed->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_press_count > 0) {
            pressure = _pressure_sum / _press_count;
            _press_count = 0;
            _pressure_sum = 0;
        }
        _sem_airspeed->give();

    }
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_UAVCAN::get_temperature(float &temperature)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    if (_sem_airspeed->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_temp_count > 0) {
            temperature = _temperature_sum / _temp_count;
            _temp_count = 0;
            _temperature_sum = 0;
        }
        _sem_airspeed->give();
    }
    return true;
}

bool AP_Airspeed_UAVCAN::register_uavcan_airspeed(uint8_t mgr, uint8_t node)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return false;
    }
    _manager = mgr;

    if (ap_uavcan->register_airspeed_listener_to_node(this, node)) {
        debug_airspeed_uavcan(2, "AP_Airspeed_UAVCAN loaded\n\r");

        _initialized = true;

        return true;
    }

    return false;
}

#endif // HAL_WITH_UAVCAN

