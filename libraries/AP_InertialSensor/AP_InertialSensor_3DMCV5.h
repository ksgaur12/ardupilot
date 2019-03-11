/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_3DMCV5 : public AP_InertialSensor_Backend {
    struct MIP_packet{
        uint8_t head;
        uint8_t desc;
        uint8_t len;
        uint8_t acc_len;
        uint8_t acc_desc;
        uint8_t ax[4];
        uint8_t ay[4];
        uint8_t az[4];
        uint8_t gyro_len;
        uint8_t gyro_desc;
        uint8_t gx[4];
        uint8_t gy[4];
        uint8_t gz[4];
        uint8_t check1;
        uint8_t check2;
    };
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
            AP_HAL::UARTDriver *_port,
            enum Rotation rotation = ROTATION_NONE);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_3DMCV5(AP_InertialSensor &imu,
            AP_HAL::UARTDriver *_port,
            enum Rotation rotation = ROTATION_NONE);

    void _get_data();

    //    /*
    //      initialise driver
    //     */
    bool init();

    uint8_t _accel_instance;
    uint8_t _gyro_instance;

    enum Rotation rotation;

    HAL_Semaphore _sem;


protected:
    AP_HAL::UARTDriver *port;           ///< UART we are attached to
};

