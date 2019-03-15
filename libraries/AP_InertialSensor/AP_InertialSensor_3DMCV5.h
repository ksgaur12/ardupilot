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

    struct IMU_setting_packet{
        uint8_t sync1           = 0x75;
        uint8_t sync2           = 0x65;
        uint8_t desc_set        = 0x0C;
        uint8_t pay_len         = 0x0A;
        uint8_t field_len       = 0x0A;
        uint8_t field_desc      = 0x08;
        uint8_t func            = 0x01;
        uint8_t desc_count      = 0x02;
        uint8_t desc1           = 0x04;
        uint8_t desc1_rate_msb  = 0x00;
        uint8_t desc1_rate_lsb  = 0x05;
        uint8_t desc2           = 0x05;
        uint8_t desc2_rate_msb  = 0x00;
        uint8_t desc2_rate_lsb  = 0x05;
        uint8_t check_msb;
        uint8_t check_lsb;
    };

    struct mode_packet{
        uint8_t sync1           = 0x75;
        uint8_t sync2           = 0x65;
        uint8_t desc_set        = 0x0C;
        uint8_t pay_len         = 0x05;
        uint8_t field_len       = 0x05;
        uint8_t field_desc      = 0x11;
        uint8_t func            = 0x01;
        uint8_t device          = 0x01;
        uint8_t stream          = 0x01;
        uint8_t check_msb;
        uint8_t check_lsb;
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

    //parse continuous stream of packet to accel and gyro data
    void _get_data();

    //calculate fletcher's checksum
    void calculate_checksum(uint8_t *data, uint8_t len);
    //    /*
    //      initialise driver
    //     */
    bool init();

    uint8_t _accel_instance;
    uint8_t _gyro_instance;

    enum Rotation rotation;
    IMU_setting_packet imu_config;
    mode_packet continuous_mode_config;

protected:
    AP_HAL::UARTDriver *port;           ///< UART we are attached to
};

