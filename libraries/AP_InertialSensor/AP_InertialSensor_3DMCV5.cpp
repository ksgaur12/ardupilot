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

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_3DMCV5.h"

extern const AP_HAL::HAL &hal;

AP_InertialSensor_3DMCV5::AP_InertialSensor_3DMCV5(AP_InertialSensor &imu,
                                                   AP_HAL::UARTDriver *_port,
                                                   enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    ,port(_port)
    ,rotation(_rotation)

{
}

AP_InertialSensor_Backend *
AP_InertialSensor_3DMCV5::probe(AP_InertialSensor &imu,
                                AP_HAL::UARTDriver *port,
                                enum Rotation rotation)
{
    auto sensor = new AP_InertialSensor_3DMCV5(imu, port, rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


void AP_InertialSensor_3DMCV5::start()
{
    _gyro_instance = _imu.register_gyro(800, 0);
    _accel_instance = _imu.register_accel(800, 0);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_3DMCV5::_get_data, void));
}

void AP_InertialSensor_3DMCV5::calculate_checksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum_lsb = 0, checksum_msb = 0;
    while(len--){
        checksum_msb = checksum_msb + *data;
        checksum_lsb = checksum_lsb + checksum_msb;
        data++;
    }
    *data = checksum_msb;
    data++;
    *data = checksum_lsb;
}

bool AP_InertialSensor_3DMCV5::init()
{
    if(port == nullptr){
        return false;
    }

    //calculate checksum and configure data rate
    calculate_checksum((uint8_t *)&imu_config, sizeof(IMU_setting_packet)-2);
    port->write((uint8_t *)&imu_config, sizeof(IMU_setting_packet));

    //calculate checksum and set continuous mode
    calculate_checksum((uint8_t *)&continuous_mode_config, sizeof(mode_packet)-2);
    port->write((uint8_t *)&continuous_mode_config, sizeof(mode_packet));

    return true;
}

void AP_InertialSensor_3DMCV5::_get_data(){

    const uint32_t lock_key = 0x3256AB9F;

    port->lock_port(lock_key, lock_key);
    uint8_t check = (uint8_t)port->read_locked(lock_key);

    uint32_t numc = port->available();

    if(check==0x75){
        uint8_t data_buf[33];
        uint8_t checksum_lsb = 0x75;
        uint8_t checksum_msb = 0x75;

        for (uint32_t i=0 ; i<33 ; i++){
            uint8_t data = port->read_locked(lock_key);
            data_buf[i] = data;

            //hal.console->printf("%x\n",data);

            if(i<31){
                checksum_msb = checksum_msb + data;
                checksum_lsb = checksum_lsb + checksum_msb;
            }
            else if(i == 31 && checksum_msb != data){
                //hal.console->printf("checksum_msb failed\n");
                return;
            }
            else if(i == 32 && checksum_lsb != data){
                //hal.console->printf("checksum_lsb failed\n");
                return;
            }
        }

        MIP_packet *imu_data = (MIP_packet*)data_buf;

        int32_t accel_x = int32_t((imu_data->ax[0] << 24) | (imu_data->ax[1] << 16) | (imu_data->ax[2] << 8) | (imu_data->ax[3]));
        int32_t accel_y = int32_t((imu_data->ay[0] << 24) | (imu_data->ay[1] << 16) | (imu_data->ay[2] << 8) | (imu_data->ay[3]));
        int32_t accel_z = int32_t((imu_data->az[0] << 24) | (imu_data->az[1] << 16) | (imu_data->az[2] << 8) | (imu_data->az[3]));

        float accel_x1 = *(float *)&accel_x;
        float accel_y1 = *(float *)&accel_y;
        float accel_z1 = *(float *)&accel_z;

        Vector3f accel(accel_x1, accel_y1, accel_z1);

        accel = accel*GRAVITY_MSS;

        _rotate_and_correct_accel(_accel_instance, accel);
        _notify_new_accel_raw_sample(_accel_instance, accel);

        int32_t gyro_x = int32_t((imu_data->gx[0] << 24) | (imu_data->gx[1] << 16) | (imu_data->gx[2] << 8) | (imu_data->gx[3]));
        int32_t gyro_y = int32_t((imu_data->gy[0] << 24) | (imu_data->gy[1] << 16) | (imu_data->gy[2] << 8) | (imu_data->gy[3]));
        int32_t gyro_z = int32_t((imu_data->gz[0] << 24) | (imu_data->gz[1] << 16) | (imu_data->gz[2] << 8) | (imu_data->gz[3]));

        float gyro_x1 = *(float *)&gyro_x;
        float gyro_y1 = *(float *)&gyro_y;
        float gyro_z1 = *(float *)&gyro_z;

        Vector3f gyro(gyro_x1, gyro_y1, gyro_z1);

        _rotate_and_correct_gyro(_gyro_instance, gyro);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);
    }
    else{
        for(uint32_t i=0; i < numc; i++){
            check = port->read_locked(lock_key);
        }
    }
}

/*
  copy filtered data to the frontend
 */
bool AP_InertialSensor_3DMCV5::update(void)
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}
