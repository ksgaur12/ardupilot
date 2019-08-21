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
 *
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <stdint.h>
#include <npnt.h>

#define HEADER_REGION_LEN 16
#define KEY_REGION_LEN 488

#if ((HEADER_REGION_LEN + KEY_REGION_LEN) != 504) && ((KEY_REGION_LEN % 4) != 0) && ((KEY_REGION_LEN % 4) != 0)
#error "Invalid Security Key region Lengths"
#endif

#define AP_PUBLIC_KEY_FILE "/APM/pubkey.pem" 

#ifndef AP_NPNT_PERMART_FILE
#define AP_NPNT_PERMART_FILE "/APM/permissionArtifact.xml"
#endif

#define AP_NPNT_LOG_FILE "/APM/log_PA.json"
class AP_Security {

public:
    AP_Security();

    void update(bool arm_flag);

    // get singleton instance
    static AP_Security *get_singleton() {
        return _singleton;
    }


    struct coordinate{
    	int32_t lat;
        int32_t lon;
    };


    struct local_coord{
    	float x;
    	float y;
    };

    struct local_coord diff_coord;
    void init();

    bool permission_granted;

private:
    bool _check_npnt_permission();
    bool _check_fence_and_time();

    void _sign_permission_artifact();

    bool _polygon_outside(struct local_coord* V, struct local_coord P, int n);
    float longitude_scale(struct coordinate loc);
    void location_diff(struct coordinate loc1, struct coordinate loc2);

    static AP_Security *_singleton;
    npnt_s npnt_handle;

    int _log_fd;

    bool _logging;
    struct coordinate geo_fence[16];
    struct local_coord geo_fence_local[16];

    struct Location _global_position_current_loc;

};

namespace AP {
    AP_Security &security();
};
