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

#include <AP_HAL/AP_HAL.h>
#ifdef HAL_IS_REGISTERED_FLIGHT_MODULE
#include "AP_Security.h"
#include <AP_Math/crc.h>
#if HAL_OS_POSIX_IO
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <dirent.h>
#if defined(__APPLE__) && defined(__MACH__)
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif
#endif

#if HAL_OS_FATFS_IO
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

//Setup Security at the initialisation step
AP_Security::AP_Security()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many security modules");
        return;
    }
    _singleton = this;
}

void AP_Security::init() {
    //Initialise NPNT Handler
    npnt_init_handle(&npnt_handle);
    if (_check_npnt_permission()) {
        permission_granted = true;
        hal.console->printf("Permission Granted!");
    }
}

//Load permissionArtefacts
bool AP_Security::_check_npnt_permission()
{
    struct stat st;
    if (::stat(AP_NPNT_PERMART_FILE, &st) != 0) {
        printf("Unable to find Permission Artifact\n", strerror(errno));
        return false;
    }
    int fd = ::open(AP_NPNT_PERMART_FILE, O_RDONLY|O_CLOEXEC);
    uint8_t* permart = (uint8_t*)malloc(st.st_size + 1);
    if (!permart) {
        return false;
    }
    
    uint32_t permart_len;
    permart_len = ::read(fd, permart, st.st_size);
    if ((off_t)permart_len != st.st_size) {
        free(permart);
        return false;
    }
    //append with NULL character
    permart[st.st_size] = '\0';
    permart_len++;
    if (npnt_set_permart(&npnt_handle, permart, permart_len, 0) < 0) {
        return false;
    }
    return true;
}

// singleton instance
AP_Security *AP_Security::_singleton;

namespace AP {

AP_Security &security()
{
    return *AP_Security::get_singleton();
}

}

#endif //HAL_IS_REGISTERED_FLIGHT_MODULE
