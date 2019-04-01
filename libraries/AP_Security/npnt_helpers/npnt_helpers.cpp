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

#include <npnt.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

void reset_sha1()
{
    hal.key_mgr->reset_sha1();
}

void update_sha1(const char* data, uint16_t data_len)
{
    hal.key_mgr->update_sha1(data, data_len);
}

void final_sha1(char* hash)
{
    hal.key_mgr->final_sha1(hash);
}

int8_t npnt_check_authenticity(npnt_s *handle, uint8_t* hashed_data, uint16_t hashed_data_len, const uint8_t* signature, uint16_t signature_len)
{
    return hal.key_mgr->verify_hash_with_server_pkey(hashed_data, hashed_data_len, signature, signature_len);
}
