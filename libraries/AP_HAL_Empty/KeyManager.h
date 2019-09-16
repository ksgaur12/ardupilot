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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once

class Empty::KeyManager : public AP_HAL::KeyManager {
public:
    KeyManager() {}
    void init() {}
    void reset_sha1() {}
    void update_sha1(const char* data, uint16_t data_len) {}
    void final_sha1(char* hash) {}
    void reset_sha256() {}
    void update_sha256(const char* data, uint16_t data_len) {}
    void final_sha256(char* hash) {}
    //void load_server_pubkey() {}
    int verify_hash_with_server_pkey(uint8_t* hashed_data, uint16_t hashed_data_len, const uint8_t* signature, uint16_t signature_len) { return -1; }
    int sign_hash_with_key(uint8_t* hashed_data, uint16_t hashed_data_len, uint8_t* signature);
};
