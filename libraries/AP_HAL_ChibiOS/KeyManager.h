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

#include "AP_HAL_ChibiOS.h"
#ifdef HAL_IS_REGISTERED_FLIGHT_MODULE
#include <wolfssl/options.h>
#include <wolfssl/wolfcrypt/rsa.h>
#include <wolfssl/wolfcrypt/sha.h>
#include <wolfssl/wolfcrypt/sha256.h>
#include <wolfssl/wolfcrypt/signature.h>
#include <wolfssl/wolfcrypt/asn.h>
#include <wolfssl/wolfcrypt/error-crypt.h>

class ChibiOS::KeyManager : public AP_HAL::KeyManager {
public:
    KeyManager() {}
    void init() override;
    void reset_sha1() override;
    void update_sha1(const char* data, uint16_t data_len) override;
    void final_sha1(char* hash) override;
    void reset_sha256() override;
    void update_sha256(const char* data, uint16_t data_len) override;
    void final_sha256(char* hash) override;
    void load_server_pubkey() override;
    int verify_hash_with_server_pkey(uint8_t* hashed_data, uint16_t hashed_data_len, const uint8_t* signature, uint16_t signature_len) override;
    int sign_hash_with_key(uint8_t* hashed_data, uint16_t hashed_data_len, uint8_t* signature);

private:
    static void _generate_private_key(void* _key_mgr);
    void _save_public_key();
    bool _check_and_initialise_private_key();
    RsaKey ap_key;
    RsaKey server_pubkey;
    wc_Sha sha;
    wc_Sha256 sha256;
    bool _server_key_loaded = false;
};

#endif
