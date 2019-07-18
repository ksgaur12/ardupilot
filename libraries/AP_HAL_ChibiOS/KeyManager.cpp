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

#include <AP_HAL/AP_HAL.h>

//Check if the build is for Registered Flight Module
#ifdef HAL_IS_REGISTERED_FLIGHT_MODULE

#include "KeyManager.h"

#include <AP_ROMFS/AP_ROMFS.h>
#include "hwdef/common/flash.h"
#include <AP_Math/crc.h>
#include "hwdef/common/stm32_util.h"
#include "Scheduler.h"
#include <npnt_internal.h>
#include <GCS_MAVLink/GCS.h>

#ifndef AP_SERVER_PUBLIC_KEY_FILE
#define AP_SERVER_PUBLIC_KEY_FILE "server_pubkey.der"
#endif

#ifndef AP_SELF_PUBLIC_KEY_FILE
#define AP_SELF_PUBLIC_KEY_FILE "/APM/self_pubkey.der"
#endif

extern const AP_HAL::HAL& hal;
using namespace ChibiOS;

void KeyManager::init() {
    //check if we already generated key during last run
    if (!_check_and_initialise_private_key()) {
        //Generate new key in case of failure
        //Run private keygen thread
        thread_create_alloc(THD_WORKING_AREA_SIZE(1024),
                                         "KEYGEN",
                                         APM_IO_PRIORITY-1,           /* Initial priority.    */
                                         KeyManager::_generate_private_key,    /* Thread function.     */
                                         this);                     /* Thread parameter.    */
    }
}

bool KeyManager::_check_and_initialise_private_key()
{
    uint32_t dersize, read_crc;
    uint32_t keyder_crc = 0xACDC;
    const uint8_t *der;
    const uint8_t *flash_data;
    word32 idx = 0;
    int ret;
    size_t base_address = stm32_flash_getpageaddr(KEY_FLASH_PAGE);
    //read dersize
    flash_data = ((const uint8_t *)base_address);
    memcpy(&dersize, flash_data, sizeof(dersize));
    //check sanity for 2048bit RSA Key's dersize
    if (dersize < 1100 || dersize > 1200) {
        gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Invalid Key Length in Flash.\n");
        hal.scheduler->delay(1);
        return false;
    }
    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Found Private Key in Flash.\n");
    //Initialise Rsa Key structure
    ret = wc_InitRsaKey(&ap_key, NULL);
    if (ret != 0) {
        return false;
    }
    //Read keyder_crc
    base_address += sizeof(dersize);
    flash_data = ((const uint8_t *)base_address);
    memcpy(&read_crc, flash_data, sizeof(read_crc));
    //Decode RSA Key in der format
    base_address += sizeof(keyder_crc);
    der = ((const uint8_t*)base_address);
    //check key is good
    keyder_crc = crc_crc32(keyder_crc, der, dersize);
    if (keyder_crc != read_crc) {
        gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Invalid Key CRC!\n");
        hal.scheduler->delay(1);
        return false;
    }
    ret = wc_RsaPrivateKeyDecode(der, &idx, &ap_key, dersize);
    if (ret != 0) {
        gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Failed to load Key from Flash.\n");
        hal.scheduler->delay(1);
        return false;
    }
    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Successfully Loaded Private Key.\n");

    _save_public_key();
    return true;
}

void KeyManager::_save_public_key()
{
    uint32_t dersize = 600;
    //Generate Public Key File
    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Extracting Public Key.\n");
    uint8_t *publickey_der = new uint8_t[dersize];
    if (publickey_der == nullptr) {
        AP_HAL::panic("KeyManager: Failed to Allocate buffer for Public Key.");
        return;
    }

    dersize = wc_RsaKeyToPublicDer(&ap_key, publickey_der, dersize);
    //Save public key to file
    int pubkey_fd = open(AP_SELF_PUBLIC_KEY_FILE, O_WRONLY|O_TRUNC|O_CREAT);
    if (pubkey_fd < 0) {
        AP_HAL::panic("KeyManager: Failed to create file for Public Key Storage.");
        return;
    }

    if (write(pubkey_fd, publickey_der, dersize) < 0) {
        AP_HAL::panic("KeyManager: Failed to write public key");
        return;
    }

    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Saved Public Key to SDCard.\n");
    close(pubkey_fd);
    delete[] publickey_der;
}

void KeyManager::_generate_private_key(void* _key_mgr)
{
    WC_RNG rng;
    uint8_t *der;
    int ret;
    uint32_t dersize = 1200;
    uint32_t keyder_crc = 0xACDC;
    KeyManager* key_mgr = (KeyManager*)_key_mgr;
    //Initialise Random Number Generator
    ret = wc_InitRng(&rng);
    if (ret != 0) {
        AP_HAL::panic("KeyManager: Failed to Initialize Random Number Generator");
        return;
    }
    //Initialise Rsa Key structure
    ret = wc_InitRsaKey(&key_mgr->ap_key, NULL);
    if (ret != 0) {
        AP_HAL::panic("KeyManager: Failed to Initialize RSA Key");
        return;
    }
    //Generate RSA Key
    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Generating Private Key\n");
    hal.scheduler->delay(1);
    ret = wc_MakeRsaKey(&key_mgr->ap_key, 2048, 65537, &rng);
    if (ret != 0) {
        AP_HAL::panic("KeyManager: Failed to Generate RSA Key Pair");
        return;
    }
    //Generate Key in DER format
    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Converting Raw Private Key to DER Format.\n");
    hal.scheduler->delay(1);
    der = new uint8_t[dersize];
    if (der == nullptr) {
        AP_HAL::panic("KeyManager: Failed to Allocate DER buffer.");
        return;
    }
    dersize = wc_RsaKeyToDer(&key_mgr->ap_key, der, dersize);
    if (ret != 0) {
        AP_HAL::panic("KeyManager: Failed to convert RSA Key to DER");
        delete[] der;
        return;
    }
    //Erase Key Flash Sector
    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Erasing Key Flash Page.\n");
    while (!stm32_flash_erasepage(KEY_FLASH_PAGE)) {
        hal.scheduler->delay(1);
    }

    //Generate CRC hash or der key for sanity check
    keyder_crc = crc_crc32(keyder_crc, der, dersize);
    //Write the Key to the Flash Memory
    size_t base_address = stm32_flash_getpageaddr(KEY_FLASH_PAGE);
    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Writing Key Length to Flash.\n");
    while (!stm32_flash_write(base_address, &dersize, sizeof(dersize))) {
        hal.scheduler->delay(1);
    }
    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Writing Key CRC to Flash.\n");
    base_address += sizeof(dersize);
    while (!stm32_flash_write(base_address, &keyder_crc, sizeof(keyder_crc))) {
        hal.scheduler->delay(1);
    }
    //ensure we setup 32bit writes only
    uint32_t written_length = 0;
    gcs().send_statustext(MAV_SEVERITY_ALERT, 0xFF, "KeyManager: Writing Key to Flash.\n");
    base_address += sizeof(keyder_crc);
    while(written_length < dersize) {
        while (!stm32_flash_write(base_address + written_length, &der[written_length], sizeof(uint32_t))) {
            hal.scheduler->delay(1);
        }
        hal.scheduler->delay(1);
        written_length += sizeof(uint32_t);
    }
    key_mgr->_save_public_key();
    delete[] der;
}


void KeyManager::reset_sha1()
{
    wc_ShaFree(&sha);
    wc_InitSha(&sha);
}

void KeyManager::update_sha1(const char* data, uint16_t data_len)
{
    wc_ShaUpdate(&sha, (uint8_t*)data, data_len);
}

void KeyManager::final_sha1(char* hash)
{
    wc_ShaFinal(&sha, (unsigned char*)hash);
}

//Reads the embedded server public key and loads into the raw structure
void KeyManager::load_server_pubkey()
{
    word32 idx = 0;
    uint32_t server_pubkey_dersize = 0;
    uint8_t *server_pubkey_der = AP_ROMFS::find_decompress(AP_SERVER_PUBLIC_KEY_FILE, server_pubkey_dersize);
    if (server_pubkey_der == NULL) {
        AP_HAL::panic("Failed to find Server Public Key!");;
    }
    int ret = wc_InitRsaKey(&server_pubkey, 0);
    if (ret == 0) {
        ret = wc_RsaPublicKeyDecode(server_pubkey_der, &idx, &server_pubkey, server_pubkey_dersize);
    }
    if (ret != 0) {
        AP_HAL::panic("Failed to load Server Public Key!");
    }
    _server_key_loaded = true;
}

//Verifies if the given hash is authentic wrt server's public key
//return from this method is similar as in Openssl's Verify Key method
int KeyManager::verify_hash_with_server_pkey(uint8_t* hashed_data, uint16_t hashed_data_len, const uint8_t* signature, uint16_t signature_len)
{
    int ret = 0;
    if (_server_key_loaded) {
        return -1;
    }
    byte *digest_buf;
    word32 digest_len;
    /* Check arguments */
    if (hashed_data == NULL || hashed_data_len <= 0 || signature == NULL || signature_len <= 0) {
        return -1;
    }

    /* Validate signature len (1 to max is okay) */
    if ((int)signature_len > wc_SignatureGetSize(WC_SIGNATURE_TYPE_RSA_W_ENC, &server_pubkey, sizeof(server_pubkey))) {
        return -1;
    }

    //create der encode from the raw data digest we recieved
    ret = wc_HashGetOID(WC_HASH_TYPE_SHA);
    if (ret > 0) {
        int oid = ret;

        /* Allocate buffer for hash and max DER encoded */
        digest_len = signature_len + MAX_DER_DIGEST_SZ;
        digest_buf = (byte*)malloc(digest_len);
        if (digest_buf) {
            ret = wc_EncodeSignature(digest_buf, hashed_data, hashed_data_len, oid);
            if (ret > 0) {
                digest_len = ret;
            }
            else {
                free(digest_buf);
            }
        }
        else {
            return -1;
        }
    } else {
        return -1;
    }

    word32 plain_len = digest_len;
    byte *plain_data;

    /* Make sure the plain text output is at least key size */
    if (plain_len < signature_len) {
        plain_len = signature_len;
    }
    plain_data = (byte*)malloc(plain_len);
    if (plain_data) {
        /* Perform verification of signature using provided RSA key */
        do {
        if (ret >= 0)
            ret = wc_RsaSSL_Verify(signature, signature_len, plain_data,
                plain_len, &server_pubkey);
        } while (ret == WC_PENDING_E);
        if (ret >= 0) {
            if ((word32)ret == digest_len &&
                    memcmp(plain_data, digest_buf, digest_len) == 0) {
                ret = 1; /* Success */
            }
            else {
                ret = 0;
            }
        }
        free(plain_data);
    }

    return ret;
}
#endif //HAL_IS_REGISTERED_FLIGHT_MODULE

