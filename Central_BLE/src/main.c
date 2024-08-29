#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/printk.h>
#include <tinycrypt/aes.h>
#include <tinycrypt/constants.h>
#include <tinycrypt/ccm_mode.h>

// Target device's MAC address
static const bt_addr_le_t target_addr = {
    .type = BT_ADDR_LE_RANDOM,
    .a.val = { 0x1B, 0x8E, 0xAB, 0x28, 0xE2, 0xE5 } // Peripheral's MAC Address in reversed order (little endian)
};

// Buffers to store the advertisement and scan response data
static uint8_t adv_data_buf[31];
static uint8_t scan_rsp_data_buf[31];
static uint8_t adv_data_len = 0;
static uint8_t scan_rsp_data_len = 0;

// AES Key (must be 16 bytes for AES-128), change if needed
static const uint8_t aes_key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                     0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

// Decryption function
void decrypt_data(uint8_t *encrypted_data, size_t encrypted_len,
                  uint8_t *nonce, uint8_t *auth_tag, uint8_t *decrypted_data)
{
    struct tc_aes_key_sched_struct aes;
    struct tc_ccm_mode_struct ccm;

    // Initialize AES context
    if (tc_aes128_set_decrypt_key(&aes, aes_key) != TC_CRYPTO_SUCCESS) {
        printk("Failed to set AES key\n");
        return;
    }

    // Initialize CCM context
    if (tc_ccm_config(&ccm, &aes, nonce, 13, 12) != TC_CRYPTO_SUCCESS) {
        printk("Failed to configure CCM\n");
        return;
    }
    uint8_t combined_data[encrypted_len + 12];
    memcpy(combined_data, encrypted_data, encrypted_len);
    memcpy(combined_data + encrypted_len, auth_tag, 12);

    // Decrypt and verify data
    if (tc_ccm_decryption_verification(decrypted_data, encrypted_len,
                                       NULL, 0, // No associated data
                                       combined_data, encrypted_len + 12, &ccm) != TC_CRYPTO_SUCCESS) {
        printk("Decryption and verification failed\n");
        return;
    }
}

int prev;

static bool device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    if (bt_addr_le_cmp(addr, BT_ADDR_LE_ANY) == 0)
    {
        printk("No address.\n");
        return false;
    }

    if (bt_addr_le_cmp(addr, &target_addr) == 0)
    {
        if (type == BT_GAP_ADV_TYPE_ADV_SCAN_IND)
        {
            // Update the advertisement data buffer
            adv_data_len = ad->len;
            memcpy(adv_data_buf, ad->data, adv_data_len);
        }
        else if (type == BT_GAP_ADV_TYPE_SCAN_RSP)
        {
            // Update the scan response data buffer
            scan_rsp_data_len = ad->len;
            memcpy(scan_rsp_data_buf, ad->data, scan_rsp_data_len);

            // If the data is encrypted, the sum of element [6] to [30] of the scan response will be > 0; otherwise it will be 0
            uint16_t sum = 0;
            for (int i = 6; i < 31; i++) {
                sum += scan_rsp_data_buf[i];
            }
            
            // Decrypt data if scan response data is available and the data is encrypted
            if (scan_rsp_data_len >= 30 && sum > 0)
            {
                uint8_t encrypted_data[22];
                uint8_t nonce[13];
                uint8_t auth_tag[12];
                uint8_t decrypted_data[22];

                // Fill encrypted data, nonce, and authentication tag buffers
                memcpy(encrypted_data, &adv_data_buf[9], 22);
                memcpy(nonce, &scan_rsp_data_buf[6], 13);
                memcpy(auth_tag, &scan_rsp_data_buf[19], 12);

                // Perform decryption
                decrypt_data(encrypted_data, 22, nonce, auth_tag, decrypted_data);

                uint16_t id = (adv_data_buf[8] << 8) | adv_data_buf[7]; // ID
                uint16_t integer1 = (decrypted_data[1] << 8) | decrypted_data[0]; // Combining elements 0 and 1 (little-endian)
                uint8_t integer2 = decrypted_data[2]; // Element 2 as an integer (no endian conversion needed)
                uint16_t integer3 = (decrypted_data[4] << 8) | decrypted_data[3]; // Combining elements 3 and 4 (little-endian)
                bool boolean_value = (decrypted_data[5] != 0); // Element 5 as a boolean (no endian conversion needed)

                // Displaying the extracted values
                if( id != prev )
                {
                    printf("\n\nID: %u\n", id);
                    printf("Azimuth: %u\n", integer1);
                    printf("Polar: %u\n", integer2);
                    printf("Average Lux: %u\n", integer3);
                    printf("Light Source: %s\n", boolean_value ? "Natural" : "Artificial");
                    prev = id;
                }
            }
            else // The data is not encrypted
            {
                uint16_t id = (adv_data_buf[8] << 8) | adv_data_buf[7];
                uint16_t integer1 = (adv_data_buf[10] << 8) | adv_data_buf[9];
                uint8_t integer2 = adv_data_buf[11]; 
                uint16_t integer3 = (adv_data_buf[13] << 8) | adv_data_buf[12];
                bool boolean_value = (adv_data_buf[14] != 0);

            if( id != prev )
                {
                    printf("\n\nID: %u\n", id);
                    printf("Azimuth: %u\n", integer1);
                    printf("Polar: %u\n", integer2);
                    printf("Average Lux: %u\n", integer3);
                    printf("Light Source: %s\n", boolean_value ? "Natural" : "Artificial");
                    prev = id;
                }
            }
        }
        return true; // Target device found
    }

    return false;
}

int main(void)
{
    int err;

    // Initialize Bluetooth
    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }

    printk("Bluetooth initialized\n");

    // Define scanning parameters for active scanning
    struct bt_le_scan_param scan_param = {
        .type       = BT_HCI_LE_SCAN_ACTIVE, // active scan
        .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE, // filter duplicate
        .interval   = 1280,  // formula (interval in ms / 0.625)
        .window     = 150,    // 30 ms, same formula
    };

    // Start scanning with defined parameters
    err = bt_le_scan_start(&scan_param, device_found);
    if (err)
    {
        printk("Scanning failed to start (err %d)\n", err);
        return err;
    }

    printk("Scanning started\n");

    return 0;
}
