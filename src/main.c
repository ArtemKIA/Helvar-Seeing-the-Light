#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <tinycrypt/aes.h>
#include <tinycrypt/constants.h>
#include <tinycrypt/ccm_mode.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/random/random.h>
#include <zephyr/bluetooth/controller.h>
#include <zephyr/device.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


//Bluetooth settings 
#define DATA_LEN 22
#define NONCE_LEN 13
#define AUTH_TAG_LEN 12
#define ADV_DATA_LEN 26
#define RSP_DATA_LEN 29
#define SLEEP_TIME_MS 2000
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ENCRYPTION false
// Multiplexer i2c address
#define PCA9548A_ADDR 0x70

// LTR303 specific definitions
#define LTR303_I2C_ADDRESS 0x29
#define LTR303_ALS_CONTR 0x80
#define LTR303_ALS_MEAS_RATE 0x85
#define LTR303_ALS_DATA_CH1_0 0x88
#define LTR303_ALS_DATA_CH1_1 0x89
#define LTR303_ALS_DATA_CH0_0 0x8A
#define LTR303_ALS_DATA_CH0_1 0x8B

// TCS34725 specific definitions

//#define TCS34725_ADDRESS 0x29 // I2C address of the TCS34725 sensor

#define COMMAND_BIT 0x80
#define ENABLE 0x00
#define ATIME 0x01
#define WTIME 0x03
#define CONFIG 0x0D
#define CONTROL 0x0F
#define CLEAR_L 0x14
#define CLEAR_H 0x15
#define RED_L 0x16
#define RED_H 0x17
#define GREEN_L 0x18
#define GREEN_H 0x19
#define BLUE_L 0x1A
#define BLUE_H 0x1B

#ifndef M_PI // PI number
#define M_PI 3.14159265358979323846
#endif
//Settings for LTR-303
#define ALS_GAIN 1 
#define ALS_INT 1  // Assuming  integration time


#define I2C_NODE_RGB DT_NODELABEL(tcs347225) // RGB addres
#define I2C_NODE DT_NODELABEL(i2c1)          // MUX addres

static const struct device *i2c_dev;
//Scan for Multiplexer
void select_mux_channel(uint8_t channel) {
    int ret = i2c_reg_write_byte(i2c_dev, PCA9548A_ADDR, 0x04, 1 << channel);
    if (ret != 0) {
        printk("Failed to select I2C multiplexer channel %d. Error code: %d\n", channel, ret);
    }
}

//TCS347225 sensor data calculation 
float cct_calculation(uint16_t red, uint16_t green, uint16_t blue)
{
    float X, Y, Z; // RGB to XYZ correlation
    float xc, yc;  // Chromaticity co-ordinates
    float n;       // McCamy's formula
    float cct;

    if (red == 0 && green == 0 && blue == 0)
    {
        return 0;
    }

    // Map RGB values to their XYZ counterparts.
    // Based on 6500K fluorescent, 3000K fluorescent
    // and 60W incandescent values for a wide range.
    // Note: Y = Illuminance or lux
    X = (-0.14282F * red) + (1.54924F * green) + (-0.95641F * blue);
    Y = (-0.32466F * red) + (1.57837F * green) + (-0.73191F * blue);
    Z = (-0.68202F * red) + (0.77073F * green) + (0.56332F * blue);

    // Calculate the chromaticity co-ordinates
    xc = X / (X + Y + Z);
    yc = Y / (X + Y + Z);

    // Use McCamy's formula to determine the CCT
    n = (xc - 0.3320F) / (0.1858F - yc);

    // Calculate the final CCT
    cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

    return cct;
}

//LTR-303 sensor data calculation 
float calculate_lux_ltr303(uint16_t ch0, uint16_t ch1)
{
    float ratio = (float)ch1 / (ch0 + ch1);
    float als_lux = 0;

    if (ratio < 0.45)
    {
        als_lux = (1.7743 * ch0 + 1.1059 * ch1) / ALS_GAIN / ALS_INT;
    }
    else if (ratio < 0.64)
    {
        als_lux = (4.2785 * ch0 - 1.9548 * ch1) / ALS_GAIN / ALS_INT;
    }
    else if (ratio < 0.85)
    {
        als_lux = (0.5926 * ch0 + 0.1185 * ch1) / ALS_GAIN / ALS_INT;
    }
    else
    {
        als_lux = 0;
    }
    return als_lux;
}


void configure_ltr303_sensor(uint8_t channel)
{
    uint8_t config_contr[2] = {LTR303_ALS_CONTR, 0x01};         // Set gain to x1
    uint8_t config_meas_rate[2] = {LTR303_ALS_MEAS_RATE, 0x01}; // Set integration time to 300ms
    int ret;

    ret = i2c_reg_write_byte(i2c_dev, PCA9548A_ADDR, 0x04, 1 << channel);
    if (ret != 0)
    {
        printk("Failed to select I2C multiplexer channel %d. Error code: %d\n", channel, ret);
        return;
    }

    ret = i2c_write(i2c_dev, config_contr, sizeof(config_contr), LTR303_I2C_ADDRESS);
    if (ret != 0)
    {
        printk("Failed to write ALS_CONTR to I2C device on channel %d. Error code: %d\n", channel, ret);
        return;
    }
    printk("Configured ALS_CONTR for channel %d\n", channel);

    ret = i2c_write(i2c_dev, config_meas_rate, sizeof(config_meas_rate), LTR303_I2C_ADDRESS);
    if (ret != 0)
    {
        printk("Failed to write ALS_MEAS_RATE to I2C device on channel %d. Error code: %d\n", channel, ret);
        return;
    }
    printk("Configured ALS_MEAS_RATE for channel %d\n", channel);
}

void read_ltr303_sensor_data(uint8_t channel, uint16_t *ch0, uint16_t *ch1)
{
    uint8_t ch1_data_reading[2] = {0};
    uint8_t ch0_data_reading[2] = {0};
    int ret;

    ret = i2c_reg_write_byte(i2c_dev, PCA9548A_ADDR, 0x04, 1 << channel);
    if (ret != 0)
    {
        printk("Failed to select I2C multiplexer channel %d. Error code: %d\n", channel, ret);
        return;
    }

    ret = i2c_reg_read_byte(i2c_dev, LTR303_I2C_ADDRESS, LTR303_ALS_DATA_CH1_0, &ch1_data_reading[0]);
    if (ret != 0)
    {
        printk("Failed to read CH1_0 from I2C device on channel %d. Error code: %d\n", channel, ret);
        return;
    }

    ret = i2c_reg_read_byte(i2c_dev, LTR303_I2C_ADDRESS, LTR303_ALS_DATA_CH1_1, &ch1_data_reading[1]);
    if (ret != 0)
    {
        printk("Failed to read CH1_1 from I2C device on channel %d. Error code: %d\n", channel, ret);
        return;
    }

    ret = i2c_reg_read_byte(i2c_dev, LTR303_I2C_ADDRESS, LTR303_ALS_DATA_CH0_0, &ch0_data_reading[0]);
    if (ret != 0)
    {
        printk("Failed to read CH0_0 from I2C device on channel %d. Error code: %d\n", channel, ret);
        return;
    }

    ret = i2c_reg_read_byte(i2c_dev, LTR303_I2C_ADDRESS, LTR303_ALS_DATA_CH0_1, &ch0_data_reading[1]);
    if (ret != 0)
    {
        printk("Failed to read CH0_1 from I2C device on channel %d. Error code: %d\n", channel, ret);
        return;
    }

    *ch1 = (ch1_data_reading[1] << 8) | ch1_data_reading[0];
    *ch0 = (ch0_data_reading[1] << 8) | ch0_data_reading[0];
}

void calculate_angles(float lux0, float lux1, float lux6, float lux7, float lux8, float *azimuth, float *polar)
{
    // Create vectors based on the lux values
    float v1_x = lux0;
    float v1_y = lux1;
    float v1_z = lux8;
    float v2_x = lux6;
    float v2_y = lux7;
    float v2_z = 0; 

    // Calculate vector differences
    float diff_x = v1_x - v2_x;
    float diff_y = v1_y - v2_y;
    float diff_z = v1_z - v2_z;

    // Calculate azimuth angle in degrees
    *azimuth = atan2(diff_y, diff_x) * 180 / M_PI;

    // Ensure azimuth is in the range of 0 to 360 degrees
    if (*azimuth < 0)
    {
        *azimuth += 360;
    }

    // Calculate polar angle in degrees
    *polar = atan2(diff_z, sqrt(diff_x * diff_x + diff_y * diff_y)) * 180 / M_PI;
}

float calculate_average_lux(float lux0, float lux1, float lux6, float lux7, float lux8)
{
    return (lux0 + lux1 + lux6 + lux7 + lux8) / 5.0;
}

uint8_t adv_data_array[ADV_DATA_LEN];
uint8_t rsp_data_array[RSP_DATA_LEN];

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, adv_data_array, sizeof(adv_data_array))
};

static struct bt_data scan_rsp[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, rsp_data_array, sizeof(rsp_data_array)) 
};


// AES Key (must be 16 bytes for AES-128)
static const uint8_t aes_key[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                     0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

void generate_encrypted_data(uint8_t *data, size_t len, uint8_t *nonce, uint8_t *auth_tag) {
    struct tc_aes_key_sched_struct aes;
    struct tc_ccm_mode_struct ccm;
    uint8_t ciphertext[DATA_LEN + AUTH_TAG_LEN];
    
    // Initialize AES context
    if (tc_aes128_set_encrypt_key(&aes, aes_key) != TC_CRYPTO_SUCCESS) {
        printk("Failed to set AES key\n");
        return;
    }

    // Initialize CCM context
    if (tc_ccm_config(&ccm, &aes, nonce, NONCE_LEN, AUTH_TAG_LEN) != TC_CRYPTO_SUCCESS) {
        printk("Failed to configure CCM\n");
        return;
    }

    // Encrypt and authenticate data
    if (tc_ccm_generation_encryption(ciphertext, sizeof(ciphertext),
                                     NULL, 0, // No associated data
                                     data, len, &ccm) != TC_CRYPTO_SUCCESS) {
        printk("Encryption and tag generation failed\n");
        return;
    }

    // Copy the ciphertext back to the original buffer
    memcpy(data, ciphertext, len);
    // Copy the authentication tag to the auth_tag buffer
    memcpy(auth_tag, ciphertext + len, AUTH_TAG_LEN);
}

void update_advertisement_data(int id, float azimuth, float polar, float average_lux, bool isNaturalLight) {
    static uint8_t adv_data[ADV_DATA_LEN];
    static uint8_t scan_rsp_data[RSP_DATA_LEN];
    uint8_t nonce[NONCE_LEN];
    uint8_t auth_tag[AUTH_TAG_LEN];

    // Round the values
    int rounded_azimuth     = (int)round(azimuth);      // use 2 bytes
    int rounded_polar       = (int)round(polar);        // use 1 byte
    int rounded_average_lux = (int)round(average_lux);  // use 2 bytes

    // Fill in advertisement data
    adv_data[0] = 0x59; // Nordic
    adv_data[1] = 0x00; // Nordic
    adv_data[2] = id & 0xFF; // ID
    adv_data[3] = (id >> 8) & 0xFF; //ID
    adv_data[4] = rounded_azimuth & 0xFF;               // Azimuth
    adv_data[5] = (rounded_azimuth >> 8) & 0xFF;        // Azimuth
    adv_data[6] = rounded_polar & 0xFF;                 // Polar
    adv_data[7] = rounded_average_lux & 0xFF;           // Avg Lux
    adv_data[8] = (rounded_average_lux >> 8) & 0xFF;    // Avg Lux
    adv_data[9] = isNaturalLight & 0xFF;                // isNaturalLight
    for (int i = 10; i < ADV_DATA_LEN; ++i) {
        adv_data[i] = 0; 
    }

    

    // Fill in scan response data
    scan_rsp_data[0] = 0x59; // Nordic
    scan_rsp_data[1] = 0x00; // Nordic
    scan_rsp_data[2] = id & 0xFF;
    scan_rsp_data[3] = (id >> 8) & 0xFF;

    if (ENCRYPTION)
    {
        // Generate a random nonce
        if (sys_csrand_get(nonce, sizeof(nonce)) < 0) {
        printk("Failed to generate random nonce\n");
        return;
        }

        // printk("Generated Nonce: ");
        // for (int i = 0; i < NONCE_LEN; i++) {
        //     printk("%02x ", nonce[i]);
        // }
        // printk("\n");

        // Encrypt the data
        generate_encrypted_data(&adv_data[4], DATA_LEN, nonce, auth_tag);

        // Copy nonce and auth tag to scan response
        memcpy(&scan_rsp_data[4], nonce, NONCE_LEN);
        memcpy(&scan_rsp_data[17], auth_tag, AUTH_TAG_LEN); // +13
    }
    else
    {
        for (int i = 4; i < RSP_DATA_LEN; ++i) {
        scan_rsp_data[i] = 0; 
    }
    }
    
    // Update advertisement and scan response data
    ad[1].data = adv_data;
    ad[1].data_len = sizeof(adv_data);
    scan_rsp[0].data = scan_rsp_data;
    scan_rsp[0].data_len = sizeof(scan_rsp_data);

    // Debug information for the data being set
    // printk("Updated advertisement data:\n");
    // for (int i = 0; i < sizeof(adv_data); i++) {
    //     printk("%02x ", adv_data[i]);
    // }
    // printk("\n");

    // printk("Updated scan response data:\n");
    // for (int i = 0; i < sizeof(scan_rsp_data); i++) {
    //     printk("%02x ", scan_rsp_data[i]);
    // }
    // printk("\n");
}

int main(void)
{
    int err;
    int id = 0;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return -2;
    }
    printk("Bluetooth initialized\n");

    err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), scan_rsp, ARRAY_SIZE(scan_rsp));

    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return -2;
    }

    printk("Advertising successfully started\n");

    // RGB sensor
    int ret;
    uint16_t clear, red, green, blue;
    uint16_t red_norm, green_norm, blue_norm;

    static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE_RGB);
    if (!device_is_ready(dev_i2c.bus))
    {
        printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
        return -1;
    }

    // Setup the sensor by writing the necessary configuration values
    uint8_t config_enable[2] = {COMMAND_BIT | ENABLE, 0x03}; // Power on and enable ADC
    ret = i2c_write_dt(&dev_i2c, config_enable, sizeof(config_enable));
    if (ret != 0)
    {
        printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr, config_enable[0]);
        return -1;
    }

    uint8_t config_control[2] = {COMMAND_BIT | CONTROL, 0x02}; // Set gain to 16X
    ret = i2c_write_dt(&dev_i2c, config_control, sizeof(config_control));
    if (ret != 0)
    {
        printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr, config_control[0]);
        return -1;
    }

    uint8_t config_atime[2] = {COMMAND_BIT | ATIME, 0xC0}; // Integration time 2.4 ms (0xFF)
    ret = i2c_write_dt(&dev_i2c, config_atime, sizeof(config_atime));
    if (ret != 0)
    {
        printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr, config_atime[0]);
        return -1;
    }

    uint8_t config_wtime[2] = {COMMAND_BIT | WTIME, 0x00};
    ret = i2c_write_dt(&dev_i2c, config_atime, sizeof(config_wtime));
    if (ret != 0)
    {
        printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr, config_atime[0]);
        return -1;
    }

    // MUX and Light sensor
    uint16_t ch0_0, ch1_0, ch0_1, ch1_1, ch0, ch1;
    float als_lux0, als_lux1, als_lux6, als_lux7, als_lux8;
    float azimuth, polar, average_lux;

    i2c_dev = DEVICE_DT_GET(I2C_NODE);

    // Configure LTR303 sensors (connected to channels 0, 1, 6, 7)
    configure_ltr303_sensor(0);
    configure_ltr303_sensor(1);
    configure_ltr303_sensor(6);
    configure_ltr303_sensor(7);
    configure_ltr303_sensor(2);
    k_sleep(K_MSEC(100)); // Give time for the sensor to stabilize

    while (1)
    {
        uint8_t rgb_readings[8] = {0};
        uint8_t sensor_regs[8] = {
            COMMAND_BIT | CLEAR_L, COMMAND_BIT | CLEAR_H,
            COMMAND_BIT | RED_L, COMMAND_BIT | RED_H,
            COMMAND_BIT | GREEN_L, COMMAND_BIT | GREEN_H,
            COMMAND_BIT | BLUE_L, COMMAND_BIT | BLUE_H};

        for (int i = 0; i < 8; i++)
        {
            ret = i2c_write_read_dt(&dev_i2c, &sensor_regs[i], 1, &rgb_readings[i], 1);
            if (ret != 0)
            {
                printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr, sensor_regs[i]);
            }
        }

        clear = (rgb_readings[1] << 8) | rgb_readings[0];
        red = (rgb_readings[3] << 8) | rgb_readings[2];
        green = (rgb_readings[5] << 8) | rgb_readings[4];
        blue = (rgb_readings[7] << 8) | rgb_readings[6];

        uint16_t max_value = clear;

        red_norm = (float)red / max_value * 256;
        green_norm = (float)green / max_value * 256;
        blue_norm = (float)blue / max_value * 256;

        // printk("Clear: %u, Red: %u, Green: %u, Blue: %u\n", clear, red_norm, green_norm, blue_norm);
        float diffRG = fabs(red_norm - green_norm);
        float diffGB = fabs(green_norm - blue_norm);
        float diffBR = fabs(blue_norm - red_norm);
        float cct = cct_calculation(red, green, blue);
        float rgbThreshold = 10.0;
        float rbrThreshold = 2.0;
        float meanCCTMin = 5000.0;
        float meanCCTMax = 6500.0;

        bool isNaturalLight = (diffGB < rgbThreshold && (diffBR > diffGB) && (diffBR > diffGB) && (diffRG - diffBR <= 3 || diffRG - diffBR <= -3));

        // const char *source = isNaturalLight ? "natural" : "unnatural";

        // printk("Clear: %u, Red: %u, Green: %u, Blue: %u, DiffRG: %f, DiffGB: %f, DiffBR: %f, CCT: %f, Source: %s\n", clear, red_norm, green_norm, blue_norm, diffRG, diffGB, diffBR, cct, source);

        // Read and calculate lux for LTR303 sensors
        read_ltr303_sensor_data(0, &ch0_0, &ch1_0);
        als_lux0 = calculate_lux_ltr303(ch0_0, ch1_0);

        read_ltr303_sensor_data(1, &ch0_1, &ch1_1);
        als_lux1 = calculate_lux_ltr303(ch0_1, ch1_1);

        read_ltr303_sensor_data(6, &ch0_0, &ch1_0);
        als_lux6 = calculate_lux_ltr303(ch0_0, ch1_0);

        read_ltr303_sensor_data(7, &ch0_1, &ch1_1);
        als_lux7 = calculate_lux_ltr303(ch0_1, ch1_1);

        read_ltr303_sensor_data(2, &ch0_1, &ch1_1);
        als_lux8 = calculate_lux_ltr303(ch0_1, ch1_1);

        // Calculate angles based on lux values from all sensors
        calculate_angles(als_lux0, als_lux1, als_lux6, als_lux7, als_lux8, &azimuth, &polar);

        // Calculate average lux
        average_lux = calculate_average_lux(als_lux0, als_lux1, als_lux6, als_lux7, als_lux8);

        const char *source = isNaturalLight ? "Natural" : "Artificial";

        // Print results
        // printk("Sensor 0: Current Lux: %.2f\n", als_lux0);
        // printk("Sensor 1: Current Lux: %.2f\n", als_lux1);
        // printk("Sensor 6: Current Lux: %.2f\n", als_lux6);
        // printk("Sensor 7: Current Lux: %.2f\n", als_lux7);
        // printk("Sensor 2: Current Lux: %.2f\n", als_lux8);

        // printk("Clear: %u, Red: %u, Green: %u, Blue: %u\n", clear, red_norm, green_norm, blue_norm);
        // printk("Clear: %u, Red: %u, Green: %u, Blue: %u, DiffRG: %f, DiffGB: %f, DiffBR: %f, CCT: %f, Source: %s\n", clear, red_norm, green_norm, blue_norm, diffRG, diffGB, diffBR, cct, source);

        
        printk("\n\nID: %d\n", id);
        printk("Azimuth: %.2f degrees\nPolar: %.2f degrees\n", azimuth, polar);
        printk("Average Lux: %.2f\n", average_lux);
        printk("Light Source: %s\n", source);

        update_advertisement_data(id, azimuth, polar, average_lux, isNaturalLight);
        bt_le_adv_update_data(ad, ARRAY_SIZE(ad), scan_rsp, ARRAY_SIZE(scan_rsp));

        if (id < 65535)
        {
            id++;
        }
        else
        {
            id = 0;
        }
        // Sleep for ms
        k_msleep(SLEEP_TIME_MS);
    }
}

