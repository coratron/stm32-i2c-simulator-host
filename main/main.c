#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           3    // GPIO 9 for SCL
#define I2C_MASTER_SDA_IO           2    // GPIO 8 for SDA
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          40000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

static const char *TAG = "i2c-tester";

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void i2c_scanner_task(void *arg) {
    ESP_LOGI(TAG, "Starting I2C scanner...");
    
    while (1) {
        printf("\n--- I2C Bus Scan ---\n");
        printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
        
        for (int i = 0; i < 128; i += 16) {
            printf("%02X: ", i);
            for (int j = 0; j < 16; j++) {
                uint8_t address = i + j;
                
                if (address < 0x08 || address > 0x77) {
                    printf("   ");
                    continue;
                }
                
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
                i2c_master_stop(cmd);
                
                esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
                i2c_cmd_link_delete(cmd);
                
                if (ret == ESP_OK) {
                    printf("%02X ", address);
                } else {
                    printf("-- ");
                }
            }
            printf("\n");
        }
        
        printf("\nExpected devices:\n");
        printf("  0x20: MCP23017 #1 (I/O Expander)\n");
        printf("  0x21: MCP23017 #2 (I/O Expander)\n");
        printf("  0x48: ADS1015 (ADC)\n");
        printf("  0x68: DS3231 (RTC)\n\n");
        
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void test_mcp23017(uint8_t addr) {
    ESP_LOGI(TAG, "Testing MCP23017 at 0x%02X", addr);
    
    // Test writing to IODIRA register (0x00)
    uint8_t write_data[2] = {0x00, 0x00}; // Set all pins as outputs
    i2c_master_write_to_device(I2C_MASTER_NUM, addr, write_data, sizeof(write_data), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    // Read back IODIRA
    uint8_t reg = 0x00;
    uint8_t read_data;
    i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, &read_data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "  IODIRA: 0x%02X (expected 0x00)", read_data);
    
    // Write to GPIOA
    write_data[0] = 0x12;  // GPIOA register
    write_data[1] = 0xAA;  // Test pattern
    i2c_master_write_to_device(I2C_MASTER_NUM, addr, write_data, sizeof(write_data), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    // Read back GPIOA
    reg = 0x12;
    i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, &read_data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "  GPIOA: 0x%02X (expected 0xAA)", read_data);
}

static void test_ads1015(void) {
    ESP_LOGI(TAG, "Testing ADS1015 at 0x48");
    
    // Read config register
    uint8_t reg = 0x01;  // Config register pointer
    uint8_t read_data[2];
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, 0x48, &reg, 1, read_data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret == ESP_OK) {
        uint16_t config = (read_data[0] << 8) | read_data[1];
        ESP_LOGI(TAG, "  Config: 0x%04X (default 0x8583)", config);
        
        // Start a conversion
        uint8_t write_data[3] = {0x01, 0xC5, 0x83}; // Start single conversion
        i2c_master_write_to_device(I2C_MASTER_NUM, 0x48, write_data, sizeof(write_data), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        
        vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for conversion
        
        // Read conversion result
        reg = 0x00;  // Conversion register
        i2c_master_write_read_device(I2C_MASTER_NUM, 0x48, &reg, 1, read_data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        uint16_t conversion = (read_data[0] << 8) | read_data[1];
        ESP_LOGI(TAG, "  Conversion: 0x%04X", conversion);
    }
}

static void test_ds3231(void) {
    ESP_LOGI(TAG, "Testing DS3231 at 0x68");
    
    // Read time registers
    uint8_t reg = 0x00;  // Start at seconds register
    uint8_t time_data[7];
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, 0x68, &reg, 1, time_data, 7, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "  Time: %02X:%02X:%02X", time_data[2], time_data[1], time_data[0]);
        ESP_LOGI(TAG, "  Date: %02X/%02X/%02X (YY/MM/DD)", time_data[6], time_data[5], time_data[4]);
        
        // Read temperature
        reg = 0x11;  // Temperature MSB
        uint8_t temp_data[2];
        i2c_master_write_read_device(I2C_MASTER_NUM, 0x68, &reg, 1, temp_data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        
        int8_t temp_msb = (int8_t)temp_data[0];
        uint8_t temp_lsb = temp_data[1];
        float temperature = temp_msb + (temp_lsb >> 6) * 0.25;
        ESP_LOGI(TAG, "  Temperature: %.2f°C", temperature);
    }
}

static void device_test_task(void *arg) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait for scanner to run first
    
    while (1) {
        ESP_LOGI(TAG, "\n=== Running Device Tests ===");
        
        test_mcp23017(0x20);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        test_mcp23017(0x21);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        test_ads1015();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        test_ds3231();
        
        ESP_LOGI(TAG, "=== Tests Complete ===\n");
        
        vTaskDelay(10000 / portTICK_PERIOD_MS);  // Run tests every 10 seconds
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    ESP_LOGI(TAG, "Connect STM32 I2C pins to ESP32-C3:");
    ESP_LOGI(TAG, "  STM32 PB6 (SCL) -> ESP32-C3 GPIO9");
    ESP_LOGI(TAG, "  STM32 PB7 (SDA) -> ESP32-C3 GPIO8");
    ESP_LOGI(TAG, "  Connect GND between both boards");
    
    xTaskCreate(i2c_scanner_task, "i2c_scanner", 4096, NULL, 10, NULL);
    xTaskCreate(device_test_task, "device_test", 4096, NULL, 9, NULL);
}