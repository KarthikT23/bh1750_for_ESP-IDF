#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "bh1750.h"

#define TAG "MAIN"

// I2C Configuration
#define I2C_MASTER_SDA_IO    21        // GPIO pin for SDA
#define I2C_MASTER_SCL_IO    22        // GPIO pin for SCL
#define I2C_MASTER_NUM       I2C_NUM_0 // I2C port number

// BH1750 Configuration
#define BH1750_I2C_ADDR      BH1750_ADDR_HI  // Use BH1750_ADDR_HI if ADDR pin is high

static void bh1750_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting BH1750 task");

    // Configure I2C
    i2c_lowlevel_config i2c_config = {
        .bus = NULL,  // Let the driver create the bus
        .port = I2C_MASTER_NUM,
        .pin_sda = I2C_MASTER_SDA_IO,
        .pin_scl = I2C_MASTER_SCL_IO,
    };

    // Initialize BH1750
    bh1750_t bh1750 = bh1750_init(&i2c_config, BH1750_I2C_ADDR, 
                                  BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH);
    
    if (bh1750 == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialize BH1750");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "BH1750 initialized successfully");

    // Wait a bit for the sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(200));

    uint16_t lux_level;
    int read_count = 0;

    while (1)
    {
        if (bh1750_read(bh1750, &lux_level))
        {
            ESP_LOGI(TAG, "Light level: %d lux", lux_level);
            read_count++;
            
            // Demo: Power management every 10 readings
            if (read_count % 10 == 0)
            {
                ESP_LOGI(TAG, "Demonstrating power management...");
                
                // Power down
                if (bh1750_power_down(bh1750))
                {
                    ESP_LOGI(TAG, "Device powered down");
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to power down device");
                }
                
                vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
                
                // Power on
                if (bh1750_power_on(bh1750))
                {
                    ESP_LOGI(TAG, "Device powered on");
                    
                    // Reconfigure after power on
                    uint8_t opcode = BH1750_MODE_CONTINUOUS == BH1750_MODE_CONTINUOUS ? 0x10 : 0x20;
                    opcode |= 0x0; // HIGH resolution
                    
                    // Wait for measurement to be ready
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to power on device");
                }
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read from BH1750");
        }

        // Wait 2 seconds between readings
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Cleanup (this won't be reached in this example)
    bh1750_free(bh1750);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "BH1750 Light Sensor Example");
    ESP_LOGI(TAG, "Using I2C SDA: GPIO%d, SCL: GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    // Create the BH1750 task
    xTaskCreate(bh1750_task, "bh1750_task", 4096, NULL, 5, NULL);
}