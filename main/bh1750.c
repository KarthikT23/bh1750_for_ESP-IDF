#include <malloc.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "bh1750.h"

#define TAG "BH1750"

#define I2C_TRANSFER_TIMEOUT  50 /* (milliseconds) give up on i2c transaction after this timeout */
#define I2C_SPEED             400000 /* hz */

#define OPCODE_HIGH  0x0
#define OPCODE_HIGH2 0x1
#define OPCODE_LOW   0x3

#define OPCODE_CONT 0x10
#define OPCODE_OT   0x20

#define OPCODE_POWER_DOWN 0x00
#define OPCODE_POWER_ON   0x01
#define OPCODE_MT_HI      0x40
#define OPCODE_MT_LO      0x60

typedef struct
{
    i2c_master_bus_handle_t bus;
    bool bus_created;
    i2c_master_dev_handle_t device;
    bh1750_mode_t mode;
    bh1750_resolution_t resolution;
} bh1750_context_t;

static bool send_command(bh1750_context_t *ctx, uint8_t cmd)
{
    return (i2c_master_transmit(ctx->device, &cmd, 1, -1) == ESP_OK);
}

static bool read_data(bh1750_context_t *ctx, uint8_t *data, uint8_t length)
{
    return (i2c_master_receive(ctx->device, data, length, -1) == ESP_OK);
}

bh1750_t bh1750_init(i2c_lowlevel_config *config, uint8_t i2c_address, bh1750_mode_t mode, bh1750_resolution_t resolution)
{
    bh1750_context_t *ctx;
    bool success = false;

    if (i2c_address != BH1750_ADDR_LO && i2c_address != BH1750_ADDR_HI)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return NULL;
    }

    ctx = (bh1750_context_t *) malloc(sizeof(*ctx));
    if (NULL == ctx)
        return NULL;

    memset(ctx, 0, sizeof(*ctx));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = I2C_SPEED,
    };

    if (NULL == config->bus)
    {
        i2c_master_bus_config_t bus_cfg = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = config->port,
            .sda_io_num = config->pin_sda,
            .scl_io_num = config->pin_scl,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        if (i2c_new_master_bus(&bus_cfg, &ctx->bus) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize I2C bus");
            free(ctx);
            return NULL;
        }
        ctx->bus_created = true;
    }
    else
    {
        ctx->bus = *config->bus;
        ctx->bus_created = false;
    }

    if (i2c_master_bus_add_device(ctx->bus, &dev_cfg, &ctx->device) != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C device initialization failed");
        if (ctx->bus_created)
            i2c_del_master_bus(ctx->bus);
        free(ctx);
        return NULL;
    }

    ctx->mode = mode;
    ctx->resolution = resolution;

    // Power on the device
    if (!send_command(ctx, OPCODE_POWER_ON))
    {
        ESP_LOGE(TAG, "Failed to power on device");
    }
    else
    {
        // Setup measurement mode and resolution
        uint8_t opcode = mode == BH1750_MODE_CONTINUOUS ? OPCODE_CONT : OPCODE_OT;
        switch (resolution)
        {
            case BH1750_RES_LOW:  opcode |= OPCODE_LOW;   break;
            case BH1750_RES_HIGH: opcode |= OPCODE_HIGH;  break;
            default:              opcode |= OPCODE_HIGH2; break;
        }

        if (send_command(ctx, opcode))
        {
            ESP_LOGD(TAG, "BH1750 setup successful (ADDR = 0x%02x, CMD = 0x%02x)", i2c_address, opcode);
            success = true;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to setup device");
        }
    }

    if (!success)
    {
        if (ctx->bus_created)
            i2c_del_master_bus(ctx->bus);
        free(ctx);
        ctx = NULL;
    }

    return ctx;
}

bool bh1750_free(bh1750_t bh1750)
{
    bh1750_context_t *ctx = (bh1750_context_t *) bh1750;
    if (NULL == ctx)
        return false;

    if (ctx->bus_created)
        i2c_del_master_bus(ctx->bus);
    free(ctx);
    return true;
}

bool bh1750_power_down(bh1750_t bh1750)
{
    bh1750_context_t *ctx = (bh1750_context_t *) bh1750;
    if (NULL == ctx)
        return false;

    return send_command(ctx, OPCODE_POWER_DOWN);
}

bool bh1750_power_on(bh1750_t bh1750)
{
    bh1750_context_t *ctx = (bh1750_context_t *) bh1750;
    if (NULL == ctx)
        return false;

    return send_command(ctx, OPCODE_POWER_ON);
}

bool bh1750_set_measurement_time(bh1750_t bh1750, uint8_t time)
{
    bh1750_context_t *ctx = (bh1750_context_t *) bh1750;
    if (NULL == ctx)
        return false;

    // Send high bits
    if (!send_command(ctx, OPCODE_MT_HI | (time >> 5)))
        return false;

    // Send low bits
    return send_command(ctx, OPCODE_MT_LO | (time & 0x1f));
}

bool bh1750_read(bh1750_t bh1750, uint16_t *lux_level)
{
    bh1750_context_t *ctx = (bh1750_context_t *) bh1750;
    if (NULL == ctx || NULL == lux_level)
        return false;

    uint8_t buf[2];

    if (!read_data(ctx, buf, 2))
        return false;

    uint16_t level = (buf[0] << 8) | buf[1];
    *lux_level = (level * 10) / 12; // convert to LUX

    ESP_LOGD(TAG, "Raw: %d, LUX: %d", level, *lux_level);

    return true;
}