#include "ft3168_touch.h"
#include "esphome/core/log.h"
#include "esp_log.h"

namespace esphome {
namespace ft3168_touch {

static const char* const TAG = "ft3168";

// FT3168 register layout: poll register 0x02 and read 5 bytes.
//   buf[0] = touch_count (low nibble)
//   buf[1..2] = X coord (12-bit)
//   buf[3..4] = Y coord (12-bit)
static constexpr uint8_t REG_NUM_TOUCHES = 0x02;
static constexpr int     I2C_TIMEOUT_MS  = 5;

void FT3168Touchscreen::setup() {
    ESP_LOGCONFIG(TAG, "Setting up FT3168 touchscreen on SDA=%d SCL=%d addr=0x%02X",
                  sda_pin_, scl_pin_, address_);

    // Suppress IDF I2C error logs — the chip NACKs idle polls and we don't
    // want the log to fill up with `i2c.master` complaints.
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    esp_log_level_set("i2c-master", ESP_LOG_NONE);

    // Create our own IDF I2C master bus. Do NOT also configure ESPHome's
    // `i2c:` block on the same pins/peripheral — they'd collide.
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port                  = I2C_NUM_0;
    bus_config.sda_io_num                = static_cast<gpio_num_t>(sda_pin_);
    bus_config.scl_io_num                = static_cast<gpio_num_t>(scl_pin_);
    bus_config.clk_source                = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt         = 7;
    bus_config.intr_priority             = 0;
    bus_config.trans_queue_depth         = 0;
    bus_config.flags.enable_internal_pullup = 1;

    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        this->mark_failed();
        return;
    }

    // Attach FT3168 with ACK checking disabled — idle NACKs become benign.
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length          = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address           = address_;
    dev_config.scl_speed_hz             = frequency_hz_;
    dev_config.scl_wait_us              = 0;
    dev_config.flags.disable_ack_check  = 1;

    err = i2c_master_bus_add_device(bus_handle_, &dev_config, &dev_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to attach FT3168: %s", esp_err_to_name(err));
        this->mark_failed();
        return;
    }

    // Default raw coordinate range — matches FT3168's 12-bit output.
    this->x_raw_max_ = 4095;
    this->y_raw_max_ = 4095;

    ready_ = true;
    ESP_LOGI(TAG, "FT3168 initialised (NACKs treated as no-touch)");
}

void FT3168Touchscreen::update_touches() {
    if (!ready_ || dev_handle_ == nullptr) return;

    uint8_t reg = REG_NUM_TOUCHES;
    uint8_t buf[5] = {0};
    esp_err_t err = i2c_master_transmit_receive(
        dev_handle_, &reg, sizeof(reg), buf, sizeof(buf),
        I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        // NACK / no-touch path. Don't log — happens every poll while idle.
        return;
    }

    uint8_t touches = buf[0] & 0x0F;
    if (touches == 0) return;

    uint16_t x = (uint16_t)(((buf[1] & 0x0F) << 8) | buf[2]);
    uint16_t y = (uint16_t)(((buf[3] & 0x0F) << 8) | buf[4]);
    this->add_raw_touch_position_(0, x, y);
}

void FT3168Touchscreen::dump_config() {
    ESP_LOGCONFIG(TAG, "FT3168 Touchscreen:");
    ESP_LOGCONFIG(TAG, "  SDA pin: GPIO%d", sda_pin_);
    ESP_LOGCONFIG(TAG, "  SCL pin: GPIO%d", scl_pin_);
    ESP_LOGCONFIG(TAG, "  Address: 0x%02X", address_);
    ESP_LOGCONFIG(TAG, "  Frequency: %u Hz", frequency_hz_);
    LOG_UPDATE_INTERVAL(this);
}

}  // namespace ft3168_touch
}  // namespace esphome
