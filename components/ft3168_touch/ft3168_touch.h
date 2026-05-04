#pragma once

#include "esphome/core/component.h"
#include "esphome/components/touchscreen/touchscreen.h"
#include <driver/i2c_master.h>
#include <cstdint>

namespace esphome {
namespace ft3168_touch {

// ============================================================================
// FT3168Touchscreen
// Custom touchscreen driver for the FT3168 IC on the Waveshare ESP32-S3-
// Touch-AMOLED-1.64 board. The board leaves TP_INT and TP_RST unconnected,
// so the chip can only be driven by polling — and it NACKs polls while idle.
// ESPHome's stock ft5x06 driver aborts on the first NACK, marking the
// component FAILED. Original PlatformIO firmware works around this by using
// IDF's I2C master with `disable_ack_check` set, treating NACKs as benign
// "no touch right now". This driver does the same.
//
// One consequence: this component owns its own IDF I2C master bus on the
// SDA/SCL pins it's given. Don't also configure ESPHome's `i2c:` block on
// the same pins — they'd collide on the same I2C peripheral.
// ============================================================================
class FT3168Touchscreen : public touchscreen::Touchscreen {
public:
    void set_sda_pin(int pin)            { sda_pin_ = pin; }
    void set_scl_pin(int pin)            { scl_pin_ = pin; }
    void set_address(uint8_t addr)       { address_ = addr; }
    void set_frequency_hz(uint32_t hz)   { frequency_hz_ = hz; }

    void setup() override;
    void update_touches() override;
    void dump_config() override;
    float get_setup_priority() const override { return setup_priority::DATA; }

private:
    int      sda_pin_{-1};
    int      scl_pin_{-1};
    uint8_t  address_{0x38};
    uint32_t frequency_hz_{300000};

    i2c_master_bus_handle_t bus_handle_{nullptr};
    i2c_master_dev_handle_t dev_handle_{nullptr};
    bool ready_{false};
};

}  // namespace ft3168_touch
}  // namespace esphome
