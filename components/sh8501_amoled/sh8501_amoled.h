#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/display/display_buffer.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>

namespace esphome {
namespace sh8501_amoled {

// ============================================================================
// SH8501AmoledDisplay
// IDF SPI master (QSPI mode) driver for the CO5300 / SH8501B AMOLED controller
// used on the Waveshare ESP32-S3 1.64" AMOLED module (280x456 px).
//
// Implements ESPHome's display::DisplayBuffer interface so that the lvgl:
// component can use it as its render target.
// ============================================================================
class SH8501AmoledDisplay : public display::DisplayBuffer {
public:
    // ── ESPHome config setters ────────────────────────────────────────────
    void set_cs_pin(InternalGPIOPin* p)    { cs_pin_    = p; }
    void set_sck_pin(InternalGPIOPin* p)   { sck_pin_   = p; }
    void set_d0_pin(InternalGPIOPin* p)    { d0_pin_    = p; }
    void set_d1_pin(InternalGPIOPin* p)    { d1_pin_    = p; }
    void set_d2_pin(InternalGPIOPin* p)    { d2_pin_    = p; }
    void set_d3_pin(InternalGPIOPin* p)    { d3_pin_    = p; }
    void set_reset_pin(InternalGPIOPin* p) { reset_pin_ = p; }
    void set_width(uint16_t w)             { width_  = w; }
    void set_height(uint16_t h)            { height_ = h; }

    // ── Component lifecycle ───────────────────────────────────────────────
    void setup() override;
    void dump_config() override;
    float get_setup_priority() const override { return setup_priority::HARDWARE; }

    // ── DisplayBuffer overrides ───────────────────────────────────────────
    int get_width_internal() override  { return (int)width_; }
    int get_height_internal() override { return (int)height_; }
    display::DisplayType get_display_type() override {
        return display::DisplayType::DISPLAY_TYPE_COLOR;
    }

    void update() override;

    // Called by LVGL flush callback — writes a pixel rectangle to the display
    void draw_pixels_at(int x_start, int y_start, int w, int h,
                        const uint8_t* data, display::ColorOrder color_order,
                        display::ColorBitness color_bitness,
                        bool big_endian, int x_offset, int y_offset,
                        int x_pad) override;

protected:
    void draw_absolute_pixel_internal(int x, int y, Color color) override;

private:
    InternalGPIOPin* cs_pin_{nullptr};
    InternalGPIOPin* sck_pin_{nullptr};
    InternalGPIOPin* d0_pin_{nullptr};
    InternalGPIOPin* d1_pin_{nullptr};
    InternalGPIOPin* d2_pin_{nullptr};
    InternalGPIOPin* d3_pin_{nullptr};
    InternalGPIOPin* reset_pin_{nullptr};

    uint16_t width_{280};
    uint16_t height_{456};

    spi_device_handle_t spi_dev_{nullptr};
    bool initialized_{false};

    void hardware_reset();
    void init_display_registers();
    void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

    // QSPI command/data helpers
    void write_cmd(uint8_t cmd);
    void write_cmd_data(uint8_t cmd, const uint8_t* data, size_t len);
    void write_pixels(const uint8_t* data, size_t len);
};

}  // namespace sh8501_amoled
}  // namespace esphome
