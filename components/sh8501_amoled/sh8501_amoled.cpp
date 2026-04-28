#include "sh8501_amoled.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>

namespace esphome {
namespace sh8501_amoled {

static const char* const TAG = "sh8501_amoled";

// ── CO5300 / SH8501 QSPI command set ─────────────────────────────────────────
// In QSPI mode the CO5300 uses a 3-byte header: CMD_NOP byte, then command,
// then data on all four data lines.  The SPI frame is MODE 0, MSB first.
#define CO5300_CMD_NOP         0x00
#define CO5300_CMD_SWRESET     0x01
#define CO5300_CMD_SLPOUT      0x11
#define CO5300_CMD_NORON       0x13
#define CO5300_CMD_INVOFF      0x20
#define CO5300_CMD_INVON       0x21
#define CO5300_CMD_ALLPOFF     0x22
#define CO5300_CMD_ALLPON      0x23
#define CO5300_CMD_GAMSET      0x26
#define CO5300_CMD_DISPOFF     0x28
#define CO5300_CMD_DISPON      0x29
#define CO5300_CMD_CASET       0x2A
#define CO5300_CMD_RASET       0x2B
#define CO5300_CMD_RAMWR       0x2C
#define CO5300_CMD_COLMOD      0x3A
#define CO5300_CMD_MADCTL      0x36
#define CO5300_CMD_WRAM        0x2C
#define CO5300_CMD_WRAMCONT    0x3C
#define CO5300_CMD_TEOFF       0x34
#define CO5300_CMD_TEON        0x35

// SPI flags for QSPI (4-line) data phase
#define SPI_TRANS_USE_QUAD_LINES  (1<<24)

// ── Component lifecycle ───────────────────────────────────────────────────────

void SH8501AmoledDisplay::setup() {
    ESP_LOGI(TAG, "Setting up SH8501 AMOLED %dx%d", width_, height_);

    // Configure reset pin
    if (reset_pin_) {
        reset_pin_->setup();
        reset_pin_->pin_mode(gpio::FLAG_OUTPUT);
    }

    // ── Configure IDF SPI master for QSPI ────────────────────────────────
    spi_bus_config_t bus_cfg = {};
    bus_cfg.sclk_io_num     = sck_pin_->get_pin();
    bus_cfg.data0_io_num    = d0_pin_->get_pin();
    bus_cfg.data1_io_num    = d1_pin_->get_pin();
    bus_cfg.data2_io_num    = d2_pin_->get_pin();
    bus_cfg.data3_io_num    = d3_pin_->get_pin();
    bus_cfg.mosi_io_num     = -1;
    bus_cfg.miso_io_num     = -1;
    bus_cfg.quadwp_io_num   = -1;
    bus_cfg.quadhd_io_num   = -1;
    bus_cfg.max_transfer_sz = width_ * height_ * 2 + 64;
    bus_cfg.flags           = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        this->mark_failed();
        return;
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = 80 * 1000 * 1000;  // 80 MHz
    dev_cfg.mode           = 0;
    dev_cfg.spics_io_num   = cs_pin_->get_pin();
    dev_cfg.queue_size     = 7;
    dev_cfg.command_bits   = 8;   // 1-byte command header
    dev_cfg.address_bits   = 8;   // 1-byte sub-address / direction
    dev_cfg.flags          = SPI_DEVICE_HALFDUPLEX;

    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        this->mark_failed();
        return;
    }

    hardware_reset();
    init_display_registers();
    initialized_ = true;
    ESP_LOGI(TAG, "SH8501 ready");
}

void SH8501AmoledDisplay::dump_config() {
    ESP_LOGCONFIG(TAG, "SH8501 AMOLED Display:");
    ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", width_, height_);
    LOG_PIN("  CS:    ", cs_pin_);
    LOG_PIN("  SCK:   ", sck_pin_);
    LOG_PIN("  RESET: ", reset_pin_);
}

// ── Reset sequence ─────────────────────────────────────────────────────────

void SH8501AmoledDisplay::hardware_reset() {
    if (reset_pin_) {
        reset_pin_->digital_write(true);
        delay(10);
        reset_pin_->digital_write(false);
        delay(20);
        reset_pin_->digital_write(true);
        delay(120);
    }
}

// ── CO5300 register init sequence (from Waveshare ESP32-S3-LCD-1.64 BSP) ────

void SH8501AmoledDisplay::init_display_registers() {
    // Software reset
    write_cmd(CO5300_CMD_SWRESET);
    delay(120);

    // Unlock CMD2 registers
    {
        const uint8_t d[] = {0x06, 0x08};
        write_cmd_data(0xFD, d, sizeof(d));
    }

    // Page 1 selection
    {
        const uint8_t d[] = {0x01};
        write_cmd_data(0xFE, d, sizeof(d));
    }

    // Panel settings (trimmed to commonly safe defaults for the CO5300)
    {
        const uint8_t d[] = {0x01};
        write_cmd_data(0x06, d, sizeof(d));  // PSEL
    }
    {
        const uint8_t d[] = {0x10};
        write_cmd_data(0x0E, d, sizeof(d));  // AVDD
    }
    {
        const uint8_t d[] = {0x10};
        write_cmd_data(0x0F, d, sizeof(d));  // AVEE
    }
    {
        const uint8_t d[] = {0x73};
        write_cmd_data(0x10, d, sizeof(d));  // VCI charge pump
    }
    {
        const uint8_t d[] = {0x08};
        write_cmd_data(0x11, d, sizeof(d));
    }
    {
        const uint8_t d[] = {0x00};
        write_cmd_data(0x12, d, sizeof(d));
    }
    {
        const uint8_t d[] = {0xB8};
        write_cmd_data(0x13, d, sizeof(d));
    }

    // Return to page 0
    {
        const uint8_t d[] = {0x00};
        write_cmd_data(0xFE, d, sizeof(d));
    }

    // Color format: 16-bit RGB565
    {
        const uint8_t d[] = {0x55};
        write_cmd_data(CO5300_CMD_COLMOD, d, sizeof(d));
    }

    // Memory Access Control (normal scan direction)
    {
        const uint8_t d[] = {0x00};
        write_cmd_data(CO5300_CMD_MADCTL, d, sizeof(d));
    }

    // Tear effect off
    write_cmd(CO5300_CMD_TEOFF);

    // Sleep out
    write_cmd(CO5300_CMD_SLPOUT);
    delay(120);

    // Display on
    write_cmd(CO5300_CMD_DISPON);
    delay(20);
}

// ── Window / pixel transfer helpers ──────────────────────────────────────────

void SH8501AmoledDisplay::set_window(uint16_t x0, uint16_t y0,
                                      uint16_t x1, uint16_t y1) {
    uint8_t caset[] = {
        (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF),
        (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF)
    };
    uint8_t raset[] = {
        (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF),
        (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF)
    };
    write_cmd_data(CO5300_CMD_CASET, caset, sizeof(caset));
    write_cmd_data(CO5300_CMD_RASET, raset, sizeof(raset));
}

void SH8501AmoledDisplay::write_cmd(uint8_t cmd) {
    spi_transaction_t t = {};
    t.flags    = SPI_TRANS_USE_RXDATA | SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
    t.cmd      = 0x02;  // Write command prefix
    t.addr     = cmd;
    t.length   = 0;
    spi_device_polling_transmit(spi_dev_, &t);
}

void SH8501AmoledDisplay::write_cmd_data(uint8_t cmd, const uint8_t* data, size_t len) {
    write_cmd(cmd);
    if (len == 0 || data == nullptr) return;

    spi_transaction_t t = {};
    t.flags       = SPI_TRANS_USE_RXDATA | SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
    t.cmd         = 0x32;   // Write data in QSPI mode
    t.addr        = 0x003C00; // CO5300 parameter address
    t.tx_buffer   = data;
    t.length      = len * 8;
    spi_device_polling_transmit(spi_dev_, &t);
}

void SH8501AmoledDisplay::write_pixels(const uint8_t* data, size_t len) {
    if (len == 0) return;
    spi_transaction_t t = {};
    t.flags       = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
    t.cmd         = 0x32;
    t.addr        = 0x003C00;
    t.tx_buffer   = data;
    t.length      = len * 8;
    spi_device_polling_transmit(spi_dev_, &t);
}

// ── DisplayBuffer overrides ───────────────────────────────────────────────────

void SH8501AmoledDisplay::update() {
    // LVGL drives refresh via draw_pixels_at — nothing to do here unless
    // running without LVGL (full-buffer redraw mode).
    do_update_();
}

void SH8501AmoledDisplay::draw_pixels_at(int x_start, int y_start, int w, int h,
                                          const uint8_t* data,
                                          display::ColorOrder color_order,
                                          display::ColorBitness color_bitness,
                                          bool big_endian, int x_offset,
                                          int y_offset, int x_pad) {
    if (!initialized_ || w <= 0 || h <= 0) return;

    set_window((uint16_t)x_start, (uint16_t)y_start,
               (uint16_t)(x_start + w - 1), (uint16_t)(y_start + h - 1));
    write_cmd(CO5300_CMD_RAMWR);

    // Data is RGB565 big-endian from LVGL — write directly if format matches
    size_t bytes = (size_t)w * h * 2;
    write_pixels(data, bytes);
}

void SH8501AmoledDisplay::draw_absolute_pixel_internal(int x, int y, Color color) {
    if (!initialized_ || x < 0 || y < 0 || x >= width_ || y >= height_) return;
    uint16_t rgb565 = display::ColorUtil::color_to_565(color);
    uint8_t px[2] = {(uint8_t)(rgb565 >> 8), (uint8_t)(rgb565 & 0xFF)};
    set_window((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)y);
    write_cmd(CO5300_CMD_RAMWR);
    write_pixels(px, 2);
}

}  // namespace sh8501_amoled
}  // namespace esphome
