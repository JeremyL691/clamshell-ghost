#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "usb_device_uac.h"

static const char *TAG = "usb_mic_175c";

/* Waveshare ESP32-S3-Touch-AMOLED-1.75C Pin Definitions */
#define BOARD_I2C_PORT          I2C_NUM_0
#define BOARD_I2C_SDA_GPIO      GPIO_NUM_15
#define BOARD_I2C_SCL_GPIO      GPIO_NUM_14

#define AXP2101_I2C_ADDR        0x34

#define AUDIO_I2S_GPIO_MCLK     GPIO_NUM_16
#define AUDIO_I2S_GPIO_BCLK     GPIO_NUM_9
#define AUDIO_I2S_GPIO_WS       GPIO_NUM_45
#define AUDIO_I2S_GPIO_DOUT     GPIO_NUM_8
#define AUDIO_I2S_GPIO_DIN      GPIO_NUM_10
#define AUDIO_PA_GPIO           GPIO_NUM_46

/* Display & Touch Reset Pins (held low to keep screen 100% powered down) */
#define LCD_RST_GPIO_175C       GPIO_NUM_1
#define LCD_RST_GPIO_175        GPIO_NUM_39
#define TOUCH_RST_GPIO_175C     GPIO_NUM_2
#define TOUCH_RST_GPIO_175      GPIO_NUM_40

#define BOOT_BUTTON_GPIO        GPIO_NUM_0

#define MIC_SAMPLE_RATE         CONFIG_UAC_SAMPLE_RATE
/* Default ES7210 PGA hardware gain step: 12 = 34.5 dB (range 0..14 => 0..37.5 dB) */
#define MIC_DEFAULT_GAIN_STEP   12

/* UDP Dual-Mode Audio & Heartbeat Protocol */
#define ESP32_UDP_PORT          18796
#define ESP32_UDP_MAGIC         0x4D323345UL /* 'E','3','2','M' */
#define ESP32_PKT_AUDIO_PCM16   0x01
#define ESP32_PKT_HEARTBEAT     0x02
#define ESP32_PKT_MAC_ACK_CTRL  0x03
#ifdef CONFIG_ESP32_DEFAULT_MAC_IP
#define ESP32_DEFAULT_MAC_IP    CONFIG_ESP32_DEFAULT_MAC_IP
#else
#define ESP32_DEFAULT_MAC_IP    "255.255.255.255"
#endif
#define ESP32_UDP_FRAME_SAMPLES 480

typedef struct __attribute__((packed)) {
    uint32_t magic;       /* 0x4D323345 */
    uint8_t  pkt_type;    /* 0x01=AUDIO, 0x02=HEARTBEAT, 0x03=MAC_ACK_CTRL */
    uint8_t  flags;       /* bit 0: muted, bit 1: usb_uac_active */
    uint16_t sample_cnt;  /* mono int16_t samples following header (480 for 10ms @ 48kHz, 0 for HEARTBEAT) */
    uint32_t seq;         /* monotonically increasing sequence number */
} esp32_udp_hdr_t;

typedef struct {
    esp32_udp_hdr_t hdr;
    int16_t         pcm[ESP32_UDP_FRAME_SAMPLES];
} esp32_udp_audio_pkt_t;

_Static_assert(sizeof(esp32_udp_hdr_t) == 12, "esp32_udp_hdr_t must be 12 bytes");
_Static_assert(sizeof(esp32_udp_audio_pkt_t) == 972, "esp32_udp_audio_pkt_t must be 972 bytes");

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_axp_dev = NULL;
static i2s_chan_handle_t s_i2s_tx_chan = NULL;
static i2s_chan_handle_t s_i2s_rx_chan = NULL;
static esp_codec_dev_handle_t s_mic_codec_dev = NULL;
static const audio_codec_ctrl_if_t *s_es7210_ctrl_if = NULL;
static SemaphoreHandle_t s_codec_mutex = NULL;

static volatile bool s_host_muted = false;
static volatile bool s_vol_muted = false;
static volatile int32_t s_pending_vol_pct = -1;
static volatile bool s_wifi_connected = false;
static volatile bool s_uac_ever_read = false;
static volatile TickType_t s_uac_last_read_tick = 0;

static int16_t s_tdm_stereo_buf[2048];
static int16_t s_wifi_stereo_buf[ESP32_UDP_FRAME_SAMPLES * 2];

static struct sockaddr_in s_mac_addr;
static struct sockaddr_in s_bcast_addr;
static TickType_t s_last_mac_ack_tick = 0;
static bool s_mac_ack_received = false;

static esp_err_t es7210_set_pga_gain_step(uint8_t step)
{
    if (!s_es7210_ctrl_if || !s_es7210_ctrl_if->write_reg) {
        return ESP_ERR_INVALID_STATE;
    }
    if (step > 14) {
        step = 14; /* Max 37.5 dB */
    }
    /* ES7210 Reg 0x43 (MIC1_GAIN) & Reg 0x44 (MIC2_GAIN): bit 4 = MIC1/2_GAIN_ENABLE (0x10), bits 3:0 = step */
    uint8_t reg_val = 0x10 | (step & 0x0F);
    int r1 = s_es7210_ctrl_if->write_reg(s_es7210_ctrl_if, 0x43, 1, &reg_val, 1);
    int r2 = s_es7210_ctrl_if->write_reg(s_es7210_ctrl_if, 0x44, 1, &reg_val, 1);
    return (r1 == 0 && r2 == 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t axp2101_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_axp_dev, buf, sizeof(buf), 100);
}

static esp_err_t axp2101_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_axp_dev, &reg, 1, val, 1, 100);
}

static void hold_screen_and_pa_off(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << AUDIO_PA_GPIO) |
                        (1ULL << LCD_RST_GPIO_175C) |
                        (1ULL << LCD_RST_GPIO_175) |
                        (1ULL << TOUCH_RST_GPIO_175C) |
                        (1ULL << TOUCH_RST_GPIO_175),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(AUDIO_PA_GPIO, 0);
    gpio_set_level(LCD_RST_GPIO_175C, 0);
    gpio_set_level(LCD_RST_GPIO_175, 0);
    gpio_set_level(TOUCH_RST_GPIO_175C, 0);
    gpio_set_level(TOUCH_RST_GPIO_175, 0);

    gpio_config_t boot_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&boot_conf);
}

static esp_err_t init_i2c_and_pmic(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "I2C bus init failed");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP2101_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_axp_dev), TAG, "AXP2101 add failed");

    /* Configure AXP2101 PMIC:
     * - Only DC1 (3.3V system) and ALDO1 (3.3V Analog Mic Supply) enabled
     * - All other LDOs/DCs disabled to keep display & unused rails powered off
     * - Charger target voltage 4.1V with hardware power-path management (NVDC)
     */
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x22, 0x06), TAG, "AXP2101 reg 0x22 failed"); // PWRON > OFFLEVEL as poweroff source enable
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x27, 0x10), TAG, "AXP2101 reg 0x27 failed"); // Hold 4s to power off
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x80, 0x01), TAG, "AXP2101 reg 0x80 failed"); // Enable only DC1
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x90, 0x00), TAG, "AXP2101 reg 0x90 failed"); // Disable all LDOs first
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x91, 0x00), TAG, "AXP2101 reg 0x91 failed");
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x82, (3300 - 1500) / 100), TAG, "AXP2101 reg 0x82 failed"); // DC1 = 3.3V
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x92, (3300 - 500) / 100), TAG, "AXP2101 reg 0x92 failed");  // ALDO1 = 3.3V (MIC bias/supply)
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x90, 0x01), TAG, "AXP2101 reg 0x90 enable failed"); // Enable ALDO1 (MIC)
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x64, 0x02), TAG, "AXP2101 reg 0x64 failed"); // CV charger voltage = 4.1V
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x61, 0x02), TAG, "AXP2101 reg 0x61 failed"); // Precharge current = 50mA
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x62, 0x08), TAG, "AXP2101 reg 0x62 failed"); // Constant charge current = 200mA
    ESP_RETURN_ON_ERROR(axp2101_write_reg(0x63, 0x11), TAG, "AXP2101 reg 0x63 failed"); // Enable charge termination (bit 4) + ITERM = 25mA
    vTaskDelay(pdMS_TO_TICKS(20)); // Allow ALDO1 mic rail to settle

    ESP_LOGI(TAG, "AXP2101 initialized (ALDO1 Mic ON, Screen Rails OFF)");
    return ESP_OK;
}

static esp_err_t init_es7210_microphone(void)
{
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_i2s_tx_chan, &s_i2s_rx_chan), TAG, "i2s_new_channel failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = MIC_SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = AUDIO_I2S_GPIO_MCLK,
            .bclk = AUDIO_I2S_GPIO_BCLK,
            .ws   = AUDIO_I2S_GPIO_WS,
            .dout = AUDIO_I2S_GPIO_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_tx_chan, &std_cfg), TAG, "init std tx failed");
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din  = AUDIO_I2S_GPIO_DIN;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_rx_chan, &std_cfg), TAG, "init std rx failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_i2s_tx_chan), TAG, "enable tx failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_i2s_rx_chan), TAG, "enable rx failed");

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = s_i2s_rx_chan,
        .tx_handle = s_i2s_tx_chan,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(data_if != NULL, ESP_FAIL, TAG, "audio_codec_new_i2s_data failed");

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BOARD_I2C_PORT,
        .addr = ES7210_CODEC_DEFAULT_ADDR,
        .bus_handle = s_i2c_bus,
    };
    const audio_codec_ctrl_if_t *in_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(in_ctrl_if != NULL, ESP_FAIL, TAG, "audio_codec_new_i2c_ctrl failed");
    s_es7210_ctrl_if = in_ctrl_if;

    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = in_ctrl_if,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2,
    };
    const audio_codec_if_t *in_codec_if = es7210_codec_new(&es7210_cfg);
    ESP_RETURN_ON_FALSE(in_codec_if != NULL, ESP_FAIL, TAG, "es7210_codec_new failed");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = in_codec_if,
        .data_if = data_if,
    };
    s_mic_codec_dev = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(s_mic_codec_dev != NULL, ESP_FAIL, TAG, "esp_codec_dev_new failed");

    /* Standard 2-channel I2S mode: Left = MIC1, Right = MIC2 */
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 2,
        .channel_mask = ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0) | ESP_CODEC_DEV_MAKE_CHANNEL_MASK(1),
        .sample_rate = MIC_SAMPLE_RATE,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    };
    ESP_RETURN_ON_FALSE(esp_codec_dev_open(s_mic_codec_dev, &fs) == ESP_CODEC_DEV_OK,
                        ESP_FAIL, TAG, "esp_codec_dev_open failed");

    /* Set exact hardware PGA gain directly via ES7210 registers 0x43 (MIC1) & 0x44 (MIC2) */
    ESP_RETURN_ON_ERROR(es7210_set_pga_gain_step(MIC_DEFAULT_GAIN_STEP), TAG, "set ES7210 PGA gain failed");

    ESP_LOGI(TAG, "ES7210 dual microphone initialized @ %d Hz", MIC_SAMPLE_RATE);
    return ESP_OK;
}

static void mix_stereo_to_mono(const int16_t *src, int16_t *dst, size_t mono_samples)
{
    for (size_t i = 0; i < mono_samples; i++) {
        int32_t m1 = src[2 * i];
        int32_t m2 = src[2 * i + 1];
        /* Combine dual microphones with 0.75x headroom scaling ((3 * (m1 + m2)) >> 2)
         * to preserve +3dB dual-mic SNR gain while preventing premature hard clipping on loud speech. */
        int32_t sum = m1 + m2;
        int32_t mixed = (sum * 3) >> 2;
        if (mixed > 32767) {
            mixed = 32767;
        } else if (mixed < -32768) {
            mixed = -32768;
        }
        dst[i] = (int16_t)mixed;
    }
}

static esp_err_t uac_mic_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *cb_ctx)
{
    (void)cb_ctx;
    if (!s_mic_codec_dev) {
        return ESP_ERR_INVALID_STATE;
    }

    s_uac_last_read_tick = xTaskGetTickCount();
    s_uac_ever_read = true;

    /* Since we read 2 channels (MIC1 + MIC2) from ES7210 and output 1 mono channel (len bytes) to Mac,
     * we read 2 * len bytes from esp_codec_dev and sum/average MIC1 + MIC2 for +3dB SNR improvement.
     */
    size_t stereo_bytes = len * 2;
    if (stereo_bytes > sizeof(s_tdm_stereo_buf)) {
        stereo_bytes = sizeof(s_tdm_stereo_buf);
        len = stereo_bytes / 2;
    }

    int ret = ESP_CODEC_DEV_DRV_ERR;
    if (s_codec_mutex && xSemaphoreTake(s_codec_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        ret = esp_codec_dev_read(s_mic_codec_dev, s_tdm_stereo_buf, stereo_bytes);
        xSemaphoreGive(s_codec_mutex);
    }

    if (ret != ESP_CODEC_DEV_OK || s_host_muted || s_vol_muted) {
        memset(buf, 0, len);
        *bytes_read = len;
        return ESP_OK;
    }

    mix_stereo_to_mono(s_tdm_stereo_buf, (int16_t *)buf, len / sizeof(int16_t));
    *bytes_read = len;
    return ESP_OK;
}

static void uac_set_mute_cb(uint32_t mute, void *cb_ctx)
{
    (void)cb_ctx;
    s_host_muted = (mute != 0);
}

static void uac_set_volume_cb(uint32_t volume, void *cb_ctx)
{
    (void)cb_ctx;
    int32_t vol_pct = (int32_t)volume;
    if (vol_pct <= 0) {
        s_vol_muted = true;
        return;
    }
    if (vol_pct > 100) {
        vol_pct = 100;
    }
    s_vol_muted = false;
    /* Defer I2C register write to housekeep_task to avoid cross-task I2C bus contention */
    s_pending_vol_pct = vol_pct;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi STA connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
    }
}

static esp_err_t init_wifi_sta_from_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs_flash_init failed");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "esp_event_loop_create_default failed");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler, NULL, NULL),
                        TAG, "register WIFI_EVENT failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                            &wifi_event_handler, NULL, NULL),
                        TAG, "register IP_EVENT_STA_GOT_IP failed");

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "esp_wifi_set_mode failed");

    /* Check NVS namespace "wifi" (key="ssid", key="password") first;
     * fall back to standard ESP-IDF Wi-Fi NVS configuration if "wifi" namespace is absent. */
    wifi_config_t wifi_cfg = {0};
    bool loaded_from_custom_nvs = false;
    nvs_handle_t nvs_h = 0;
    if (nvs_open("wifi", NVS_READONLY, &nvs_h) == ESP_OK) {
        char ssid_buf[33] = {0};
        char pass_buf[65] = {0};
        size_t ssid_len = sizeof(ssid_buf);
        size_t pass_len = sizeof(pass_buf);
        esp_err_t s_err = nvs_get_str(nvs_h, "ssid", ssid_buf, &ssid_len);
        if (s_err != ESP_OK) {
            ssid_len = sizeof(ssid_buf);
            s_err = nvs_get_str(nvs_h, "ssid0", ssid_buf, &ssid_len);
            if (s_err == ESP_OK) {
                nvs_get_str(nvs_h, "password0", pass_buf, &pass_len);
            }
        } else {
            nvs_get_str(nvs_h, "password", pass_buf, &pass_len);
        }
        nvs_close(nvs_h);

        if (s_err == ESP_OK && ssid_buf[0] != '\0') {
            strncpy((char *)wifi_cfg.sta.ssid, ssid_buf, sizeof(wifi_cfg.sta.ssid) - 1);
            strncpy((char *)wifi_cfg.sta.password, pass_buf, sizeof(wifi_cfg.sta.password) - 1);
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
            loaded_from_custom_nvs = true;
        }
        memset(ssid_buf, 0, sizeof(ssid_buf));
        memset(pass_buf, 0, sizeof(pass_buf));
    }

    if (!loaded_from_custom_nvs) {
        esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);
    }

#ifdef CONFIG_ESP32_WIFI_SSID
    if (wifi_cfg.sta.ssid[0] == '\0' && CONFIG_ESP32_WIFI_SSID[0] != '\0') {
        strncpy((char *)wifi_cfg.sta.ssid, CONFIG_ESP32_WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
#ifdef CONFIG_ESP32_WIFI_PASSWORD
        strncpy((char *)wifi_cfg.sta.password, CONFIG_ESP32_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);
#endif
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    }
#endif

    ESP_LOGI(TAG, "Wi-Fi STA credentials loaded (ssid_len=%u)",
             (unsigned)strnlen((const char *)wifi_cfg.sta.ssid, sizeof(wifi_cfg.sta.ssid)));
    memset(&wifi_cfg, 0, sizeof(wifi_cfg));

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "esp_wifi_set_ps(WIFI_PS_NONE) failed");
    return ESP_OK;
}

static void poll_mac_udp_ctrl(int sock)
{
    uint8_t rx_buf[64];
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);

    while (1) {
        addr_len = sizeof(src_addr);
        ssize_t n = recvfrom(sock, rx_buf, sizeof(rx_buf), MSG_DONTWAIT,
                             (struct sockaddr *)&src_addr, &addr_len);
        if (n < (ssize_t)sizeof(esp32_udp_hdr_t)) {
            break;
        }
        esp32_udp_hdr_t hdr;
        memcpy(&hdr, rx_buf, sizeof(hdr));
        if (hdr.magic == ESP32_UDP_MAGIC && hdr.pkt_type == ESP32_PKT_MAC_ACK_CTRL) {
            s_mac_addr.sin_addr = src_addr.sin_addr;
            s_last_mac_ack_tick = xTaskGetTickCount();
            s_mac_ack_received = true;
            if (hdr.sample_cnt >= 1 && hdr.sample_cnt <= 100) {
                s_vol_muted = false;
                s_pending_vol_pct = (int32_t)hdr.sample_cnt;
            }
        }
    }
}

static void send_udp_heartbeat(int sock, const struct sockaddr_in *dest, uint8_t flags, uint32_t *seq)
{
    esp32_udp_hdr_t hb = {
        .magic = ESP32_UDP_MAGIC,
        .pkt_type = ESP32_PKT_HEARTBEAT,
        .flags = flags,
        .sample_cnt = 0,
        .seq = (*seq)++,
    };
    sendto(sock, &hb, sizeof(hb), 0, (const struct sockaddr *)dest, sizeof(*dest));
}

static void wifi_audio_task(void *arg)
{
    (void)arg;
    memset(&s_mac_addr, 0, sizeof(s_mac_addr));
    s_mac_addr.sin_family = AF_INET;
    s_mac_addr.sin_port = htons(ESP32_UDP_PORT);
    inet_pton(AF_INET, ESP32_DEFAULT_MAC_IP, &s_mac_addr.sin_addr);

    memset(&s_bcast_addr, 0, sizeof(s_bcast_addr));
    s_bcast_addr.sin_family = AF_INET;
    s_bcast_addr.sin_port = htons(ESP32_UDP_PORT);
    s_bcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    int sock = -1;
    uint32_t udp_seq = 0;
    uint32_t hb_seq = 0;
    uint32_t audio_pkt_cnt = 0;
    TickType_t last_bcast_tick = 0;
    esp32_udp_audio_pkt_t audio_pkt;

    while (1) {
        if (!s_wifi_connected) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            int broadcast = 1;
            setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
            int fl = fcntl(sock, F_GETFL, 0);
            fcntl(sock, F_SETFL, fl | O_NONBLOCK);

            struct sockaddr_in bind_addr = {
                .sin_family = AF_INET,
                .sin_port = htons(ESP32_UDP_PORT),
                .sin_addr.s_addr = htonl(INADDR_ANY),
            };
            if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
                close(sock);
                sock = -1;
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
        }

        poll_mac_udp_ctrl(sock);

        TickType_t now = xTaskGetTickCount();
        bool usb_uac_active = s_uac_ever_read &&
                              ((now - s_uac_last_read_tick) < pdMS_TO_TICKS(150));
        uint8_t flags = ((s_host_muted || s_vol_muted) ? 0x01 : 0x00) |
                        (usb_uac_active ? 0x02 : 0x00);

        /* Broadcast discovery fallback when Mac ACK has not been received in the last 5 seconds */
        if ((!s_mac_ack_received || (now - s_last_mac_ack_tick) > pdMS_TO_TICKS(5000)) &&
            (now - last_bcast_tick) >= pdMS_TO_TICKS(2000)) {
            send_udp_heartbeat(sock, &s_bcast_addr, flags, &hb_seq);
            last_bcast_tick = now;
        }

        if (usb_uac_active) {
            /* USB UAC is actively streaming audio; send Wi-Fi heartbeat every 500ms without reading I2S */
            send_udp_heartbeat(sock, &s_mac_addr, flags, &hb_seq);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        /* Wireless Mode: USB UAC is inactive, stream 48kHz 16-bit mono PCM (480 samples / 10ms) over UDP */
        int ret = ESP_CODEC_DEV_DRV_ERR;
        if (s_codec_mutex && xSemaphoreTake(s_codec_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            ret = esp_codec_dev_read(s_mic_codec_dev, s_wifi_stereo_buf, sizeof(s_wifi_stereo_buf));
            xSemaphoreGive(s_codec_mutex);
        }

        if (ret != ESP_CODEC_DEV_OK || s_host_muted || s_vol_muted) {
            memset(audio_pkt.pcm, 0, sizeof(audio_pkt.pcm));
        } else {
            mix_stereo_to_mono(s_wifi_stereo_buf, audio_pkt.pcm, ESP32_UDP_FRAME_SAMPLES);
        }

        audio_pkt.hdr.magic = ESP32_UDP_MAGIC;
        audio_pkt.hdr.pkt_type = ESP32_PKT_AUDIO_PCM16;
        audio_pkt.hdr.flags = flags;
        audio_pkt.hdr.sample_cnt = ESP32_UDP_FRAME_SAMPLES;
        audio_pkt.hdr.seq = udp_seq++;

        sendto(sock, &audio_pkt, sizeof(audio_pkt), 0,
               (const struct sockaddr *)&s_mac_addr, sizeof(s_mac_addr));

        audio_pkt_cnt++;
        if (audio_pkt_cnt >= 100) {
            audio_pkt_cnt = 0;
            send_udp_heartbeat(sock, &s_mac_addr, flags, &hb_seq);
        }
    }
}

static void housekeep_task(void *arg)
{
    (void)arg;
    int unplug_ticks = 0;
    int boot_hold_ticks = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250));

        /* 0. Apply any pending host volume update on the single I2C housekeeping thread */
        int32_t pending_vol = s_pending_vol_pct;
        if (pending_vol > 0) {
            s_pending_vol_pct = -1;
            /* Map 1..100% host volume to ES7210 PGA gain step 5..14 (15.0 dB .. 37.5 dB) */
            uint8_t step = (uint8_t)(5 + ((pending_vol * 9 + 50) / 100));
            es7210_set_pga_gain_step(step);
        }

        /* 1. Check if USB cable was physically unplugged (AXP2101 Reg 0x00 bit 5 = VBUS good).
         * Do NOT trigger on discharging status alone, as a battery above 4.1V target or transient load
         * can report discharging while VBUS is still connected. */
        uint8_t status0 = 0x20;
        if (axp2101_read_reg(0x00, &status0) == ESP_OK) {
            bool vbus_present = (status0 & 0x20) != 0;
            if (!vbus_present) {
                unplug_ticks++;
                if (unplug_ticks >= 6) { /* VBUS absent for 1.5s -> power off PMIC so battery never drains */
                    ESP_LOGI(TAG, "USB unplugged, shutting down AXP2101 PMIC");
                    uint8_t pwr_ctrl = 0;
                    axp2101_read_reg(0x10, &pwr_ctrl);
                    axp2101_write_reg(0x10, pwr_ctrl | 0x01);
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            } else {
                unplug_ticks = 0;
            }
        }

        /* 2. Maintenance escape hatch: holding BOOT for 3s restarts into ROM USB-Serial-JTAG download mode */
        if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
            boot_hold_ticks++;
            if (boot_hold_ticks >= 12) {
                REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
                esp_restart();
            }
        } else {
            boot_hold_ticks = 0;
        }
    }
}

void app_main(void)
{
    hold_screen_and_pa_off();
    s_codec_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_codec_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(init_i2c_and_pmic());
    ESP_ERROR_CHECK(init_es7210_microphone());
    ESP_ERROR_CHECK(init_wifi_sta_from_nvs());

    uac_device_config_t uac_cfg = {
        .skip_tinyusb_init = false,
        .output_cb = NULL,
        .input_cb = uac_mic_input_cb,
        .set_mute_cb = uac_set_mute_cb,
        .set_volume_cb = uac_set_volume_cb,
        .cb_ctx = NULL,
    };
    ESP_ERROR_CHECK(uac_device_init(&uac_cfg));

    BaseType_t wifi_task_ok = xTaskCreatePinnedToCore(wifi_audio_task, "wifi_audio", 6144, NULL, 5, NULL, 1);
    ESP_ERROR_CHECK(wifi_task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    BaseType_t pmic_task_ok = xTaskCreatePinnedToCore(housekeep_task, "pmic_watch", 4096, NULL, 2, NULL, 0);
    ESP_ERROR_CHECK(pmic_task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_LOGI(TAG, "Waveshare ESP32-S3 Dual-Mode (Wi-Fi UDP + USB UAC) Microphone Ready");
}
