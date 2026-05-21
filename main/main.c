
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wps.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/temperature_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_ota_ops.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "class/hid/hid_device.h"

extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[]   asm("_binary_index_html_end");
extern const unsigned char wifi_html_start[]  asm("_binary_wifi_html_start");
extern const unsigned char wifi_html_end[]    asm("_binary_wifi_html_end");
extern const unsigned char usb_html_start[]   asm("_binary_usb_html_start");
extern const unsigned char usb_html_end[]     asm("_binary_usb_html_end");
extern const unsigned char help_html_start[]  asm("_binary_help_html_start");
extern const unsigned char help_html_end[]    asm("_binary_help_html_end");
extern const unsigned char hw_html_start[]      asm("_binary_hw_html_start");
extern const unsigned char hw_html_end[]        asm("_binary_hw_html_end");
extern const unsigned char monitor_html_start[] asm("_binary_monitor_html_start");
extern const unsigned char monitor_html_end[]   asm("_binary_monitor_html_end");
extern const unsigned char ota_html_start[]     asm("_binary_ota_html_start");
extern const unsigned char ota_html_end[]       asm("_binary_ota_html_end");

#define VERSION "0.4.3"
#define REVISION 0

typedef struct {
    char name[32];
    char body[512];
} macro_t;

#define MAX_MACROS 20
#define MACRO_NS   "macros"
static macro_t macros[MAX_MACROS];
static bool    hid_enumerated = false;
static uint8_t hid_led_state  = 0;  /* bits: 0=NumLock 1=CapsLock 2=ScrollLock */

/* Web client tracking — FD→IP captured at accept() time, timestamp updated per request */
#define MAX_WEB_CLIENTS       16
#define WEB_CLIENT_TIMEOUT_MS 60000
typedef struct { int fd; uint32_t ip; int64_t last_ms; } web_client_t;
static web_client_t s_web_clients[MAX_WEB_CLIENTS];
static int          s_web_client_count = 0;

/* Called by httpd right after accept() — socket is in CONNECTED state here.
   With CONFIG_LWIP_IPV6=y the httpd uses AF_INET6 dual-stack; IPv4 clients
   appear as IPv4-mapped IPv6 addresses (::ffff:a.b.c.d). */
static esp_err_t httpd_open_fn(httpd_handle_t hd, int sockfd) {
    (void)hd;
    struct sockaddr_storage ss;
    socklen_t len = sizeof(ss);
    memset(&ss, 0, sizeof(ss));
    if (lwip_getpeername(sockfd, (struct sockaddr *)&ss, &len) != 0) return ESP_OK;
    uint32_t ip = 0;
    if (ss.ss_family == AF_INET) {
        ip = ((struct sockaddr_in *)&ss)->sin_addr.s_addr;
    } else if (ss.ss_family == AF_INET6) {
        /* extract IPv4 from IPv4-mapped address ::ffff:a.b.c.d */
        uint8_t *b = ((struct sockaddr_in6 *)&ss)->sin6_addr.s6_addr;
        if (b[10] == 0xFF && b[11] == 0xFF)   /* IPv4-mapped */
            memcpy(&ip, b + 12, 4);
    }
    if (ip == 0) return ESP_OK;
    int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    /* update existing slot for this fd (reused fd) */
    for (int i = 0; i < s_web_client_count; i++) {
        if (s_web_clients[i].fd == sockfd) {
            s_web_clients[i].ip = ip; s_web_clients[i].last_ms = now; return ESP_OK;
        }
    }
    /* add new slot */
    if (s_web_client_count < MAX_WEB_CLIENTS) {
        s_web_clients[s_web_client_count++] = (web_client_t){sockfd, ip, now};
    } else {
        int old = 0;
        for (int i = 1; i < MAX_WEB_CLIENTS; i++)
            if (s_web_clients[i].last_ms < s_web_clients[old].last_ms) old = i;
        s_web_clients[old] = (web_client_t){sockfd, ip, now};
    }
    return ESP_OK;
}

/* Called by httpd on close — replaces the default close(), so we must close the socket */
static void httpd_close_fn(httpd_handle_t hd, int sockfd) {
    (void)hd;
    for (int i = 0; i < s_web_client_count; i++)
        if (s_web_clients[i].fd == sockfd) { s_web_clients[i].fd = -1; break; }
    close(sockfd);
}

/* Update last-seen timestamp for this request's socket */
static void track_web_client(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);
    int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    for (int i = 0; i < s_web_client_count; i++)
        if (s_web_clients[i].fd == fd) { s_web_clients[i].last_ms = now; return; }
}

static int web_client_count_active(void) {
    int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    int n = 0;
    for (int i = 0; i < s_web_client_count; i++)
        if (s_web_clients[i].ip && now - s_web_clients[i].last_ms < WEB_CLIENT_TIMEOUT_MS) n++;
    return n;
}

static bool    sta_connected   = false;
static bool    s_sta_connecting = false; /* esp_wifi_connect() called, not yet got IP */
static char    sta_ip[16]      = "";
static bool    s_manual_disconnect = false;
static int     s_suppress_disc     = 0;
static TaskHandle_t s_wifi_mgr_task = NULL;
static volatile bool s_scanning  = false;
static volatile bool s_scan_done = false;

/* ---- LED status indicator ---- */
#define LED_PIN_DEFAULT (-1)   /* -1 = disabled; configure via USB page */
static int  s_led_pin      = LED_PIN_DEFAULT;
static bool s_led_neopixel = false;
static bool s_led_invert   = false;  /* active-low LED: invert GPIO output */
static volatile int s_led_logical = 0;  /* last written logical level */

#define LED_CMD_BOOT      0   /* 2 Hz blink while booting */
#define LED_CMD_IDLE      1   /* off (not connected) */
#define LED_CMD_CONNECTED 2   /* steady on */
#define LED_CMD_FLASH     3   /* one-shot 0.7 s flash (connect attempt) */

static QueueHandle_t s_led_queue;

/* ---- NeoPixel (WS2812) via RMT ---- */
#define NEO_RES_HZ 10000000  /* 10 MHz → 0.1 µs per tick */

static rmt_channel_handle_t s_neo_chan    = NULL;
static rmt_encoder_handle_t s_neo_encoder = NULL;
static uint8_t s_neo_rgb[3] = {0, 0, 0};  /* current "on" color */

static const rmt_symbol_word_t s_neo_zero  = { .level0=1,.duration0=3, .level1=0,.duration1=9 };
static const rmt_symbol_word_t s_neo_one   = { .level0=1,.duration0=9, .level1=0,.duration1=3 };
static const rmt_symbol_word_t s_neo_reset = { .level0=0,.duration0=250,.level1=0,.duration1=250 };

static size_t neo_encoder_cb(const void *data, size_t data_size,
                              size_t symbols_written, size_t symbols_free,
                              rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    (void)arg;
    if (symbols_free < 8) return 0;
    size_t pos = symbols_written / 8;
    const uint8_t *bytes = (const uint8_t *)data;
    if (pos < data_size) {
        size_t n = 0;
        for (int bit = 0x80; bit; bit >>= 1)
            symbols[n++] = (bytes[pos] & bit) ? s_neo_one : s_neo_zero;
        return n;
    }
    symbols[0] = s_neo_reset;
    *done = true;
    return 1;
}

static void neo_write(uint8_t r, uint8_t g, uint8_t b) {
    if (!s_neo_chan || !s_neo_encoder) return;
    uint8_t grb[3] = {g, r, b};  /* WS2812 wants GRB order */
    rmt_transmit_config_t tx_cfg = {.loop_count = 0};
    rmt_transmit(s_neo_chan, s_neo_encoder, grb, sizeof(grb), &tx_cfg);
    rmt_tx_wait_all_done(s_neo_chan, pdMS_TO_TICKS(100));
}

static void led_write(int v) {
    s_led_logical = v;
    if (s_led_neopixel) {
        neo_write(v ? s_neo_rgb[0] : 0, v ? s_neo_rgb[1] : 0, v ? s_neo_rgb[2] : 0);
    } else if (s_led_pin >= 0) {
        gpio_set_level(s_led_pin, s_led_invert ? !v : v);
    }
}

static void led_cmd(uint8_t c) {
    if (s_led_queue) xQueueSend(s_led_queue, &c, pdMS_TO_TICKS(50));
}

/* Invert LED output around key-down without changing the logical state */
static void led_key_invert(bool inv) {
    if (s_led_neopixel) {
        if (inv) {
            neo_write(32, 0, 0);  /* red flash during keypress */
        } else {
            neo_write(s_led_logical ? s_neo_rgb[0] : 0,
                      s_led_logical ? s_neo_rgb[1] : 0,
                      s_led_logical ? s_neo_rgb[2] : 0);
        }
    } else if (s_led_pin >= 0) {
        gpio_set_level(s_led_pin, (bool)s_led_logical ^ (bool)inv ^ s_led_invert);
    }
}

static void led_task(void *arg) {
    (void)arg;
    if (s_led_neopixel && s_led_pin >= 0) {
        rmt_tx_channel_config_t tx_cfg = {
            .clk_src          = RMT_CLK_SRC_DEFAULT,
            .gpio_num         = s_led_pin,
            .mem_block_symbols = 64,
            .resolution_hz    = NEO_RES_HZ,
            .trans_queue_depth = 4,
        };
        rmt_new_tx_channel(&tx_cfg, &s_neo_chan);
        rmt_simple_encoder_config_t enc_cfg = { .callback = neo_encoder_cb };
        rmt_new_simple_encoder(&enc_cfg, &s_neo_encoder);
        rmt_enable(s_neo_chan);
    } else if (!s_led_neopixel && s_led_pin >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_led_pin,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io);
    }
    uint8_t cmd;
    uint8_t mode = LED_CMD_BOOT;
    int blink = 0;
    for (;;) {
        TickType_t timeout = (mode == LED_CMD_BOOT) ? pdMS_TO_TICKS(250) : portMAX_DELAY;
        if (xQueueReceive(s_led_queue, &cmd, timeout) == pdTRUE) {
            if (cmd == LED_CMD_FLASH) {
                uint8_t sr = s_neo_rgb[0], sg = s_neo_rgb[1], sb = s_neo_rgb[2];
                s_neo_rgb[0] = 32; s_neo_rgb[1] = 32; s_neo_rgb[2] = 0; /* yellow */
                led_write(1);
                vTaskDelay(pdMS_TO_TICKS(700));
                s_neo_rgb[0] = sr; s_neo_rgb[1] = sg; s_neo_rgb[2] = sb;
                led_write(mode == LED_CMD_CONNECTED ? 1 : 0);
                continue;
            }
            mode = cmd;
            blink = 0;
        }
        switch (mode) {
            case LED_CMD_BOOT:
                s_neo_rgb[0] = 0; s_neo_rgb[1] = 0; s_neo_rgb[2] = 32; /* blue */
                blink ^= 1; led_write(blink); break;
            case LED_CMD_IDLE:
                led_write(0); break;
            case LED_CMD_CONNECTED:
                s_neo_rgb[0] = 0; s_neo_rgb[1] = 32; s_neo_rgb[2] = 0; /* green */
                led_write(1); break;
        }
    }
}

/* ---- Macro NVS persistence ---- */
static void macros_load(void) {
    nvs_handle_t h;
    if (nvs_open(MACRO_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t cnt = 0;
    nvs_get_u8(h, "count", &cnt);
    if (cnt > MAX_MACROS) cnt = MAX_MACROS;
    for (int i = 0; i < (int)cnt; i++) {
        char key[16];
        size_t len;
        snprintf(key, sizeof(key), "n%d", i);
        len = sizeof(macros[i].name);
        nvs_get_str(h, key, macros[i].name, &len);
        snprintf(key, sizeof(key), "b%d", i);
        len = sizeof(macros[i].body);
        nvs_get_str(h, key, macros[i].body, &len);
    }
    nvs_close(h);
}

/* Save one macro to NVS; returns false if NVS is full */
static bool macros_save_one(int idx) {
    nvs_handle_t h;
    if (nvs_open(MACRO_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    char key[16];
    bool ok = true;
    snprintf(key, sizeof(key), "n%d", idx);
    if (nvs_set_str(h, key, macros[idx].name) != ESP_OK) ok = false;
    snprintf(key, sizeof(key), "b%d", idx);
    if (nvs_set_str(h, key, macros[idx].body) != ESP_OK) ok = false;
    /* Update count if this slot is newly filled */
    uint8_t cnt = 0; nvs_get_u8(h, "count", &cnt);
    if (idx >= (int)cnt) { cnt = (uint8_t)(idx + 1); nvs_set_u8(h, "count", cnt); }
    if (ok) nvs_commit(h);
    nvs_close(h);
    return ok;
}

static void macros_delete_one(int idx) {
    nvs_handle_t h;
    if (nvs_open(MACRO_NS, NVS_READWRITE, &h) != ESP_OK) return;
    char key[16];
    snprintf(key, sizeof(key), "n%d", idx);
    nvs_erase_key(h, key);
    snprintf(key, sizeof(key), "b%d", idx);
    nvs_erase_key(h, key);
    nvs_commit(h);
    nvs_close(h);
}

/* ---- AP config (NVS) ---- */
#define AP_CFG_NS "ap_cfg"
static char ap_ssid[33]    = "AdminKbd";
static char ap_pass[65]    = "12345678";
static char s_hostname[33] = "adminkbd";
static int  s_btn_pin      = 0;         /* WPS long-press button GPIO, -1=disabled */
static volatile bool s_wps_active = false;

static void ap_cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open(AP_CFG_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len;
    len = sizeof(ap_ssid);    nvs_get_str(h, "ssid",     ap_ssid,    &len);
    len = sizeof(ap_pass);    nvs_get_str(h, "pass",     ap_pass,    &len);
    len = sizeof(s_hostname); nvs_get_str(h, "hostname", s_hostname, &len);
    nvs_close(h);
}

static void ap_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(AP_CFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "ssid",     ap_ssid);
    nvs_set_str(h, "pass",     ap_pass);
    nvs_set_str(h, "hostname", s_hostname);
    nvs_commit(h);
    nvs_close(h);
}

/* ---- WiFi credential storage (NVS) ---- */
#define WIFI_CRED_NS  "wifi_cfg"
#define WIFI_CRED_MAX 10

typedef struct { char ssid[33]; char pass[65]; } wifi_cred_t;
static wifi_cred_t wifi_creds[WIFI_CRED_MAX];
static int         wifi_cred_count = 0;

/* ---- USB HID descriptors ---- */
#define HID_REPORT_ID_KEYBOARD 1
#define HID_REPORT_ID_CONSUMER 2
#define EPNUM_HID              0x81

static const uint8_t s_hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_REPORT_ID_KEYBOARD)),
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(HID_REPORT_ID_CONSUMER))
};

/* Mutable USB device identity — overwritten from NVS before USB init */
static char s_manufacturer[64] = "Anonymous";
static char s_product[64]      = "Keyboard";
static char s_serial[32]       = "lifesim.de-001";

static tusb_desc_device_t s_device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0,
    .bDeviceSubClass    = 0,
    .bDeviceProtocol    = 0,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x16c0,
    .idProduct          = 0x27db,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

#define HID_CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t s_hid_config_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, HID_CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(s_hid_report_descriptor), EPNUM_HID, 64, 10),
};

static const char *s_string_descriptor[] = {
    "\x09\x04",      /* 0: language (English) */
    s_manufacturer,  /* 1: Manufacturer */
    s_product,       /* 2: Product */
    s_serial,        /* 3: Serial */
};

/* ---- USB config (NVS) ---- */
#define USB_CFG_NS "usb_cfg"

static void usb_cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open(USB_CFG_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len;
    len = sizeof(s_manufacturer); nvs_get_str(h, "mfr",     s_manufacturer, &len);
    len = sizeof(s_product);      nvs_get_str(h, "product",  s_product,      &len);
    len = sizeof(s_serial);       nvs_get_str(h, "serial",   s_serial,       &len);
    uint16_t vid = 0, pid = 0;
    if (nvs_get_u16(h, "vid", &vid) == ESP_OK) s_device_descriptor.idVendor  = vid;
    if (nvs_get_u16(h, "pid", &pid) == ESP_OK) s_device_descriptor.idProduct = pid;
    nvs_close(h);
}

static void usb_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(USB_CFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "mfr",     s_manufacturer);
    nvs_set_str(h, "product", s_product);
    nvs_set_str(h, "serial",  s_serial);
    nvs_set_u16(h, "vid",     s_device_descriptor.idVendor);
    nvs_set_u16(h, "pid",     s_device_descriptor.idProduct);
    nvs_commit(h);
    nvs_close(h);
}

/* ---- Hardware config (LED + WPS button) ---- */
#define HW_CFG_NS "hw_cfg"

static void hw_cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open(HW_CFG_NS, NVS_READONLY, &h) != ESP_OK) return;
    int8_t lp = LED_PIN_DEFAULT;
    if (nvs_get_i8(h, "led_pin", &lp) == ESP_OK) s_led_pin = lp;
    uint8_t neo = 0;
    if (nvs_get_u8(h, "led_neo", &neo) == ESP_OK) s_led_neopixel = (neo != 0);
    uint8_t inv = 0;
    if (nvs_get_u8(h, "led_inv", &inv) == ESP_OK) s_led_invert = (inv != 0);
    int8_t bp = 0;
    if (nvs_get_i8(h, "btn_pin", &bp) == ESP_OK) s_btn_pin = bp;
    nvs_close(h);
}

static void hw_cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open(HW_CFG_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i8(h, "led_pin", (int8_t)s_led_pin);
    nvs_set_u8(h, "led_neo", s_led_neopixel ? 1 : 0);
    nvs_set_u8(h, "led_inv", s_led_invert   ? 1 : 0);
    nvs_set_i8(h, "btn_pin", (int8_t)s_btn_pin);
    nvs_commit(h);
    nvs_close(h);
}

/* ---- Delayed restart helper ---- */
static void restart_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static void schedule_restart(void) {
    xTaskCreate(restart_task, "restart", 1024, NULL, 5, NULL);
}

/* ---- TinyUSB callbacks ---- */
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return s_hid_report_descriptor;
}

static void usb_event_cb(tinyusb_event_t *event, void *arg) {
    (void)arg;
    switch (event->id) {
        case TINYUSB_EVENT_ATTACHED:
            hid_enumerated = true;
            printf("USB HID mounted\n");
            break;
        case TINYUSB_EVENT_DETACHED:
            hid_enumerated = false;
            printf("USB HID unmounted\n");
            break;
#ifdef CONFIG_TINYUSB_SUSPEND_CALLBACK
        case TINYUSB_EVENT_SUSPENDED:
            /* Physical cable removal fires SUSPENDED, not DETACHED */
            hid_enumerated = false;
            printf("USB HID suspended (disconnected)\n");
            break;
#endif
#ifdef CONFIG_TINYUSB_RESUME_CALLBACK
        case TINYUSB_EVENT_RESUMED:
            hid_enumerated = tud_mounted();
            printf("USB HID resumed\n");
            break;
#endif
        default:
            break;
    }
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type,
                                uint8_t *buffer, uint16_t reqlen) {
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type,
                            uint8_t const *buffer, uint16_t bufsize) {
    (void)instance; (void)report_id;
    if (report_type == HID_REPORT_TYPE_OUTPUT && bufsize >= 1) {
        hid_led_state = buffer[0];
    }
}

/* ---- WiFi credential storage ---- */
static void creds_save(void){
    nvs_handle_t h;
    if(nvs_open(WIFI_CRED_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "count", (uint8_t)wifi_cred_count);
    for(int i = 0; i < wifi_cred_count; i++){
        char key[16];
        snprintf(key, sizeof(key), "ssid_%d", i);
        nvs_set_str(h, key, wifi_creds[i].ssid);
        snprintf(key, sizeof(key), "pass_%d", i);
        nvs_set_str(h, key, wifi_creds[i].pass);
    }
    nvs_commit(h);
    nvs_close(h);
}

static void creds_load(void){
    nvs_handle_t h;
    if(nvs_open(WIFI_CRED_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t cnt = 0;
    nvs_get_u8(h, "count", &cnt);
    if(cnt > WIFI_CRED_MAX) cnt = WIFI_CRED_MAX;
    wifi_cred_count = cnt;
    for(int i = 0; i < cnt; i++){
        char key[16];
        size_t len = sizeof(wifi_creds[i].ssid);
        snprintf(key, sizeof(key), "ssid_%d", i);
        nvs_get_str(h, key, wifi_creds[i].ssid, &len);
        len = sizeof(wifi_creds[i].pass);
        snprintf(key, sizeof(key), "pass_%d", i);
        nvs_get_str(h, key, wifi_creds[i].pass, &len);
    }
    nvs_close(h);
}

static void creds_add(const char *ssid, const char *pass){
    for(int i = 0; i < wifi_cred_count; i++){
        if(strcmp(wifi_creds[i].ssid, ssid) == 0){
            strncpy(wifi_creds[i].pass, pass, sizeof(wifi_creds[i].pass)-1);
            creds_save();
            return;
        }
    }
    if(wifi_cred_count == WIFI_CRED_MAX){
        memmove(&wifi_creds[0], &wifi_creds[1], sizeof(wifi_cred_t) * (WIFI_CRED_MAX - 1));
        wifi_cred_count--;
    }
    memset(&wifi_creds[wifi_cred_count], 0, sizeof(wifi_cred_t));
    strncpy(wifi_creds[wifi_cred_count].ssid, ssid, sizeof(wifi_creds[0].ssid)-1);
    strncpy(wifi_creds[wifi_cred_count].pass, pass, sizeof(wifi_creds[0].pass)-1);
    wifi_cred_count++;
    creds_save();
}

static void creds_delete(int idx){
    if(idx < 0 || idx >= wifi_cred_count) return;
    memmove(&wifi_creds[idx], &wifi_creds[idx+1],
            sizeof(wifi_cred_t) * (wifi_cred_count - idx - 1));
    memset(&wifi_creds[wifi_cred_count-1], 0, sizeof(wifi_cred_t));
    wifi_cred_count--;
    creds_save();
}

/* Move credential at idx up (-1) or down (+1) in priority order */
static void creds_move(int idx, int dir){
    int tgt = idx + dir;
    if(tgt < 0 || tgt >= wifi_cred_count) return;
    wifi_cred_t tmp = wifi_creds[idx];
    wifi_creds[idx] = wifi_creds[tgt];
    wifi_creds[tgt] = tmp;
    creds_save();
}

/* Connect to credential idx directly (used for explicit user requests) */
static void wifi_do_connect(int idx){
    if(idx < 0 || idx >= wifi_cred_count) return;
    wifi_config_t cfg = {};
    strncpy((char*)cfg.sta.ssid,     wifi_creds[idx].ssid, sizeof(cfg.sta.ssid)-1);
    strncpy((char*)cfg.sta.password, wifi_creds[idx].pass, sizeof(cfg.sta.password)-1);
    /* Only call esp_wifi_disconnect() — which fires WIFI_EVENT_STA_DISCONNECTED —
       when we are actually connected or mid-connection.  Calling it from INIT state
       (e.g. right after a scan, or after an auth failure) does NOT fire the event,
       so s_suppress_disc would be left at 1 and the *next* real DISCONNECTED
       (e.g. another auth failure) would be silently swallowed, stalling reconnect
       for the full WIFI_RETRY_MS. */
    if(sta_connected || s_sta_connecting){
        s_suppress_disc++;
        esp_wifi_disconnect();
    }
    sta_connected    = false;
    s_sta_connecting = true;
    sta_ip[0]        = 0;
    s_manual_disconnect = false;
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    led_cmd(LED_CMD_FLASH);
    esp_wifi_connect();
    printf("Connecting to SSID: %s\n", wifi_creds[idx].ssid);
}

/* Scan visible APs, return credential index with best RSSI; -1 if none match */
/* Read scan results already collected by the driver and pick the best known credential.
   Priority (list index) breaks ties: lower index wins when RSSI is within 5 dBm. */
static int wifi_pick_best_from_scan(void){
    if(wifi_cred_count == 0) return -1;
    uint16_t n = 24;
    wifi_ap_record_t *aps = malloc(n * sizeof(wifi_ap_record_t));
    if(!aps) return 0;
    esp_wifi_scan_get_ap_num(&n);
    if(n > 24) n = 24;
    esp_wifi_scan_get_ap_records(&n, aps);

    int best_cred = -1;
    int8_t best_rssi = -127;
    for(int i = 0; i < (int)n; i++){
        for(int j = 0; j < wifi_cred_count; j++){
            if(strcmp((char*)aps[i].ssid, wifi_creds[j].ssid) == 0){
                /* prefer higher RSSI; if within 5 dBm prefer lower index (higher priority) */
                if(aps[i].rssi > best_rssi + 5 ||
                   (aps[i].rssi >= best_rssi - 5 && j < best_cred) ||
                   best_cred < 0){
                    best_rssi = aps[i].rssi;
                    best_cred = j;
                }
                break;
            }
        }
    }
    free(aps);
    if(best_cred >= 0)
        printf("WiFi scan: best [%d] %s (%d dBm)\n",
               best_cred, wifi_creds[best_cred].ssid, (int)best_rssi);
    else
        printf("WiFi scan: no known network visible\n");
    return best_cred;
}

static void wifi_scan_start_async(void){
    wifi_scan_config_t sc = {
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    s_scanning = true;
    s_scan_done = false;
    if(esp_wifi_scan_start(&sc, false) != ESP_OK) s_scanning = false;
}

/* Manager task: scans and connects; retries every 30 s if nothing visible */
#define WIFI_RETRY_MS     30000
#define WIFI_SCAN_TIMEOUT  5000  /* if SCAN_DONE doesn't arrive, give up and connect directly */
static void wifi_manager_task(void *arg){
    (void)arg;
    bool first = true;
    for(;;){
        /* Use a short timeout while a scan is in flight so a missed SCAN_DONE
           doesn't stall reconnect for the full 30 s retry interval */
        uint32_t block_ms = s_scanning ? WIFI_SCAN_TIMEOUT : WIFI_RETRY_MS;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(block_ms));

        if(s_manual_disconnect || s_wps_active || wifi_cred_count == 0){ first=false; s_scan_done=false; s_scanning=false; continue; }

        /* Scan timed out without SCAN_DONE — clear flag and fall through to direct connect */
        if(s_scanning && !s_scan_done){
            s_scanning = false;
            printf("WiFi scan timeout, connecting directly\n");
        }

        /* Woken by SCAN_DONE: pick best credential from results and connect */
        if(s_scan_done){
            s_scan_done = false;
            if(!sta_connected && !s_manual_disconnect){
                int idx = wifi_pick_best_from_scan();
                if(idx >= 0) wifi_do_connect(idx);
            }
            first = false;
            continue;
        }

        if(sta_connected || s_sta_connecting){ first=false; continue; }

        /* On reconnect (not first boot) let the radio settle after disconnect */
        if(!first) vTaskDelay(pdMS_TO_TICKS(1500));
        first = false;
        if(sta_connected || s_manual_disconnect) continue;

        if(wifi_cred_count == 1){
            /* Only one network saved — connect directly, no scan needed */
            wifi_do_connect(0);
        } else {
            /* Multiple networks: scan first to find the strongest known AP,
               then connect to it. Scan runs async; manager sleeps until SCAN_DONE.
               If scan fails to start, connect directly to highest-priority credential. */
            wifi_scan_start_async();
            if(!s_scanning) wifi_do_connect(0);
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data){
    if(base == IP_EVENT && id == IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        esp_ip4addr_ntoa(&ev->ip_info.ip, sta_ip, sizeof(sta_ip));
        s_sta_connecting = false;
        sta_connected    = true;
        led_cmd(LED_CMD_CONNECTED);
        printf("STA connected, IP: %s\n", sta_ip);
    } else if(base == WIFI_EVENT && id == WIFI_EVENT_SCAN_DONE){
        s_scanning = false;
        s_scan_done = true;
        if(s_wifi_mgr_task) xTaskNotifyGive(s_wifi_mgr_task);
    } else if(base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED){
        sta_connected    = false;
        s_sta_connecting = false;
        sta_ip[0] = 0;
        if(s_suppress_disc > 0){ s_suppress_disc--; return; }
        if(s_wps_active) return;  /* WPS aborted connection; WPS events will reconnect */
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t*)data;
        printf("STA disconnected reason=%d\n", d ? (int)d->reason : -1);
        if(s_manual_disconnect){ s_manual_disconnect = false; led_cmd(LED_CMD_IDLE); return; }
        led_cmd(LED_CMD_IDLE);
        if(s_wifi_mgr_task) xTaskNotifyGive(s_wifi_mgr_task);
    } else if(base == WIFI_EVENT && id == WIFI_EVENT_STA_WPS_ER_SUCCESS){
        s_wps_active = false;
        esp_wifi_wps_disable();
        wifi_config_t sta_cfg = {};
        if(esp_wifi_get_config(WIFI_IF_STA, &sta_cfg) == ESP_OK && sta_cfg.sta.ssid[0]){
            printf("WPS: saved SSID=%s\n", sta_cfg.sta.ssid);
            creds_add((const char*)sta_cfg.sta.ssid, (const char*)sta_cfg.sta.password);
        }
        if(s_wifi_mgr_task) xTaskNotifyGive(s_wifi_mgr_task);
    } else if(base == WIFI_EVENT && (id == WIFI_EVENT_STA_WPS_ER_FAILED ||
                                     id == WIFI_EVENT_STA_WPS_ER_TIMEOUT)){
        s_wps_active = false;
        esp_wifi_wps_disable();
        printf("WPS: %s\n", id == WIFI_EVENT_STA_WPS_ER_TIMEOUT ? "timeout" : "failed");
        led_cmd(s_manual_disconnect ? LED_CMD_IDLE : (sta_connected ? LED_CMD_CONNECTED : LED_CMD_IDLE));
        if(s_wifi_mgr_task) xTaskNotifyGive(s_wifi_mgr_task);
    }
}

static void wps_start_pbc(void) {
    if (s_wps_active) return;
    printf("WPS: starting PBC\n");
    s_wps_active = true;
    led_cmd(LED_CMD_BOOT);   /* fast blink while searching */
    esp_wps_config_t wps_cfg = WPS_CONFIG_INIT_DEFAULT(WPS_TYPE_PBC);
    if (esp_wifi_wps_enable(&wps_cfg) != ESP_OK ||
        esp_wifi_wps_start() != ESP_OK) {
        printf("WPS: start failed\n");
        s_wps_active = false;
        led_cmd(LED_CMD_IDLE);
    }
}

/* Polls a GPIO for a long press (>= 3 s, active-low) and starts WPS PBC mode */
static void button_task(void *arg) {
    (void)arg;
    if (s_btn_pin < 0) { vTaskDelete(NULL); return; }
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_btn_pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    int held_ms = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(s_btn_pin) == 0) {   /* active-low: pressed */
            held_ms += 50;
            if (held_ms == 3000) {
                printf("WPS: long press\n");
                wps_start_pbc();
            }
        } else {
            held_ms = 0;
        }
    }
}

static void wifi_init_ap(void){
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta_netif, s_hostname);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,      wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP,   wifi_event_handler, NULL);

    wifi_config_t apcfg = {};
    strncpy((char*)apcfg.ap.ssid,     ap_ssid, sizeof(apcfg.ap.ssid)-1);
    strncpy((char*)apcfg.ap.password, ap_pass,  sizeof(apcfg.ap.password)-1);
    apcfg.ap.max_connection = 4;
    apcfg.ap.authmode = (strlen(ap_pass) >= 8) ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_AP, &apcfg);
    esp_wifi_start();

    creds_load();

    /* Start WiFi manager task; notify it immediately if we have saved credentials */
    xTaskCreate(wifi_manager_task, "wifi_mgr", 4096, NULL, 4, &s_wifi_mgr_task);
    if(wifi_cred_count > 0) xTaskNotifyGive(s_wifi_mgr_task);
    xTaskCreate(button_task, "btn", 2048, NULL, 3, NULL);
}

/* ---- HID keyboard helpers ---- */
static bool ascii_to_hid(char c, uint8_t *keycode, uint8_t *modifier) {
    *modifier = 0;
    *keycode  = 0;
    if (c >= 'a' && c <= 'z') { *keycode = HID_KEY_A + (c - 'a'); return true; }
    if (c >= 'A' && c <= 'Z') { *keycode = HID_KEY_A + (c - 'A'); *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true; }
    if (c >= '1' && c <= '9') { *keycode = HID_KEY_1 + (c - '1'); return true; }
    switch (c) {
        case '0':  *keycode = HID_KEY_0;             return true;
        case ' ':  *keycode = HID_KEY_SPACE;          return true;
        case '\n': *keycode = HID_KEY_ENTER;          return true;
        case '\t': *keycode = HID_KEY_TAB;            return true;
        case '-':  *keycode = HID_KEY_MINUS;          return true;
        case '=':  *keycode = HID_KEY_EQUAL;          return true;
        case '[':  *keycode = HID_KEY_BRACKET_LEFT;   return true;
        case ']':  *keycode = HID_KEY_BRACKET_RIGHT;  return true;
        case '\\': *keycode = HID_KEY_BACKSLASH;      return true;
        case ';':  *keycode = HID_KEY_SEMICOLON;      return true;
        case '\'': *keycode = HID_KEY_APOSTROPHE;     return true;
        case '`':  *keycode = HID_KEY_GRAVE;          return true;
        case ',':  *keycode = HID_KEY_COMMA;          return true;
        case '.':  *keycode = HID_KEY_PERIOD;         return true;
        case '/':  *keycode = HID_KEY_SLASH;          return true;
        case '!':  *keycode = HID_KEY_1;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '@':  *keycode = HID_KEY_2;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '#':  *keycode = HID_KEY_3;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '$':  *keycode = HID_KEY_4;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '%':  *keycode = HID_KEY_5;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '^':  *keycode = HID_KEY_6;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '&':  *keycode = HID_KEY_7;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '*':  *keycode = HID_KEY_8;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '(':  *keycode = HID_KEY_9;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case ')':  *keycode = HID_KEY_0;             *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '_':  *keycode = HID_KEY_MINUS;         *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '+':  *keycode = HID_KEY_EQUAL;         *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '{':  *keycode = HID_KEY_BRACKET_LEFT;  *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '}':  *keycode = HID_KEY_BRACKET_RIGHT; *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '|':  *keycode = HID_KEY_BACKSLASH;     *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case ':':  *keycode = HID_KEY_SEMICOLON;     *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '"':  *keycode = HID_KEY_APOSTROPHE;    *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '<':  *keycode = HID_KEY_COMMA;         *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '>':  *keycode = HID_KEY_PERIOD;        *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '?':  *keycode = HID_KEY_SLASH;         *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
        case '~':  *keycode = HID_KEY_GRAVE;         *modifier = KEYBOARD_MODIFIER_LEFTSHIFT; return true;
    }
    return false;
}

static uint8_t keyname_to_hid(const char *name) {
    if (!strcasecmp(name, "ENTER") || !strcasecmp(name, "RETURN")) return HID_KEY_ENTER;
    if (!strcasecmp(name, "ESC")   || !strcasecmp(name, "ESCAPE")) return HID_KEY_ESCAPE;
    if (!strcasecmp(name, "BACKSPACE"))              return HID_KEY_BACKSPACE;
    if (!strcasecmp(name, "TAB"))                    return HID_KEY_TAB;
    if (!strcasecmp(name, "SPACE"))                  return HID_KEY_SPACE;
    if (!strcasecmp(name, "DELETE"))                 return HID_KEY_DELETE;
    if (!strcasecmp(name, "INSERT"))                 return HID_KEY_INSERT;
    if (!strcasecmp(name, "HOME"))                   return HID_KEY_HOME;
    if (!strcasecmp(name, "END"))                    return HID_KEY_END;
    if (!strcasecmp(name, "PAGEUP"))                 return HID_KEY_PAGE_UP;
    if (!strcasecmp(name, "PAGEDOWN"))               return HID_KEY_PAGE_DOWN;
    if (!strcasecmp(name, "UP")    || !strcasecmp(name, "ARROWUP"))    return HID_KEY_ARROW_UP;
    if (!strcasecmp(name, "DOWN")  || !strcasecmp(name, "ARROWDOWN"))  return HID_KEY_ARROW_DOWN;
    if (!strcasecmp(name, "LEFT")  || !strcasecmp(name, "ARROWLEFT"))  return HID_KEY_ARROW_LEFT;
    if (!strcasecmp(name, "RIGHT") || !strcasecmp(name, "ARROWRIGHT")) return HID_KEY_ARROW_RIGHT;
    if (!strcasecmp(name, "CAPSLOCK"))               return HID_KEY_CAPS_LOCK;
    if (!strcasecmp(name, "PRINTSCREEN"))            return HID_KEY_PRINT_SCREEN;
    if (!strcasecmp(name, "SCROLLLOCK"))             return HID_KEY_SCROLL_LOCK;
    if (!strcasecmp(name, "PAUSE"))                  return HID_KEY_PAUSE;
    if (!strcasecmp(name, "NUMLOCK"))                return HID_KEY_NUM_LOCK;
    /* Punctuation / physical key positions */
    if (!strcasecmp(name, "MINUS"))                  return HID_KEY_MINUS;
    if (!strcasecmp(name, "EQUAL"))                  return HID_KEY_EQUAL;
    if (!strcasecmp(name, "BRACKETLEFT"))            return HID_KEY_BRACKET_LEFT;
    if (!strcasecmp(name, "BRACKETRIGHT"))           return HID_KEY_BRACKET_RIGHT;
    if (!strcasecmp(name, "BACKSLASH"))              return HID_KEY_BACKSLASH;
    if (!strcasecmp(name, "SEMICOLON"))              return HID_KEY_SEMICOLON;
    if (!strcasecmp(name, "APOSTROPHE") ||
        !strcasecmp(name, "QUOTE"))                  return HID_KEY_APOSTROPHE;
    if (!strcasecmp(name, "GRAVE") ||
        !strcasecmp(name, "BACKQUOTE"))              return HID_KEY_GRAVE;
    if (!strcasecmp(name, "COMMA"))                  return HID_KEY_COMMA;
    if (!strcasecmp(name, "PERIOD"))                 return HID_KEY_PERIOD;
    if (!strcasecmp(name, "SLASH"))                  return HID_KEY_SLASH;
    if (!strcasecmp(name, "INTLBACKSLASH"))          return HID_KEY_EUROPE_2;
    /* F1-F12 */
    if ((name[0] == 'F' || name[0] == 'f') && name[1] >= '1' && name[1] <= '9') {
        int fn = atoi(name + 1);
        if (fn >= 1 && fn <= 12) return HID_KEY_F1 + (fn - 1);
    }
    /* Single letter A-Z (key position, no shift) */
    if (name[0] && !name[1]) {
        char c = name[0];
        if (c >= 'A' && c <= 'Z') return HID_KEY_A + (c - 'A');
        if (c >= 'a' && c <= 'z') return HID_KEY_A + (c - 'a');
        if (c >= '1' && c <= '9') return HID_KEY_1 + (c - '1');
        if (c == '0') return HID_KEY_0;
    }
    return 0;
}

static uint8_t modname_to_modifier(const char *name) {
    if (!strcasecmp(name, "CTRL")   || !strcasecmp(name, "CONTROL") ||
        !strcasecmp(name, "LCTRL"))  return KEYBOARD_MODIFIER_LEFTCTRL;
    if (!strcasecmp(name, "RCTRL"))  return KEYBOARD_MODIFIER_RIGHTCTRL;
    if (!strcasecmp(name, "SHIFT")  || !strcasecmp(name, "LSHIFT")) return KEYBOARD_MODIFIER_LEFTSHIFT;
    if (!strcasecmp(name, "RSHIFT")) return KEYBOARD_MODIFIER_RIGHTSHIFT;
    if (!strcasecmp(name, "ALT")    || !strcasecmp(name, "LALT"))   return KEYBOARD_MODIFIER_LEFTALT;
    if (!strcasecmp(name, "RALT"))   return KEYBOARD_MODIFIER_RIGHTALT;
    if (!strcasecmp(name, "GUI") || !strcasecmp(name, "WIN") ||
        !strcasecmp(name, "SUPER") || !strcasecmp(name, "META")) return KEYBOARD_MODIFIER_LEFTGUI;
    return 0;
}

/* Wait for HID endpoint ready, up to timeout_ms */
static bool hid_wait_ready(int timeout_ms) {
    for (int i = 0; i < timeout_ms / 5; i++) {
        if (tud_hid_ready()) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return tud_hid_ready();
}

/* Modifiers held persistently (set via HOLDMOD command from UI) */
static uint8_t g_held_mods = 0;

static void hid_press_key(uint8_t modifier, uint8_t keycode) {
    /* Key-down: wait up to 200 ms */
    if (!hid_wait_ready(200)) {
        printf("[HID] key-down skipped: not ready\n");
        return;
    }
    led_key_invert(true);
    uint8_t keys[6] = {keycode, 0, 0, 0, 0, 0};
    tud_hid_keyboard_report(HID_REPORT_ID_KEYBOARD, modifier | g_held_mods, keys);
    vTaskDelay(pdMS_TO_TICKS(15));

    /* Key-up: restore held mods (not full release) so sticky mods stay active */
    for (int i = 0; i < 60; i++) {
        if (hid_wait_ready(50)) {
            if (tud_hid_keyboard_report(HID_REPORT_ID_KEYBOARD, g_held_mods, NULL)) break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    led_key_invert(false);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void hid_consumer_key(uint16_t usage) {
    if (!hid_wait_ready(200)) return;
    led_key_invert(true);
    tud_hid_report(HID_REPORT_ID_CONSUMER, (uint8_t *)&usage, 2);
    vTaskDelay(pdMS_TO_TICKS(15));
    /* Release: retry until it succeeds */
    uint16_t release = 0;
    for (int i = 0; i < 60; i++) {
        if (hid_wait_ready(50)) {
            if (tud_hid_report(HID_REPORT_ID_CONSUMER, (uint8_t *)&release, 2)) break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    led_key_invert(false);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static uint16_t medianame_to_usage(const char *name) {
    if (!strcasecmp(name, "PLAY_PAUSE") || !strcasecmp(name, "PLAYPAUSE") || !strcasecmp(name, "PLAY"))
        return HID_USAGE_CONSUMER_PLAY_PAUSE;
    if (!strcasecmp(name, "NEXT") || !strcasecmp(name, "NEXT_TRACK") || !strcasecmp(name, "FORWARD"))
        return HID_USAGE_CONSUMER_SCAN_NEXT_TRACK;
    if (!strcasecmp(name, "PREV") || !strcasecmp(name, "PREV_TRACK") || !strcasecmp(name, "BACKWARD") || !strcasecmp(name, "PREVIOUS"))
        return HID_USAGE_CONSUMER_SCAN_PREVIOUS_TRACK;
    if (!strcasecmp(name, "VOL_UP") || !strcasecmp(name, "VOLUME_UP") || !strcasecmp(name, "VOLUMEUP"))
        return HID_USAGE_CONSUMER_VOLUME_INCREMENT;
    if (!strcasecmp(name, "VOL_DOWN") || !strcasecmp(name, "VOLUME_DOWN") || !strcasecmp(name, "VOLUMEDOWN"))
        return HID_USAGE_CONSUMER_VOLUME_DECREMENT;
    if (!strcasecmp(name, "MUTE"))
        return HID_USAGE_CONSUMER_MUTE;
    if (!strcasecmp(name, "STOP"))
        return HID_USAGE_CONSUMER_STOP;
    return 0;
}

/* ---- HID command queue ---- */
#define HID_CMD_MAX    512
#define HID_QUEUE_DEPTH 32
typedef char hid_cmd_t[HID_CMD_MAX];
static QueueHandle_t s_hid_queue;

/* Enqueue a command string for the HID worker task (callable from any task) */
static void backend_send(const char *s) {
    hid_cmd_t cmd;
    strncpy(cmd, s, HID_CMD_MAX - 1);
    cmd[HID_CMD_MAX - 1] = 0;
    if (xQueueSend(s_hid_queue, cmd, pdMS_TO_TICKS(2000)) != pdTRUE) {
        printf("[HID] queue full, dropped: %.40s\n", s);
    }
}

/* Execute one HID command — called only from the HID worker task */
static void hid_exec(const char *s) {
    if (strncmp(s, "DELAY ", 6) == 0) {
        int ms = atoi(s + 6);
        if (ms > 0 && ms <= 30000) vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }
    if (strncmp(s, "MEDIA ", 6) == 0) {
        uint16_t usage = medianame_to_usage(s + 6);
        if (usage) hid_consumer_key(usage);
        else printf("[HID] Unknown media key: %s\n", s + 6);
        return;
    }
    if (strncmp(s, "HOLDMOD", 7) == 0) {
        uint8_t mods = 0;
        if (s[7] == ' ' && s[8]) {
            char tmp[64];
            strncpy(tmp, s + 8, sizeof(tmp) - 1);
            tmp[sizeof(tmp) - 1] = 0;
            char *save, *tok = strtok_r(tmp, "+", &save);
            while (tok) { mods |= modname_to_modifier(tok); tok = strtok_r(NULL, "+", &save); }
        }
        g_held_mods = mods;
        uint8_t empty[6] = {0};
        if (hid_wait_ready(200))
            tud_hid_keyboard_report(HID_REPORT_ID_KEYBOARD, g_held_mods, empty);
        return;
    }
    if (strncmp(s, "KEY ", 4) == 0) {
        uint8_t kc = keyname_to_hid(s + 4);
        if (kc) hid_press_key(0, kc);
        else printf("[HID] Unknown key: %s\n", s + 4);
    } else if (strncmp(s, "COMBO ", 6) == 0) {
        char combo[64];
        strncpy(combo, s + 6, sizeof(combo) - 1);
        combo[sizeof(combo) - 1] = 0;
        uint8_t modifier = 0, keycode = 0;
        char *saveptr;
        char *tok = strtok_r(combo, "+", &saveptr);
        while (tok) {
            while (*tok == ' ') tok++;
            uint8_t mod = modname_to_modifier(tok);
            if (mod) {
                modifier |= mod;
            } else {
                uint8_t kc = keyname_to_hid(tok);
                if (!kc && tok[0] && !tok[1]) {
                    uint8_t dummy_mod;
                    ascii_to_hid(tok[0], &kc, &dummy_mod);
                }
                if (kc) keycode = kc;
            }
            tok = strtok_r(NULL, "+", &saveptr);
        }
        if (keycode) hid_press_key(modifier, keycode);
    } else {
        /* Plain text: type character by character */
        for (const char *p = s; *p; p++) {
            uint8_t kc, mod;
            if (ascii_to_hid(*p, &kc, &mod)) {
                hid_press_key(mod, kc);
            }
        }
    }
}

static void hid_worker_task(void *arg) {
    (void)arg;
    hid_cmd_t cmd;
    for (;;) {
        if (xQueueReceive(s_hid_queue, cmd, portMAX_DELAY) == pdTRUE) {
            hid_exec(cmd);
        }
    }
}

static void run_macro_script(const char *script){
    char buf[512];
    strncpy(buf, script, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;

    char *saveptr;
    char *line = strtok_r(buf, "\n", &saveptr);
    while (line) {
        if (strncmp(line, "STRING ", 7) == 0) {
            backend_send(line + 7);
        } else if (strncmp(line, "KEY ",   4) == 0 ||
                   strncmp(line, "COMBO ", 6) == 0 ||
                   strncmp(line, "MEDIA ", 6) == 0 ||
                   strncmp(line, "DELAY ", 6) == 0) {
            backend_send(line);  /* HID task handles DELAY too */
        }
        line = strtok_r(NULL, "\n", &saveptr);
    }
}

/* Receive the complete POST body, returns bytes received (body is NUL-terminated) */
static int recv_body(httpd_req_t *req, char *buf, size_t buf_size) {
    int total = 0, remaining = (int)req->content_len;
    while (remaining > 0 && total < (int)buf_size - 1) {
        int n = httpd_req_recv(req, buf + total,
                               (size_t)(remaining < (int)(buf_size - total - 1)
                                        ? remaining : (int)(buf_size - total - 1)));
        if (n <= 0) break;
        total += n; remaining -= n;
    }
    buf[total] = 0;
    return total;
}

/* Send an HTML file, substituting %%VERSION%% with the VERSION string */
static esp_err_t send_html_versioned(httpd_req_t *req, const uint8_t *start, const uint8_t *end) {
    size_t src_len = end - start;
    char *buf = malloc(src_len + 1);
    if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
    memcpy(buf, start, src_len);
    buf[src_len] = 0;

    const char *ph = "%%VERSION%%";
    const size_t ph_len = strlen(ph);
    const char *ver = VERSION;
    const size_t ver_len = strlen(ver);
    char *p;
    while ((p = strstr(buf, ph)) != NULL) {
        memmove(p + ver_len, p + ph_len, strlen(p + ph_len) + 1);
        memcpy(p, ver, ver_len);
    }

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, buf, strlen(buf));
    free(buf);
    return ESP_OK;
}

static esp_err_t root_get(httpd_req_t *req){
    return send_html_versioned(req, index_html_start, index_html_end);
}

static esp_err_t send_post(httpd_req_t *req){
    char buf[512];
    recv_body(req, buf, sizeof(buf));
    backend_send(buf);
    httpd_resp_sendstr(req,"OK");
    return ESP_OK;
}

static esp_err_t macro_save(httpd_req_t *req){
    char buf[512];
    recv_body(req, buf, sizeof(buf));

    char *sep=strchr(buf,'|');
    if(!sep){ httpd_resp_sendstr(req,"ERR"); return ESP_OK; }
    *sep=0;
    char *name=buf;
    char *body=sep+1;

    int slot = -1;
    for(int i=0;i<MAX_MACROS;i++){
        if(macros[i].name[0]==0 || strcmp(macros[i].name,name)==0){ slot=i; break; }
    }
    if(slot < 0){ httpd_resp_sendstr(req,"ERR:FULL"); return ESP_OK; }
    strncpy(macros[slot].name, name, sizeof(macros[slot].name)-1);
    strncpy(macros[slot].body, body, sizeof(macros[slot].body)-1);
    if(!macros_save_one(slot)){ httpd_resp_sendstr(req,"ERR:NVS"); return ESP_OK; }
    httpd_resp_sendstr(req,"OK");
    return ESP_OK;
}

static esp_err_t macro_list(httpd_req_t *req){
    char out[2048];
    strcpy(out,"[");
    int first=1;
    for(int i=0;i<MAX_MACROS;i++){
        if(macros[i].name[0]){
            if(!first) strcat(out,",");
            first=0;
            strcat(out,"{\"id\":");
            char t[16]; sprintf(t,"%d",i); strcat(out,t);
            strcat(out,",\"name\":\"");
            strcat(out,macros[i].name);
            strcat(out,"\"}");
        }
    }
    strcat(out,"]");
    httpd_resp_set_type(req,"application/json");
    httpd_resp_sendstr(req,out);
    return ESP_OK;
}

static esp_err_t web_clients_get(httpd_req_t *req) {
    int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    /* collect unique IPs that are still active */
    uint32_t seen[MAX_WEB_CLIENTS]; int seen_n = 0;
    for (int i = 0; i < s_web_client_count; i++) {
        if (!s_web_clients[i].ip) continue;
        if (now - s_web_clients[i].last_ms >= WEB_CLIENT_TIMEOUT_MS) continue;
        bool dup = false;
        for (int j = 0; j < seen_n; j++) if (seen[j] == s_web_clients[i].ip) { dup = true; break; }
        if (!dup) seen[seen_n++] = s_web_clients[i].ip;
    }
    char buf[MAX_WEB_CLIENTS * 20 + 8];
    int pos = 0;
    buf[pos++] = '[';
    for (int i = 0; i < seen_n; i++) {
        struct in_addr a = { .s_addr = seen[i] };
        char ip_str[16];
        ip4addr_ntoa_r((const ip4_addr_t *)&a, ip_str, sizeof(ip_str));
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%s\"%s\"", i ? "," : "", ip_str);
    }
    buf[pos++] = ']'; buf[pos] = '\0';
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t status_get(httpd_req_t *req){
    track_web_client(req);
    /* Guard: event callbacks aren't always reliable on rapid cable removal */
    if (!tud_mounted() || tud_suspended()) hid_enumerated = false;
    wifi_sta_list_t sta;
    esp_wifi_ap_get_sta_list(&sta);
    char out[180];
    snprintf(out, sizeof(out), "{\"hid\":%s,\"clients\":%d,\"sta\":%s,\"ip\":\"%s\",\"leds\":%d,\"wps\":%s,\"web_clients\":%d}",
             hid_enumerated ? "true" : "false",
             sta.num,
             sta_connected ? "true" : "false",
             sta_ip,
             hid_led_state,
             s_wps_active ? "true" : "false",
             web_client_count_active());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t wifi_page_get(httpd_req_t *req){
    return send_html_versioned(req, wifi_html_start, wifi_html_end);
}

static esp_err_t wifi_disconnect_post(httpd_req_t *req){
    s_manual_disconnect = true;
    sta_connected = false;
    sta_ip[0] = 0;
    esp_wifi_disconnect();
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t wifi_connect_post(httpd_req_t *req){
    char buf[128];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if(len < 0) len = 0;
    buf[len] = 0;

    char *sep = strchr(buf, '|');
    if(!sep){ httpd_resp_sendstr(req, "ERR"); return ESP_OK; }
    *sep = 0;
    const char *ssid = buf, *pass = sep+1;

    creds_add(ssid, pass);

    int idx = 0;
    for(int i = 0; i < wifi_cred_count; i++)
        if(strcmp(wifi_creds[i].ssid, ssid) == 0){ idx = i; break; }
    wifi_do_connect(idx);

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t wifi_list_get(httpd_req_t *req){
    char out[1024];
    int pos = 0;
    pos += snprintf(out+pos, sizeof(out)-pos, "[");
    for(int i = 0; i < wifi_cred_count; i++){
        if(i) pos += snprintf(out+pos, sizeof(out)-pos, ",");
        pos += snprintf(out+pos, sizeof(out)-pos,
                        "{\"idx\":%d,\"ssid\":\"%s\"}", i, wifi_creds[i].ssid);
    }
    pos += snprintf(out+pos, sizeof(out)-pos, "]");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t wifi_delete_post(httpd_req_t *req){
    char buf[8];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if(len < 0) len = 0;
    buf[len] = 0;
    creds_delete(atoi(buf));
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t wifi_connect_idx_post(httpd_req_t *req){
    char buf[8];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if(len < 0) len = 0;
    buf[len] = 0;
    wifi_do_connect(atoi(buf));
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t wifi_move_post(httpd_req_t *req){
    char buf[16];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if(len < 0) len = 0;
    buf[len] = 0;
    char *sp, *is = strtok_r(buf, "|", &sp), *ds = strtok_r(NULL, "|", &sp);
    if(!is || !ds){ httpd_resp_sendstr(req, "ERR"); return ESP_OK; }
    creds_move(atoi(is), atoi(ds));
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t macro_run(httpd_req_t *req){
    char buf[8];
    recv_body(req, buf, sizeof(buf));
    int id=atoi(buf);
    if(id>=0 && id<MAX_MACROS && macros[id].name[0]){
        run_macro_script(macros[id].body);
    }
    httpd_resp_sendstr(req,"OK");
    return ESP_OK;
}

static esp_err_t macro_delete(httpd_req_t *req){
    char buf[8];
    recv_body(req, buf, sizeof(buf));
    int id=atoi(buf);
    if(id>=0 && id<MAX_MACROS && macros[id].name[0]){
        macros_delete_one(id);
        memset(&macros[id], 0, sizeof(macro_t));
    }
    httpd_resp_sendstr(req,"OK");
    return ESP_OK;
}

static esp_err_t wifi_scan_get(httpd_req_t *req){
    wifi_scan_config_t scan_cfg = {
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    esp_wifi_scan_start(&scan_cfg, true);

    uint16_t count = 20;
    wifi_ap_record_t *records = malloc(count * sizeof(wifi_ap_record_t));
    if(!records){ httpd_resp_sendstr(req, "[]"); return ESP_OK; }
    esp_wifi_scan_get_ap_records(&count, records);

    char *out = malloc(2048);
    if(!out){ free(records); httpd_resp_sendstr(req, "[]"); return ESP_OK; }

    int pos = 0;
    pos += snprintf(out+pos, 2048-pos, "[");
    for(int i = 0; i < count; i++){
        if(i) pos += snprintf(out+pos, 2048-pos, ",");
        char safe_ssid[67] = {0};
        int si = 0;
        for(int j = 0; records[i].ssid[j] && si < 64; j++){
            if(records[i].ssid[j] == '"') safe_ssid[si++] = '\\';
            safe_ssid[si++] = records[i].ssid[j];
        }
        pos += snprintf(out+pos, 2048-pos,
            "{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":%d}",
            safe_ssid, records[i].rssi, records[i].authmode);
    }
    pos += snprintf(out+pos, 2048-pos, "]");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    free(out);
    free(records);
    return ESP_OK;
}

static esp_err_t help_page_get(httpd_req_t *req) {
    return send_html_versioned(req, help_html_start, help_html_end);
}

static esp_err_t usb_page_get(httpd_req_t *req) {
    return send_html_versioned(req, usb_html_start, usb_html_end);
}

static esp_err_t usb_cfg_get_h(httpd_req_t *req) {
    char out[300];
    snprintf(out, sizeof(out),
             "{\"vid\":\"0x%04X\",\"pid\":\"0x%04X\",\"mfr\":\"%s\",\"product\":\"%s\",\"serial\":\"%s\"}",
             s_device_descriptor.idVendor, s_device_descriptor.idProduct,
             s_manufacturer, s_product, s_serial);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static void usb_cfg_apply(char *buf) {
    char *sp;
    char *vid_s  = strtok_r(buf,  "|", &sp);
    char *pid_s  = strtok_r(NULL, "|", &sp);
    char *mfr_s  = strtok_r(NULL, "|", &sp);
    char *prod_s = strtok_r(NULL, "|", &sp);
    char *ser_s  = strtok_r(NULL, "|", &sp);
    if (vid_s)  s_device_descriptor.idVendor  = (uint16_t)strtol(vid_s, NULL, 16);
    if (pid_s)  s_device_descriptor.idProduct = (uint16_t)strtol(pid_s, NULL, 16);
    if (mfr_s)  strncpy(s_manufacturer, mfr_s,  sizeof(s_manufacturer)-1);
    if (prod_s) strncpy(s_product,      prod_s,  sizeof(s_product)-1);
    if (ser_s)  strncpy(s_serial,       ser_s,   sizeof(s_serial)-1);
    usb_cfg_save();
}

static esp_err_t usb_cfg_post_h(httpd_req_t *req) {
    char buf[256];
    recv_body(req, buf, sizeof(buf));
    usb_cfg_apply(buf);
    httpd_resp_sendstr(req, "OK");
    schedule_restart();
    return ESP_OK;
}

static esp_err_t usb_cfg_save_h(httpd_req_t *req) {
    char buf[256];
    recv_body(req, buf, sizeof(buf));
    usb_cfg_apply(buf);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t ap_cfg_get_h(httpd_req_t *req) {
    char out[256];
    snprintf(out, sizeof(out), "{\"ssid\":\"%s\",\"pass\":\"%s\",\"hostname\":\"%s\"}",
             ap_ssid, ap_pass, s_hostname);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static esp_err_t ap_cfg_post_h(httpd_req_t *req) {
    char buf[128];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if (len < 0) len = 0;
    buf[len] = 0;
    /* format: ssid|password|hostname */
    char *sp;
    char *ssid_s = strtok_r(buf,  "|", &sp);
    char *pass_s = strtok_r(NULL, "|", &sp);
    char *host_s = strtok_r(NULL, "|", &sp);
    if (ssid_s) strncpy(ap_ssid,    ssid_s, sizeof(ap_ssid)-1);
    if (pass_s) strncpy(ap_pass,    pass_s, sizeof(ap_pass)-1);
    if (host_s && host_s[0]) strncpy(s_hostname, host_s, sizeof(s_hostname)-1);
    ap_cfg_save();
    httpd_resp_sendstr(req, "OK");
    schedule_restart();
    return ESP_OK;
}

/* ---- hardware monitor ---- */
static temperature_sensor_handle_t s_temp_sensor = NULL;
static adc_oneshot_unit_handle_t   s_adc1_handle  = NULL;

static const int s_valid_gpios[] = {
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,
    26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48
};
#define VALID_GPIO_COUNT ((int)(sizeof(s_valid_gpios)/sizeof(s_valid_gpios[0])))

static void hw_monitor_init(void) {
    temperature_sensor_config_t tsens_cfg = {
        .range_min = -10, .range_max = 80
    };
    if (temperature_sensor_install(&tsens_cfg, &s_temp_sensor) == ESP_OK)
        temperature_sensor_enable(s_temp_sensor);

    adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&adc_cfg, &s_adc1_handle) == ESP_OK) {
        adc_oneshot_chan_cfg_t ch_cfg = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,
            .atten    = ADC_ATTEN_DB_12
        };
        for (int ch = 0; ch <= 9; ch++)
            adc_oneshot_config_channel(s_adc1_handle, (adc_channel_t)ch, &ch_cfg);
    }
}

static esp_err_t hw_status_get(httpd_req_t *req) {
    float temp_c = 0.0f;
    if (s_temp_sensor) temperature_sensor_get_celsius(s_temp_sensor, &temp_c);

    char buf[512];
    int pos = snprintf(buf, sizeof(buf), "{\"temp\":%.1f,\"gpio\":[", temp_c);
    for (int i = 0; i < VALID_GPIO_COUNT; i++)
        pos += snprintf(buf + pos, sizeof(buf) - pos,
                        i ? ",%d" : "%d", gpio_get_level(s_valid_gpios[i]));
    pos += snprintf(buf + pos, sizeof(buf) - pos, "],\"adc\":[");
    for (int ch = 0; ch <= 9; ch++) {
        int raw = 0;
        if (s_adc1_handle) adc_oneshot_read(s_adc1_handle, (adc_channel_t)ch, &raw);
        pos += snprintf(buf + pos, sizeof(buf) - pos, ch ? ",%d" : "%d", raw);
    }
    snprintf(buf + pos, sizeof(buf) - pos, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t hw_page_get(httpd_req_t *req) {
    return send_html_versioned(req, hw_html_start, hw_html_end);
}

static esp_err_t monitor_page_get(httpd_req_t *req) {
    return send_html_versioned(req, monitor_html_start, monitor_html_end);
}

static esp_err_t hw_cfg_get_h(httpd_req_t *req) {
    char out[128];
    snprintf(out, sizeof(out),
             "{\"led_pin\":%d,\"led_neo\":%d,\"led_inv\":%d,\"btn_pin\":%d}",
             s_led_pin, s_led_neopixel ? 1 : 0, s_led_invert ? 1 : 0, s_btn_pin);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    return ESP_OK;
}

static void hw_cfg_apply(char *buf) {
    char *sp;
    char *led_s = strtok_r(buf,  "|", &sp);
    char *neo_s = strtok_r(NULL, "|", &sp);
    char *inv_s = strtok_r(NULL, "|", &sp);
    char *btn_s = strtok_r(NULL, "|", &sp);
    if (led_s) s_led_pin      = atoi(led_s);
    if (neo_s) s_led_neopixel = (atoi(neo_s) != 0);
    if (inv_s) s_led_invert   = (atoi(inv_s) != 0);
    if (btn_s) s_btn_pin      = atoi(btn_s);
    hw_cfg_save();
}

static esp_err_t hw_cfg_post_h(httpd_req_t *req) {
    char buf[64];
    recv_body(req, buf, sizeof(buf));
    hw_cfg_apply(buf);
    httpd_resp_sendstr(req, "OK");
    schedule_restart();
    return ESP_OK;
}

static esp_err_t hw_cfg_save_h(httpd_req_t *req) {
    char buf[64];
    recv_body(req, buf, sizeof(buf));
    hw_cfg_apply(buf);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t wifi_wps_post(httpd_req_t *req) {
    wps_start_pbc();
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* ---- OTA firmware update ---- */
static esp_err_t ota_page_get(httpd_req_t *req) {
    return send_html_versioned(req, ota_html_start, ota_html_end);
}

static esp_err_t ota_upload_post(httpd_req_t *req) {
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }
    esp_ota_handle_t ota = 0;
    if (esp_ota_begin(part, OTA_SIZE_UNKNOWN, &ota) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }
    char buf[4096];
    int remaining = req->content_len;
    while (remaining > 0) {
        int n = httpd_req_recv(req, buf, remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf));
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0) { esp_ota_abort(ota); return ESP_FAIL; }
        if (esp_ota_write(ota, buf, n) != ESP_OK) {
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }
        remaining -= n;
    }
    if (esp_ota_end(ota) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid image");
        return ESP_FAIL;
    }
    if (esp_ota_set_boot_partition(part) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "OK");
    schedule_restart();
    return ESP_OK;
}

/* ---- JSON string escape helper ---- */
/* Appends a JSON-escaped version of src into dst (dst must have room).
   Returns number of bytes written (not counting NUL). */
static int json_escape(char *dst, size_t dst_size, const char *src) {
    int out = 0;
    for (int i = 0; src[i] && out + 2 < (int)dst_size; i++) {
        unsigned char c = (unsigned char)src[i];
        if (c == '"' || c == '\\') {
            if (out + 3 >= (int)dst_size) break;
            dst[out++] = '\\'; dst[out++] = c;
        } else if (c == '\n') {
            if (out + 3 >= (int)dst_size) break;
            dst[out++] = '\\'; dst[out++] = 'n';
        } else if (c == '\r') {
            if (out + 3 >= (int)dst_size) break;
            dst[out++] = '\\'; dst[out++] = 'r';
        } else if (c == '\t') {
            if (out + 3 >= (int)dst_size) break;
            dst[out++] = '\\'; dst[out++] = 't';
        } else if (c < 0x20) {
            if (out + 7 >= (int)dst_size) break;
            out += snprintf(dst + out, dst_size - out, "\\u%04X", c);
        } else {
            dst[out++] = c;
        }
    }
    dst[out] = '\0';
    return out;
}

/* ---- /macro/export — returns JSON array of all macros ---- */
/* Worst-case per macro: name 31 chars * 6 (\\uXXXX) + body 511 chars * 6 + JSON overhead ~20 */
#define MACRO_JSON_MAX  (MAX_MACROS * (31*6 + 511*6 + 24) + 8)
static esp_err_t macro_export_get(httpd_req_t *req) {
    char *buf = malloc(MACRO_JSON_MAX);
    if (!buf) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM"); return ESP_FAIL; }
    int pos = 0;
    pos += snprintf(buf + pos, MACRO_JSON_MAX - pos, "[");
    int first = 1;
    for (int i = 0; i < MAX_MACROS; i++) {
        if (!macros[i].name[0]) continue;
        pos += snprintf(buf + pos, MACRO_JSON_MAX - pos, "%s{\"name\":\"", first ? "" : ",");
        pos += json_escape(buf + pos, MACRO_JSON_MAX - pos, macros[i].name);
        pos += snprintf(buf + pos, MACRO_JSON_MAX - pos, "\",\"body\":\"");
        pos += json_escape(buf + pos, MACRO_JSON_MAX - pos, macros[i].body);
        pos += snprintf(buf + pos, MACRO_JSON_MAX - pos, "\"}");
        first = 0;
    }
    pos += snprintf(buf + pos, MACRO_JSON_MAX - pos, "]");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"macros.json\"");
    httpd_resp_sendstr(req, buf);
    free(buf);
    return ESP_OK;
}

/* ---- /macro/import — receives JSON array, replaces all macros ---- */
/* Format: JSON array as produced by /macro/export.
   Parsing is minimal: relies on the exact format we write. */
static esp_err_t macro_import_post(httpd_req_t *req) {
    size_t buf_size = MACRO_JSON_MAX;
    char *buf = malloc(buf_size);
    if (!buf) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM"); return ESP_FAIL; }
    int len = recv_body(req, buf, buf_size);
    if (len <= 0) { free(buf); httpd_resp_sendstr(req, "ERR"); return ESP_OK; }
    buf[len] = '\0';

    /* Clear all macros first */
    for (int i = 0; i < MAX_MACROS; i++) {
        if (macros[i].name[0]) { macros_delete_one(i); memset(&macros[i], 0, sizeof(macro_t)); }
    }

    /* Simple tokeniser: find "name":"<val>" and "body":"<val>" pairs.
       We trust our own exported format (no embedded \" in practice but handle \n \r). */
    int slot = 0;
    char *p = buf;
    while (slot < MAX_MACROS) {
        char *nk = strstr(p, "\"name\":\"");
        if (!nk) break;
        nk += 8;
        char *ne = strchr(nk, '"');
        if (!ne) break;
        *ne = '\0';

        char *bk = strstr(ne + 1, "\"body\":\"");
        if (!bk) break;
        bk += 8;
        /* body may contain escaped sequences; find the closing unescaped " */
        char *be = bk;
        while (*be && !(*be == '"' && *(be-1) != '\\')) be++;
        if (!*be) break;
        *be = '\0';

        /* unescape \n \r \t \\ \" in body */
        char body_dec[sizeof(macros[0].body)];
        int di = 0;
        for (int si = 0; bk[si] && di < (int)sizeof(body_dec) - 1; si++) {
            if (bk[si] == '\\' && bk[si+1]) {
                si++;
                if      (bk[si] == 'n')  body_dec[di++] = '\n';
                else if (bk[si] == 'r')  body_dec[di++] = '\r';
                else if (bk[si] == 't')  body_dec[di++] = '\t';
                else                     body_dec[di++] = bk[si];
            } else {
                body_dec[di++] = bk[si];
            }
        }
        body_dec[di] = '\0';

        strncpy(macros[slot].name, nk,       sizeof(macros[slot].name) - 1);
        strncpy(macros[slot].body, body_dec,  sizeof(macros[slot].body) - 1);
        macros_save_one(slot);
        slot++;
        p = be + 1;
    }
    free(buf);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* ---- /settings/export — full backup JSON ---- */
#define SETTINGS_JSON_MAX  (MACRO_JSON_MAX + WIFI_CRED_MAX*(33*6+65*6+24) + 1024)
static esp_err_t settings_export_get(httpd_req_t *req) {
    char *buf = malloc(SETTINGS_JSON_MAX);
    if (!buf) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM"); return ESP_FAIL; }
    int pos = 0;
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "{");

    /* AP config */
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\"ap\":{\"ssid\":\"");
    pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, ap_ssid);
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\",\"pass\":\"");
    pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, ap_pass);
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\",\"hostname\":\"");
    pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, s_hostname);
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\"},");

    /* USB config */
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos,
                    "\"usb\":{\"vid\":\"0x%04X\",\"pid\":\"0x%04X\",\"mfr\":\"",
                    s_device_descriptor.idVendor, s_device_descriptor.idProduct);
    pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, s_manufacturer);
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\",\"product\":\"");
    pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, s_product);
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\",\"serial\":\"");
    pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, s_serial);
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\"},");

    /* HW config */
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos,
                    "\"hw\":{\"led_pin\":%d,\"led_neo\":%d,\"led_inv\":%d,\"btn_pin\":%d},",
                    s_led_pin, s_led_neopixel ? 1 : 0, s_led_invert ? 1 : 0, s_btn_pin);

    /* WiFi saved networks */
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\"wifi\":[");
    for (int i = 0; i < wifi_cred_count; i++) {
        pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "%s{\"ssid\":\"", i ? "," : "");
        pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, wifi_creds[i].ssid);
        pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\",\"pass\":\"");
        pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, wifi_creds[i].pass);
        pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\"}");
    }
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "],");

    /* Macros */
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\"macros\":[");
    int first = 1;
    for (int i = 0; i < MAX_MACROS; i++) {
        if (!macros[i].name[0]) continue;
        pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "%s{\"name\":\"", first ? "" : ",");
        pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, macros[i].name);
        pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\",\"body\":\"");
        pos += json_escape(buf + pos, SETTINGS_JSON_MAX - pos, macros[i].body);
        pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "\"}");
        first = 0;
    }
    pos += snprintf(buf + pos, SETTINGS_JSON_MAX - pos, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"adminkbd-backup.json\"");
    httpd_resp_sendstr(req, buf);
    free(buf);
    return ESP_OK;
}

/* ---- /settings/import — full restore ---- */
/* Receives the same JSON produced by /settings/export.
   Parses top-level keys; WiFi networks are appended (not replaced) to avoid
   locking the user out. All other settings overwrite current values and are
   saved to NVS. A restart is required for AP/USB/HW to take effect. */
static esp_err_t settings_import_post(httpd_req_t *req) {
    size_t buf_size = SETTINGS_JSON_MAX;
    char *buf = malloc(buf_size);
    if (!buf) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM"); return ESP_FAIL; }
    int len = recv_body(req, buf, buf_size);
    if (len <= 0) { free(buf); httpd_resp_sendstr(req, "ERR"); return ESP_OK; }
    buf[len] = '\0';

    /* --- AP config --- */
    char *ap = strstr(buf, "\"ap\":{");
    if (ap) {
        char *s = strstr(ap, "\"ssid\":\""); if (s) { s+=8; char *e=strchr(s,'"'); if(e){*e=0; strncpy(ap_ssid,s,sizeof(ap_ssid)-1); *e='"';} }
        char *q = strstr(ap, "\"pass\":\""); if (q) { q+=8; char *e=strchr(q,'"'); if(e){*e=0; strncpy(ap_pass,q,sizeof(ap_pass)-1); *e='"';} }
        char *h = strstr(ap, "\"hostname\":\""); if (h) { h+=12; char *e=strchr(h,'"'); if(e){*e=0; if(h[0]) strncpy(s_hostname,h,sizeof(s_hostname)-1); *e='"';} }
        ap_cfg_save();
    }

    /* --- USB config --- */
    char *usb = strstr(buf, "\"usb\":{");
    if (usb) {
        char *v = strstr(usb, "\"vid\":\""); if (v) { v+=7; char *e=strchr(v,'"'); if(e){*e=0; s_device_descriptor.idVendor=(uint16_t)strtol(v,NULL,16); *e='"';} }
        char *pi = strstr(usb, "\"pid\":\""); if (pi) { pi+=7; char *e=strchr(pi,'"'); if(e){*e=0; s_device_descriptor.idProduct=(uint16_t)strtol(pi,NULL,16); *e='"';} }
        char *m = strstr(usb, "\"mfr\":\""); if (m) { m+=7; char *e=strchr(m,'"'); if(e){*e=0; strncpy(s_manufacturer,m,sizeof(s_manufacturer)-1); *e='"';} }
        char *pr = strstr(usb, "\"product\":\""); if (pr) { pr+=11; char *e=strchr(pr,'"'); if(e){*e=0; strncpy(s_product,pr,sizeof(s_product)-1); *e='"';} }
        char *sr = strstr(usb, "\"serial\":\""); if (sr) { sr+=10; char *e=strchr(sr,'"'); if(e){*e=0; strncpy(s_serial,sr,sizeof(s_serial)-1); *e='"';} }
        usb_cfg_save();
    }

    /* --- HW config --- */
    char *hw = strstr(buf, "\"hw\":{");
    if (hw) {
        char *lp = strstr(hw, "\"led_pin\":"); if (lp) { lp+=10; s_led_pin=atoi(lp); }
        char *ln = strstr(hw, "\"led_neo\":"); if (ln) { ln+=10; s_led_neopixel=(atoi(ln)!=0); }
        char *li = strstr(hw, "\"led_inv\":"); if (li) { li+=10; s_led_invert=(atoi(li)!=0); }
        char *bp = strstr(hw, "\"btn_pin\":"); if (bp) { bp+=10; s_btn_pin=atoi(bp); }
        hw_cfg_save();
    }

    /* --- WiFi networks --- */
    char *wf = strstr(buf, "\"wifi\":[");
    if (wf) {
        wf += 8;
        char *p = wf;
        while (1) {
            char *s = strstr(p, "\"ssid\":\""); if (!s) break;
            s += 8; char *se = strchr(s, '"'); if (!se) break; *se = 0;
            char *q = strstr(se+1, "\"pass\":\""); if (!q) { *se='"'; break; }
            q += 8; char *qe = strchr(q, '"'); if (!qe) { *se='"'; break; } *qe = 0;
            creds_add(s, q);
            *se = '"'; *qe = '"';
            p = qe + 1;
        }
    }

    /* --- Macros --- */
    char *mk = strstr(buf, "\"macros\":[");
    if (mk) {
        /* Clear all existing macros */
        for (int i = 0; i < MAX_MACROS; i++) {
            if (macros[i].name[0]) { macros_delete_one(i); memset(&macros[i], 0, sizeof(macro_t)); }
        }
        int slot = 0;
        char *p = mk + 10;
        while (slot < MAX_MACROS) {
            char *nk = strstr(p, "\"name\":\""); if (!nk) break;
            nk += 8; char *ne = strchr(nk, '"'); if (!ne) break; *ne = '\0';
            char *bk = strstr(ne+1, "\"body\":\""); if (!bk) break;
            bk += 8;
            char *be = bk;
            while (*be && !(*be == '"' && *(be-1) != '\\')) be++;
            if (!*be) break;
            *be = '\0';
            /* unescape body */
            char body_dec[sizeof(macros[0].body)];
            int di = 0;
            for (int si = 0; bk[si] && di < (int)sizeof(body_dec)-1; si++) {
                if (bk[si]=='\\' && bk[si+1]) { si++;
                    if(bk[si]=='n') body_dec[di++]='\n';
                    else if(bk[si]=='r') body_dec[di++]='\r';
                    else if(bk[si]=='t') body_dec[di++]='\t';
                    else body_dec[di++]=bk[si];
                } else body_dec[di++]=bk[si];
            }
            body_dec[di]='\0';
            strncpy(macros[slot].name, nk,      sizeof(macros[slot].name)-1);
            strncpy(macros[slot].body, body_dec, sizeof(macros[slot].body)-1);
            macros_save_one(slot);
            slot++; p = be+1;
        }
    }

    free(buf);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static void web_start(void){
    httpd_handle_t server=NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 34;
    cfg.open_fn  = httpd_open_fn;
    cfg.close_fn = httpd_close_fn;
    httpd_start(&server,&cfg);

    httpd_uri_t u1={.uri="/",.method=HTTP_GET,.handler=root_get};
    httpd_uri_t u2={.uri="/send",.method=HTTP_POST,.handler=send_post};
    httpd_uri_t u3={.uri="/macro/save",.method=HTTP_POST,.handler=macro_save};
    httpd_uri_t u4={.uri="/macro/list",.method=HTTP_GET,.handler=macro_list};
    httpd_uri_t u5={.uri="/macro/run",.method=HTTP_POST,.handler=macro_run};
    httpd_uri_t u5b={.uri="/macro/delete",.method=HTTP_POST,.handler=macro_delete};
    httpd_uri_t u6={.uri="/status",.method=HTTP_GET,.handler=status_get};
    httpd_uri_t u7={.uri="/wifi/connect",.method=HTTP_POST,.handler=wifi_connect_post};
    httpd_uri_t u8 ={.uri="/wifi",.method=HTTP_GET,.handler=wifi_page_get};
    httpd_uri_t u9 ={.uri="/wifi/disconnect",.method=HTTP_POST,.handler=wifi_disconnect_post};
    httpd_uri_t u10={.uri="/wifi/list",.method=HTTP_GET,.handler=wifi_list_get};
    httpd_uri_t u11={.uri="/wifi/delete",.method=HTTP_POST,.handler=wifi_delete_post};
    httpd_uri_t u12={.uri="/wifi/connect_idx",.method=HTTP_POST,.handler=wifi_connect_idx_post};
    httpd_uri_t u12b={.uri="/wifi/move",.method=HTTP_POST,.handler=wifi_move_post};
    httpd_uri_t u13={.uri="/wifi/scan",.method=HTTP_GET,.handler=wifi_scan_get};
    httpd_uri_t u13b={.uri="/usb",.method=HTTP_GET,.handler=usb_page_get};
    httpd_uri_t u14={.uri="/usb/config",.method=HTTP_GET,.handler=usb_cfg_get_h};
    httpd_uri_t u15={.uri="/usb/config",.method=HTTP_POST,.handler=usb_cfg_post_h};
    httpd_uri_t u15b={.uri="/usb/config/save",.method=HTTP_POST,.handler=usb_cfg_save_h};
    httpd_uri_t u16={.uri="/ap/config",.method=HTTP_GET,.handler=ap_cfg_get_h};
    httpd_uri_t u17={.uri="/ap/config",.method=HTTP_POST,.handler=ap_cfg_post_h};
    httpd_uri_t u18={.uri="/help",.method=HTTP_GET,.handler=help_page_get};
    httpd_uri_t u19={.uri="/wifi/wps",.method=HTTP_POST,.handler=wifi_wps_post};
    httpd_uri_t u20={.uri="/hw",.method=HTTP_GET,.handler=hw_page_get};
    httpd_uri_t u21={.uri="/hw/config",.method=HTTP_GET,.handler=hw_cfg_get_h};
    httpd_uri_t u22={.uri="/hw/config",.method=HTTP_POST,.handler=hw_cfg_post_h};
    httpd_uri_t u23={.uri="/hw/config/save",.method=HTTP_POST,.handler=hw_cfg_save_h};
    httpd_uri_t u24={.uri="/hw/status",.method=HTTP_GET,.handler=hw_status_get};
    httpd_uri_t u25={.uri="/monitor",.method=HTTP_GET,.handler=monitor_page_get};
    httpd_uri_t u26={.uri="/web/clients",.method=HTTP_GET,.handler=web_clients_get};
    httpd_uri_t u27={.uri="/ota",.method=HTTP_GET,.handler=ota_page_get};
    httpd_uri_t u28={.uri="/ota/upload",.method=HTTP_POST,.handler=ota_upload_post};
    httpd_uri_t u29={.uri="/macro/export",.method=HTTP_GET,.handler=macro_export_get};
    httpd_uri_t u30={.uri="/macro/import",.method=HTTP_POST,.handler=macro_import_post};
    httpd_uri_t u31={.uri="/settings/export",.method=HTTP_GET,.handler=settings_export_get};
    httpd_uri_t u32={.uri="/settings/import",.method=HTTP_POST,.handler=settings_import_post};

    httpd_register_uri_handler(server,&u1);
    httpd_register_uri_handler(server,&u2);
    httpd_register_uri_handler(server,&u3);
    httpd_register_uri_handler(server,&u4);
    httpd_register_uri_handler(server,&u5);
    httpd_register_uri_handler(server,&u5b);
    httpd_register_uri_handler(server,&u6);
    httpd_register_uri_handler(server,&u7);
    httpd_register_uri_handler(server,&u8);
    httpd_register_uri_handler(server,&u9);
    httpd_register_uri_handler(server,&u10);
    httpd_register_uri_handler(server,&u11);
    httpd_register_uri_handler(server,&u12);
    httpd_register_uri_handler(server,&u12b);
    httpd_register_uri_handler(server,&u13);
    httpd_register_uri_handler(server,&u13b);
    httpd_register_uri_handler(server,&u14);
    httpd_register_uri_handler(server,&u15);
    httpd_register_uri_handler(server,&u15b);
    httpd_register_uri_handler(server,&u16);
    httpd_register_uri_handler(server,&u17);
    httpd_register_uri_handler(server,&u18);
    httpd_register_uri_handler(server,&u19);
    httpd_register_uri_handler(server,&u20);
    httpd_register_uri_handler(server,&u21);
    httpd_register_uri_handler(server,&u22);
    httpd_register_uri_handler(server,&u23);
    httpd_register_uri_handler(server,&u24);
    httpd_register_uri_handler(server,&u25);
    httpd_register_uri_handler(server,&u26);
    httpd_register_uri_handler(server,&u27);
    httpd_register_uri_handler(server,&u28);
    httpd_register_uri_handler(server,&u29);
    httpd_register_uri_handler(server,&u30);
    httpd_register_uri_handler(server,&u31);
    httpd_register_uri_handler(server,&u32);
}

static void usb_hid_init(void) {
    tinyusb_config_t tusb_cfg = TINYUSB_CONFIG_FULL_SPEED(usb_event_cb, NULL);
    tusb_cfg.descriptor.device           = &s_device_descriptor;
    tusb_cfg.descriptor.string           = s_string_descriptor;
    tusb_cfg.descriptor.string_count     = sizeof(s_string_descriptor) / sizeof(s_string_descriptor[0]);
    tusb_cfg.descriptor.full_speed_config = s_hid_config_descriptor;
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    printf("USB HID initialized\n");
}

void app_main(void){
    /* NVS must be initialized before anything reads from it */
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        nvs_flash_erase();
        nvs_flash_init();
    }
    ap_cfg_load();
    usb_cfg_load();
    hw_cfg_load();
    macros_load();

    /* LED task starts immediately in boot-blink mode */
    s_led_queue = xQueueCreate(8, sizeof(uint8_t));
    xTaskCreate(led_task, "led", 2048, NULL, 3, NULL);

    /* Create HID command queue and worker task BEFORE USB init */
    s_hid_queue = xQueueCreate(HID_QUEUE_DEPTH, sizeof(hid_cmd_t));
    xTaskCreate(hid_worker_task, "hid_worker", 4096, NULL, 5, NULL);

    esp_ota_mark_app_valid_cancel_rollback();
    hw_monitor_init();
    usb_hid_init();
    wifi_init_ap();
    web_start();
    led_cmd(LED_CMD_IDLE);   /* boot complete; WiFi manager will flash on connect attempts */
    printf("AP: %s  PASS:%s\n", ap_ssid, ap_pass);
}
