
idf_component_register(
SRCS "main.c"
INCLUDE_DIRS "."
EMBED_FILES "index.html" "wifi.html" "usb.html" "help.html" "hw.html" "monitor.html" "ota.html"
REQUIRES esp_tinyusb esp_wifi esp_netif esp_event nvs_flash esp_http_server driver esp_driver_rmt wpa_supplicant esp_adc esp_driver_tsens app_update
)
