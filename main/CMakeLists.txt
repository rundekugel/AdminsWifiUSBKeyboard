
idf_component_register(
SRCS "main.c"
INCLUDE_DIRS "."
EMBED_FILES "index.html" "wifi.html"
REQUIRES esp_tinyusb esp_wifi esp_netif esp_event nvs_flash esp_http_server
)
