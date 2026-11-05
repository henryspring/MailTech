idf_component_register(
    SRCS "receive.c" "cloud.c" "main.c" "provisioning.c" "receive.c"
    INCLUDE_DIRS "."
    REQUIRES esp_http_client esp_wifi nvs_flash mbedtls esp_event esp_netif esp_system
    PRIV_REQUIRES network_provisioning)
