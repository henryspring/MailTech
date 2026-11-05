#include "esp_random.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "cloud.h"
#include "psa/crypto.h"
#include "nvs.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "device_claim";
static char device_token[65];
static bool initialized = false;

esp_err_t http_request_init(void)
{
    if (initialized)
        return ESP_OK;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("mailbox_cloud", NVS_READWRITE, &nvs);
    if (err != ESP_OK)
        return err;
    size_t length = sizeof(device_token);
    err = nvs_get_str(nvs, "device_token", device_token, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "Creating device identity in NVS (credential redacted)");
        uint8_t bytes[32];
        esp_fill_random(bytes, sizeof(bytes));
        for (size_t i = 0; i < sizeof(bytes); ++i)
            snprintf(device_token + 2 * i, 3, "%02x", bytes[i]);
        err = nvs_set_str(nvs, "device_token", device_token);
        if (err == ESP_OK)
            err = nvs_commit(nvs);
    }
    else if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Loaded device identity from NVS (credential redacted)");
        if (length != 65 || strspn(device_token, "0123456789abcdef") != 64)
            err = ESP_ERR_INVALID_STATE;
    }
    nvs_close(nvs);
    if (err != ESP_OK)
        return err;
    if (strlen(CONFIG_MAILBOX_ENROLLMENT_TOKEN) < 32) {
        ESP_LOGE(TAG, "Enrollment signing secret is missing or shorter than 32 characters");
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Claim identity and signing configuration ready");
    initialized = true;
    return ESP_OK;
}

esp_err_t http_request_make_claim(const char *nonce, char *out, size_t out_size)
{
    if (!initialized || !nonce || strlen(nonce) != 32 || !out)
        return ESP_ERR_INVALID_STATE;
    uint8_t mac[6];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (err != ESP_OK)
        return err;
    char id[12], message[160];
    snprintf(id, sizeof(id), "MB-%02X%02X", mac[4], mac[5]);
    int n = snprintf(message, sizeof(message), "letterpeek-claim-v1|%s|%s|%s", id, device_token, nonce);
    if (n < 0 || n >= (int)sizeof(message))
        return ESP_ERR_INVALID_SIZE;

    uint8_t signature[32];
    size_t signature_len = 0;

    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attributes, PSA_ALG_HMAC(PSA_ALG_SHA_256));
    psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);

    psa_key_id_t key_id = 0;

    psa_status_t status = psa_import_key(
        &attributes,
        (const uint8_t *)CONFIG_MAILBOX_ENROLLMENT_TOKEN,
        strlen(CONFIG_MAILBOX_ENROLLMENT_TOKEN),
        &key_id);

    psa_reset_key_attributes(&attributes);

    if (status != PSA_SUCCESS)
    {
        return ESP_FAIL;
    }

    status = psa_mac_compute(
        key_id,
        PSA_ALG_HMAC(PSA_ALG_SHA_256),
        (const uint8_t *)message,
        strlen(message),
        signature,
        sizeof(signature),
        &signature_len);

    psa_status_t cleanup_status = psa_destroy_key(key_id);

    if (status != PSA_SUCCESS ||
        cleanup_status != PSA_SUCCESS ||
        signature_len != sizeof(signature))
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "HMAC-SHA256 proof computed successfully (contents redacted)");
    char proof[65];
    for (size_t i = 0; i < sizeof(signature); i++)
        snprintf(proof + 2 * i, 3, "%02x", signature[i]);
    n = snprintf(out, out_size, "{\"mailboxId\":\"%s\",\"deviceToken\":\"%s\",\"nonce\":\"%s\",\"proof\":\"%s\"}", id,
                 device_token, nonce, proof);
    return (n < 0 || (size_t)n >= out_size) ? ESP_ERR_INVALID_SIZE : ESP_OK;
}
