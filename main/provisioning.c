/* Wi-Fi Provisioning Manager Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "http_request.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <esp_event.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs_flash.h>

#include <network_provisioning/manager.h>

#include <network_provisioning/scheme_ble.h>

static const char *TAG = "provisioning";

#define MAX_RETRIES 5

// #define EXAMPLE_PROV_SEC2_USERNAME "wifiprov"
// #define EXAMPLE_PROV_SEC2_PWD "abcd1234"

/* Signal Wi-Fi events on this event-group */
const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;

#define PROV_TRANSPORT_BLE "ble"

static const char sec2_salt[] = {0x65, 0xeb, 0x2a, 0x6c, 0xd6, 0xde, 0x29, 0xc2,
                                 0xdc, 0xc6, 0xb9, 0xd9, 0xb6, 0x71, 0x90, 0x1e};

static const char sec2_verifier[] = {
    0x9f, 0x3c, 0xc5, 0x2f, 0x35, 0xba, 0x98, 0xac, 0x59, 0x56, 0x8d, 0xef, 0x92, 0x83, 0xa9, 0x1b, 0xa2, 0x46, 0x0f,
    0x3a, 0x07, 0x57, 0xa7, 0x3f, 0x2b, 0x62, 0x07, 0x96, 0x4f, 0x59, 0x29, 0x78, 0xce, 0x41, 0x59, 0xbc, 0x60, 0x8f,
    0xfe, 0x63, 0x31, 0x11, 0x6e, 0x7e, 0xa0, 0xed, 0xa3, 0xfa, 0x9e, 0xe9, 0xe6, 0x81, 0x41, 0xa8, 0xc9, 0xf7, 0x17,
    0x1a, 0xab, 0xdb, 0x71, 0x34, 0x71, 0x57, 0x94, 0x4a, 0x3f, 0xcc, 0x25, 0xbf, 0xae, 0x04, 0x6e, 0x87, 0x9a, 0xb2,
    0x6d, 0x71, 0x00, 0x33, 0xef, 0x6d, 0x15, 0xd5, 0xd5, 0x01, 0x1a, 0xcb, 0xd8, 0x0c, 0x45, 0x0a, 0x8b, 0xb6, 0xd1,
    0xb0, 0x12, 0x0d, 0x9d, 0xd1, 0x11, 0xdd, 0x40, 0x90, 0xc9, 0x00, 0x2a, 0x97, 0x55, 0xfb, 0xd4, 0xd3, 0x14, 0x15,
    0xf2, 0xde, 0x9e, 0x2e, 0x28, 0xe8, 0x04, 0xb3, 0xb9, 0x03, 0x61, 0x40, 0xed, 0xbe, 0x79, 0xaf, 0x17, 0x2e, 0xf0,
    0xcc, 0xf7, 0x42, 0xe7, 0xe1, 0xb3, 0x56, 0x13, 0xf6, 0xdc, 0xee, 0xf5, 0x93, 0x1c, 0x4d, 0x67, 0x42, 0x71, 0xd7,
    0xa3, 0x7d, 0x9b, 0x09, 0x60, 0x33, 0xbe, 0x0f, 0xc4, 0xd5, 0xe8, 0x90, 0xb4, 0x0c, 0xf7, 0x60, 0x29, 0x5e, 0x07,
    0x7e, 0x1a, 0x65, 0xff, 0xec, 0xa3, 0xe2, 0xba, 0x92, 0xdc, 0x6a, 0xc0, 0x53, 0x93, 0x31, 0x5e, 0xa1, 0x1f, 0xfb,
    0x72, 0x36, 0xf4, 0xb3, 0x92, 0x21, 0x3b, 0x4b, 0xd2, 0x36, 0xd1, 0xcc, 0xef, 0x96, 0x6b, 0x10, 0x4b, 0xb4, 0xb7,
    0xe5, 0x83, 0xf1, 0x9c, 0xcd, 0xb3, 0xc4, 0x0e, 0x1b, 0xb2, 0xdb, 0x77, 0x60, 0xeb, 0xef, 0x58, 0x4e, 0xa7, 0xec,
    0x1a, 0x23, 0x32, 0x45, 0x11, 0xc9, 0x3b, 0xcf, 0x98, 0x3e, 0xa2, 0x63, 0xba, 0xe2, 0xce, 0x29, 0xfe, 0x56, 0xbf,
    0x41, 0xfa, 0x01, 0xb6, 0x8c, 0x87, 0x63, 0xa5, 0x98, 0x05, 0xc7, 0x8a, 0x94, 0xf9, 0x94, 0x98, 0x20, 0x95, 0x41,
    0x91, 0xf2, 0xc3, 0x3c, 0x47, 0x99, 0x8d, 0xd7, 0xb8, 0x5b, 0x97, 0x75, 0x4d, 0x40, 0x65, 0xd1, 0xaf, 0xb5, 0x57,
    0xff, 0x91, 0xd7, 0x25, 0x16, 0xfc, 0xb2, 0x20, 0x87, 0x81, 0xf8, 0xef, 0x60, 0xe6, 0x82, 0xe2, 0x41, 0xbd, 0x97,
    0x4b, 0xe6, 0x19, 0x06, 0xcd, 0xb5, 0xbc, 0xc6, 0x99, 0xbb, 0x58, 0xfe, 0x66, 0xd1, 0xd9, 0x16, 0xa3, 0x50, 0xb2,
    0xd9, 0x0d, 0xc9, 0x73, 0x4d, 0x5f, 0x06, 0xe6, 0xb1, 0x0a, 0x29, 0x0b, 0x8e, 0xf9, 0xf2, 0x64, 0xdb, 0x11, 0x2c,
    0xc9, 0x33, 0x5d, 0xf1, 0x4f, 0x61, 0x67, 0x73, 0x5a, 0x1b, 0xa1, 0xef, 0x7c, 0x3a, 0xd1, 0xe4, 0x64, 0x40, 0x70,
    0x5d, 0x15, 0x89, 0x2d, 0xf5, 0xfb, 0xbd, 0xcb, 0x52, 0xaa, 0x46, 0x50, 0x8d, 0xd7, 0x1c, 0xbf, 0x3e, 0xef, 0x9d,
    0x42, 0xbb, 0xcd, 0x44};

/* This salt,verifier has been generated for username = "wifiprov" and password = "abcd1234"
 * IMPORTANT NOTE: For production cases, this must be unique to every device
 * and should come from device manufacturing partition.*/
// static const char sec2_salt[] = {0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8,
//                                  0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4};

// static const char sec2_verifier[] = {
//     0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43, 0x78, 0xcf, 0xfd,
//     0x61, 0x6c, 0x59, 0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24, 0xa4, 0x70, 0x26, 0x1c, 0xdf, 0xa9,
//     0x03, 0xc2, 0xb2, 0x70, 0xe7, 0xb1, 0x32, 0x24, 0xda, 0x11, 0x1d, 0x97, 0x18, 0xdc, 0x60, 0x72, 0x08, 0xcc, 0x9a,
//     0xc9, 0x0c, 0x48, 0x27, 0xe2, 0xae, 0x89, 0xaa, 0x16, 0x25, 0xb8, 0x04, 0xd2, 0x1a, 0x9b, 0x3a, 0x8f, 0x37, 0xf6,
//     0xe4, 0x3a, 0x71, 0x2e, 0xe1, 0x27, 0x86, 0x6e, 0xad, 0xce, 0x28, 0xff, 0x54, 0x46, 0x60, 0x1f, 0xb9, 0x96, 0x87,
//     0xdc, 0x57, 0x40, 0xa7, 0xd4, 0x6c, 0xc9, 0x77, 0x54, 0xdc, 0x16, 0x82, 0xf0, 0xed, 0x35, 0x6a, 0xc4, 0x70, 0xad,
//     0x3d, 0x90, 0xb5, 0x81, 0x94, 0x70, 0xd7, 0xbc, 0x65, 0xb2, 0xd5, 0x18, 0xe0, 0x2e, 0xc3, 0xa5, 0xf9, 0x68, 0xdd,
//     0x64, 0x7b, 0xb8, 0xb7, 0x3c, 0x9c, 0xfc, 0x00, 0xd8, 0x71, 0x7e, 0xb7, 0x9a, 0x7c, 0xb1, 0xb7, 0xc2, 0xc3, 0x18,
//     0x34, 0x29, 0x32, 0x43, 0x3e, 0x00, 0x99, 0xe9, 0x82, 0x94, 0xe3, 0xd8, 0x2a, 0xb0, 0x96, 0x29, 0xb7, 0xdf, 0x0e,
//     0x5f, 0x08, 0x33, 0x40, 0x76, 0x52, 0x91, 0x32, 0x00, 0x9f, 0x97, 0x2c, 0x89, 0x6c, 0x39, 0x1e, 0xc8, 0x28, 0x05,
//     0x44, 0x17, 0x3f, 0x68, 0x02, 0x8a, 0x9f, 0x44, 0x61, 0xd1, 0xf5, 0xa1, 0x7e, 0x5a, 0x70, 0xd2, 0xc7, 0x23, 0x81,
//     0xcb, 0x38, 0x68, 0xe4, 0x2c, 0x20, 0xbc, 0x40, 0x57, 0x76, 0x17, 0xbd, 0x08, 0xb8, 0x96, 0xbc, 0x26, 0xeb, 0x32,
//     0x46, 0x69, 0x35, 0x05, 0x8c, 0x15, 0x70, 0xd9, 0x1b, 0xe9, 0xbe, 0xcc, 0xa9, 0x38, 0xa6, 0x67, 0xf0, 0xad, 0x50,
//     0x13, 0x19, 0x72, 0x64, 0xbf, 0x52, 0xc2, 0x34, 0xe2, 0x1b, 0x11, 0x79, 0x74, 0x72, 0xbd, 0x34, 0x5b, 0xb1, 0xe2,
//     0xfd, 0x66, 0x73, 0xfe, 0x71, 0x64, 0x74, 0xd0, 0x4e, 0xbc, 0x51, 0x24, 0x19, 0x40, 0x87, 0x0e, 0x92, 0x40, 0xe6,
//     0x21, 0xe7, 0x2d, 0x4e, 0x37, 0x76, 0x2f, 0x2e, 0xe2, 0x68, 0xc7, 0x89, 0xe8, 0x32, 0x13, 0x42, 0x06, 0x84, 0x84,
//     0x53, 0x4a, 0xb3, 0x0c, 0x1b, 0x4c, 0x8d, 0x1c, 0x51, 0x97, 0x19, 0xab, 0xae, 0x77, 0xff, 0xdb, 0xec, 0xf0, 0x10,
//     0x95, 0x34, 0x33, 0x6b, 0xcb, 0x3e, 0x84, 0x0f, 0xb9, 0xd8, 0x5f, 0xb8, 0xa0, 0xb8, 0x55, 0x53, 0x3e, 0x70, 0xf7,
//     0x18, 0xf5, 0xce, 0x7b, 0x4e, 0xbf, 0x27, 0xce, 0xce, 0xa8, 0xb3, 0xbe, 0x40, 0xc5, 0xc5, 0x32, 0x29, 0x3e, 0x71,
//     0x64, 0x9e, 0xde, 0x8c, 0xf6, 0x75, 0xa1, 0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8, 0x78, 0xde, 0x50, 0x40, 0xf7, 0x62,
//     0xde, 0x36, 0xb2, 0xba};

static esp_err_t example_get_sec2_salt(const char **salt, uint16_t *salt_len) {
    ESP_LOGI(TAG, "Development mode: using hard coded salt");
    *salt = sec2_salt;
    *salt_len = sizeof(sec2_salt);
    return ESP_OK;
}

static esp_err_t example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len) {
    ESP_LOGI(TAG, "Development mode: using hard coded verifier");
    *verifier = sec2_verifier;
    *verifier_len = sizeof(sec2_verifier);
    return ESP_OK;
}

/* Event handler for catching system events */
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == NETWORK_PROV_EVENT) {
        switch (event_id) {
        case NETWORK_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;

        case NETWORK_PROV_WIFI_CRED_RECV: {
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG, "Received Wi-Fi credentials for SSID: %.*s", (int)sizeof(wifi_sta_cfg->ssid),
                     (const char *)wifi_sta_cfg->ssid);
            break;
        }

        case NETWORK_PROV_WIFI_CRED_FAIL: {
            network_prov_wifi_sta_fail_reason_t *reason = (network_prov_wifi_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG,
                     "Provisioning failed!\n\tReason : %s"
                     "\n\tPlease reset to factory and retry provisioning",
                     (*reason == NETWORK_PROV_WIFI_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed"
                                                                   : "Wi-Fi access-point not found");

            network_prov_mgr_reset_wifi_sm_state_on_failure();
            break;
        }

        case NETWORK_PROV_WIFI_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning successful");
            break;

        case NETWORK_PROV_END:
            /* De-initialize manager once provisioning is finished */
            esp_err_t err = network_prov_mgr_deinit();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to de-initialize provisioning manager: %s", esp_err_to_name(err));
            }
            break;

        default:
            break;
        }

    } else if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_EVENT);

            ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
            esp_wifi_connect();
            break;

        default:
            break;
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        /* Signal main application to continue execution */
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);

    } else if (event_base == PROTOCOMM_TRANSPORT_BLE_EVENT) {

        switch (event_id) {
        case PROTOCOMM_TRANSPORT_BLE_CONNECTED:
            ESP_LOGI(TAG, "BLE transport: Connected!");
            break;

        case PROTOCOMM_TRANSPORT_BLE_DISCONNECTED:
            ESP_LOGI(TAG, "BLE transport: Disconnected!");
            break;

        default:
            break;
        }

    } else if (event_base == PROTOCOMM_SECURITY_SESSION_EVENT) {

        switch (event_id) {
        case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
            ESP_LOGI(TAG, "Secured session established!");
            break;

        case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
            ESP_LOGE(TAG, "Received invalid security parameters for establishing secure session!");
            break;

        case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
            ESP_LOGE(TAG, "Received incorrect username and/or PoP for establishing secure session!");
            break;

        default:
            break;
        }
    }
}

/* Authenticated BLE endpoint. Input: 32 lowercase hexadecimal nonce bytes.
 * Output: JSON device identity plus proof; no Wi-Fi is required.
 * The phone MUST use ESP-IDF protocomm Security 2, not a raw GATT write.
 */
static esp_err_t device_claim_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen, uint8_t **outbuf,
                                      ssize_t *outlen, void *priv_data) {
    (void)priv_data;
    ESP_LOGI(TAG, "device-claim invoked: session=%lu, input_bytes=%d", (unsigned long)session_id, (int)inlen);
    if (!inbuf || inlen != 32 || !outbuf || !outlen) {
        ESP_LOGE(TAG, "device-claim rejected: expected 32 nonce bytes and valid output pointers");
        return ESP_ERR_INVALID_ARG;
    }
    char nonce[33];
    memcpy(nonce, inbuf, 32);
    nonce[32] = '\0';
    for (int i = 0; i < 32; i++) {
        if (!((nonce[i] >= '0' && nonce[i] <= '9') || (nonce[i] >= 'a' && nonce[i] <= 'f'))) {
            ESP_LOGE(TAG, "device-claim rejected: nonce is not lowercase hexadecimal");
            return ESP_ERR_INVALID_ARG;
        }
    }
    char *response = malloc(320);
    if (!response) {
        ESP_LOGE(TAG, "device-claim: response allocation failed");
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = http_request_make_claim(nonce, response, 320);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "device-claim: identity/proof generation failed: %s", esp_err_to_name(err));
        free(response);
        return err;
    }
    *outlen = strlen(response);
    *outbuf = (uint8_t *)response;
    ESP_LOGI(TAG, "device-claim response ready: %d bytes (contents redacted)", (int)*outlen);
    return ESP_OK;
}

static void get_device_service_name(char *service_name, size_t max) {
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));

    snprintf(service_name, max, "MB-%02X%02X", mac[4], mac[5]);
}

esp_err_t provisioning_init(void) {
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        // ESP_ERROR_CHECK(nvs_flash_init());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Initialize the event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_event_group = xEventGroupCreate();
    assert(wifi_event_group != NULL);

    /* Register our event handler for Wi-Fi, IP and Provisioning related events */
    ESP_ERROR_CHECK(esp_event_handler_register(NETWORK_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_TRANSPORT_BLE_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(
        esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    /* Initialize Wi-Fi including netif with default config */
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Configuration for the provisioning manager */
    network_prov_mgr_config_t config = {.network_prov_wifi_conn_cfg = {.wifi_conn_attempts = MAX_RETRIES},
                                        .scheme = network_prov_scheme_ble,
                                        .scheme_event_handler = NETWORK_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM};

    ESP_LOGI(TAG, "Claim firmware build: %s %s", __DATE__, __TIME__);
    /* Initialize claim identity BEFORE advertising the endpoint. */
    esp_err_t identity_err = http_request_init();
    if (identity_err != ESP_OK) {
        ESP_LOGE(TAG, "Claim identity initialization failed: %s", esp_err_to_name(identity_err));
        return identity_err;
    }

    /* Initialize provisioning manager with the
     * configuration parameters set above */
    ESP_ERROR_CHECK(network_prov_mgr_init(config));

    bool provisioned = false;
    // #ifdef CONFIG_EXAMPLE_RESET_PROVISIONED
    //     network_prov_mgr_reset_wifi_provisioning();
    // #else
    //     /* Let's find out if the device is provisioned */
    ESP_ERROR_CHECK(network_prov_mgr_is_wifi_provisioned(&provisioned));

    // #endif
    /* If device is not yet provisioned start provisioning service */
    if (!provisioned) {
        ESP_LOGI(TAG, "Starting provisioning");

        /* What is the Device Service Name that we want
         * This translates to :
         *     - Wi-Fi SSID when scheme is network_prov_scheme_softap
         *     - device name when scheme is network_prov_scheme_ble
         */
        char service_name[12];
        get_device_service_name(service_name, sizeof(service_name));

        network_prov_security_t security = NETWORK_PROV_SECURITY_2;
        /* The username must be the same one, which has been used in the generation of salt and verifier */

        /* This is the structure for passing security parameters
         * for the protocomm security 2.
         * If dynamically allocated, sec2_params pointer and its content
         * must be valid till NETWORK_PROV_END event is triggered.
         */
        static network_prov_security2_params_t sec2_params = {0};

        ESP_ERROR_CHECK(example_get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
        ESP_ERROR_CHECK(example_get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));

        network_prov_security2_params_t *sec_params = &sec2_params;
        /* What is the service key (could be NULL)
         * This translates to :
         *     - Wi-Fi password when scheme is network_prov_scheme_softap
         *          (Minimum expected length: 8, maximum 64 for WPA2-PSK)
         *     - simply ignored when scheme is network_prov_scheme_ble
         */
        const char *service_key = NULL;

        /* This step is only useful when scheme is network_prov_scheme_ble. This will
         * set a custom 128 bit UUID which will be included in the BLE advertisement
         * and will correspond to the primary GATT service that provides provisioning
         * endpoints as GATT characteristics. Each GATT characteristic will be
         * formed using the primary service UUID as base, with different auto assigned
         * 12th and 13th bytes (assume counting starts from 0th byte). The client side
         * applications must identify the endpoints by reading the User Characteristic
         * Description descriptor (0x2901) for each characteristic, which contains the
         * endpoint name of the characteristic */
        static uint8_t custom_service_uuid[] = {
            /* LSB <---------------------------------------
             * ---------------------------------------> MSB */
            0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf, 0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
        };

        /* If your build fails with linker errors at this point, then you may have
         * forgotten to enable the BT stack or BTDM BLE settings in the SDK (e.g. see
         * the sdkconfig.defaults in the example project) */
        network_prov_scheme_ble_set_service_uuid(custom_service_uuid);

        /* Do not stop and de-init provisioning even after success,
         * so that we can restart it later. */
        // #ifdef CONFIG_EXAMPLE_REPROVISIONING
        //         network_prov_mgr_disable_auto_stop(1000);
        // #endif
        /* Custom BLE endpoint must be created BEFORE provisioning starts. */
        /* Let provisioning close normally when Wi-Fi provisioning finishes. */
        ESP_LOGI(TAG, "Creating BLE endpoint device-claim before service start");
        ESP_ERROR_CHECK(network_prov_mgr_endpoint_create("device-claim"));
        ESP_LOGI(TAG, "BLE endpoint device-claim created; advertising provisioning service as %s", service_name);

        /* Start provisioning service */
        ESP_ERROR_CHECK(
            network_prov_mgr_start_provisioning(security, (const void *)sec_params, service_name, service_key));

        /* Handler must be registered AFTER provisioning starts. */
        ESP_ERROR_CHECK(network_prov_mgr_endpoint_register("device-claim", device_claim_handler, NULL));
        ESP_LOGI(TAG, "BLE endpoint device-claim registered; awaiting secure nonce challenge");

        /* Uncomment the following to wait for the provisioning to finish and then release
         * the resources of the manager. Since in this case de-initialization is triggered
         * by the default event loop handler, we don't need to call the following */
        // network_prov_mgr_wait();
        // network_prov_mgr_deinit();
    } else {
        ESP_LOGW(TAG, "Saved Wi-Fi present: BLE device-claim is NOT advertised on this boot");
        ESP_ERROR_CHECK(network_prov_mgr_deinit());
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    /* Wait for Wi-Fi connection */
    // xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);
    return ESP_OK;
}

bool provisioning_is_connected(void) { return (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_EVENT) != 0; }
