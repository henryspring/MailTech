#pragma once
#include <stddef.h>
#include "esp_err.h"

/* Initialize persistent hardware identity AFTER provisioning_init(). */
esp_err_t http_request_init(void);
/* Respond to authenticated BLE device-claim challenge with signed identity JSON. */
esp_err_t http_request_make_claim(const char *nonce, char *out, size_t out_size);
