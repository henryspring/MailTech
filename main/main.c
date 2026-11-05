#include "esp_log.h"
#include "provisioning.h"
#include "cloud.h"

void app_main(void)
{
    ESP_ERROR_CHECK(provisioning_init());
    ESP_ERROR_CHECK(http_request_init());
    ESP_LOGI("main", "LetterPeek BLE claim endpoint ready during first-time provisioning");
}
