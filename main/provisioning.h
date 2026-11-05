#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t provisioning_init(void);
bool provisioning_is_connected(void);