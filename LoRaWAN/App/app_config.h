#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"

#define APP_PROJECT_TITLE                    "Monitoramento Inteligente da Qualidade de Gr\xE3os"

#define APP_LORA_AU915_FSB                  2U
#define APP_LORA_JOIN_MAX_FAILS             5U
#define APP_LORA_JOIN_RETRY_SECONDS          (30U * 60U)
#define APP_TELEMETRY_PERIOD_SECONDS        (30U * 60U)

#if ((APP_LORA_AU915_FSB < 1U) || (APP_LORA_AU915_FSB > 8U))
#error "APP_LORA_AU915_FSB must be between 1 and 8"
#endif

#define APP_FIELD_DEBUG_LOG                 1U
#define APP_PRINT_LORAWAN_KEYS              0U

/* Add a new pin/channel pair here and one entry in app_dht22_sensors[] to add another DHT22. */
#define APP_DHT22_1_PORT                    GPIOA
#define APP_DHT22_1_PIN                     GPIO_PIN_0
#define APP_DHT22_1_TEMPERATURE_CHANNEL     1U
#define APP_DHT22_1_HUMIDITY_CHANNEL        2U
#define APP_DHT22_2_PORT                    GPIOA
#define APP_DHT22_2_PIN                     GPIO_PIN_1
#define APP_DHT22_2_TEMPERATURE_CHANNEL     3U
#define APP_DHT22_2_HUMIDITY_CHANNEL        4U
#define APP_BATTERY_CHANNEL                 5U
/* Cayenne LPP channels 6 and 7 are reserved for the next temperature/humidity sensor. */
#define APP_DHT22_BOOT_SETTLE_MS             300U
#define APP_DHT22_START_LOW_MS              20U
#define APP_DHT22_RELEASE_US                40U
#define APP_DHT22_RESPONSE_TIMEOUT_US       500U
#define APP_DHT22_BIT_TIMEOUT_US            300U
#define APP_DHT22_HIGH_TIMEOUT_US           2000U
#define APP_DHT22_ONE_THRESHOLD_US          40U


#define APP_E77_LED1_PORT                   GPIOB
#define APP_E77_LED1_PIN                    GPIO_PIN_4
#define APP_E77_LED2_PORT                   GPIOB
#define APP_E77_LED2_PIN                    GPIO_PIN_3

#endif
