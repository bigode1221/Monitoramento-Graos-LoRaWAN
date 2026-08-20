#ifndef DHT22_H
#define DHT22_H

#include <stdint.h>
#include "stm32wlxx_hal.h"

typedef enum
{
  DHT22_STATUS_ERROR = 0,
  DHT22_STATUS_OK = 1,
  DHT22_STATUS_PARTIAL = 2,
  DHT22_STATUS_CHECKSUM_ERROR = 3,
  DHT22_STATUS_RANGE_ERROR = 4
} DHT22_Status_t;

void DHT22_Init(GPIO_TypeDef *port, uint16_t pin);
DHT22_Status_t DHT22_Read(GPIO_TypeDef *port, uint16_t pin, float *temperature_c, float *humidity_percent);

#endif
