#include "dht22.h"
#include "app_config.h"
#include <stdbool.h>

#define DHT22_READ_FAST(port, pin)  (((port)->IDR & (pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define DHT22_LOW_FAST(port, pin)   ((port)->BSRR = ((uint32_t)(pin) << 16U))
#define DHT22_HIGH_FAST(port, pin)  ((port)->BSRR = (pin))

static void DHT22_EnableDWT(void);
static void DHT22_DelayUs(uint32_t microseconds);
static void DHT22_SetPinOutput(GPIO_TypeDef *port, uint16_t pin);
static void DHT22_SetPinInput(GPIO_TypeDef *port, uint16_t pin);
static bool DHT22_WaitForLevel(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level, uint32_t timeout_us);
static bool DHT22_IsLineIdleHigh(GPIO_TypeDef *port, uint16_t pin);

void DHT22_Init(GPIO_TypeDef *port, uint16_t pin)
{
  if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
  else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
  else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();

  DHT22_SetPinInput(port, pin);
  DHT22_EnableDWT();
}

DHT22_Status_t DHT22_Read(GPIO_TypeDef *port, uint16_t pin, float *temperature_c, float *humidity_percent)
{
  uint8_t data[5] = {0};
  uint8_t bits_read = 0U;
  uint32_t primask;
  uint32_t cycles_per_us;
  DHT22_Status_t status = DHT22_STATUS_ERROR;

  if ((temperature_c == NULL) || (humidity_percent == NULL))
  {
    return DHT22_STATUS_ERROR;
  }

  *temperature_c = 0.0f;
  *humidity_percent = 0.0f;

  DHT22_EnableDWT();
  cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;

  if ((cycles_per_us == 0U) || !DHT22_IsLineIdleHigh(port, pin))
  {
    return DHT22_STATUS_ERROR;
  }

  DHT22_SetPinOutput(port, pin);
  DHT22_LOW_FAST(port, pin);
  HAL_Delay(APP_DHT22_START_LOW_MS);

  primask = __get_PRIMASK();
  __disable_irq();

  DHT22_HIGH_FAST(port, pin);
  DHT22_DelayUs(APP_DHT22_RELEASE_US);
  DHT22_SetPinInput(port, pin);

  if (!DHT22_WaitForLevel(port, pin, GPIO_PIN_RESET, APP_DHT22_RESPONSE_TIMEOUT_US) ||
      !DHT22_WaitForLevel(port, pin, GPIO_PIN_SET, APP_DHT22_RESPONSE_TIMEOUT_US) ||
      !DHT22_WaitForLevel(port, pin, GPIO_PIN_RESET, APP_DHT22_RESPONSE_TIMEOUT_US))
  {
    goto read_finished;
  }

  for (uint8_t bit = 0U; bit < 40U; bit++)
  {
    uint32_t high_started_at;
    uint32_t high_duration_us;

    if (!DHT22_WaitForLevel(port, pin, GPIO_PIN_SET, APP_DHT22_BIT_TIMEOUT_US))
    {
      goto read_finished;
    }

    high_started_at = DWT->CYCCNT;

    if (!DHT22_WaitForLevel(port, pin, GPIO_PIN_RESET, APP_DHT22_HIGH_TIMEOUT_US))
    {
      goto read_finished;
    }

    high_duration_us = (DWT->CYCCNT - high_started_at) / cycles_per_us;
    data[bit / 8U] <<= 1U;

    if (high_duration_us > APP_DHT22_ONE_THRESHOLD_US)
    {
      data[bit / 8U] |= 1U;
    }

    bits_read = (uint8_t)(bit + 1U);
  }

  if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4])
  {
    status = DHT22_STATUS_CHECKSUM_ERROR;
    goto read_finished;
  }

  *humidity_percent = (float)(((uint16_t)data[0] << 8U) | data[1]) / 10.0f;

  if ((data[2] & 0x80U) != 0U)
  {
    *temperature_c = -((float)(((uint16_t)(data[2] & 0x7FU) << 8U) | data[3]) / 10.0f);
  }
  else
  {
    *temperature_c = (float)(((uint16_t)data[2] << 8U) | data[3]) / 10.0f;
  }

  if ((*humidity_percent < 0.0f) || (*humidity_percent > 100.0f) ||
      (*temperature_c < -40.0f) || (*temperature_c > 80.0f))
  {
    *temperature_c = 0.0f;
    *humidity_percent = 0.0f;
    status = DHT22_STATUS_RANGE_ERROR;
    goto read_finished;
  }

  status = DHT22_STATUS_OK;

read_finished:
  __set_PRIMASK(primask);

  if ((status == DHT22_STATUS_ERROR) && (bits_read >= 16U))
  {
    uint16_t partial_humidity = ((uint16_t)data[0] << 8U) | data[1];

    if (partial_humidity <= 1000U)
    {
      *humidity_percent = (float)partial_humidity / 10.0f;
      status = DHT22_STATUS_PARTIAL;
    }
  }

  return status;
}

static void DHT22_EnableDWT(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DHT22_DelayUs(uint32_t microseconds)
{
  uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  uint32_t started_at = DWT->CYCCNT;
  uint32_t wait_cycles = microseconds * cycles_per_us;

  while ((DWT->CYCCNT - started_at) < wait_cycles)
  {
  }
}

static void DHT22_SetPinOutput(GPIO_TypeDef *port, uint16_t pin)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = pin;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(port, &gpio);
}

static void DHT22_SetPinInput(GPIO_TypeDef *port, uint16_t pin)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &gpio);
}

static bool DHT22_WaitForLevel(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level, uint32_t timeout_us)
{
  uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  uint32_t started_at = DWT->CYCCNT;
  uint32_t timeout_cycles = timeout_us * cycles_per_us;

  while (DHT22_READ_FAST(port, pin) != level)
  {
    if ((DWT->CYCCNT - started_at) > timeout_cycles)
    {
      return false;
    }
  }

  return true;
}

static bool DHT22_IsLineIdleHigh(GPIO_TypeDef *port, uint16_t pin)
{
  DHT22_SetPinInput(port, pin);
  HAL_Delay(10U);
  return (DHT22_READ_FAST(port, pin) == GPIO_PIN_SET);
}
