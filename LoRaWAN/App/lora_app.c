/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "app_version.h"
#include "Commissioning.h"
#include "subghz_phy_version.h"
#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"
#include "smtc_modem_hal.h"
#include "smtc_modem_relay_api.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"
#include "rng.h"
#include "lorawan_api.h"
#include "stm32_lpm.h"

/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "dht22.h"
#include "smtc_real.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t temperature_channel;
  uint8_t humidity_channel;
} AppDht22Sensor_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * LEDs period value of the timer in ms
  */
#define LED_PERIOD_TIME 500

/**
  * Join switch period value of the timer in ms
  */
#define JOIN_TIME 2000

/**
  * Uplink time interval in Certification mode is 10s (recommended value)
  */
#define CERT_TX_DUTYCYCLE   10

/**
  * Stack id value (multistacks modem is not yet available)
  */
#define STACK_ID 0

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 256kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS        (0x0803F000UL)

#define SECURE_ELEMENT_CONTEXT_SIZE     0x2A0UL
#define MODEM_CONTEXT_SIZE              0x10UL
#define LORAWAN_CONTEXT_SIZE            0x28UL

#define ADDR_FLASH_LORAWAN_CONTEXT              LORAWAN_NVM_BASE_ADDRESS
#define ADDR_FLASH_MODEM_CONTEXT                (void *)(LORAWAN_NVM_BASE_ADDRESS + LORAWAN_CONTEXT_SIZE)
#define ADDR_FLASH_SECURE_ELEMENT_CONTEXT       (void *)(LORAWAN_NVM_BASE_ADDRESS + LORAWAN_CONTEXT_SIZE + MODEM_CONTEXT_SIZE)

/* USER CODE BEGIN PD */

#if APP_FIELD_DEBUG_LOG
#define FIELD_LOG(...) APP_LOG(__VA_ARGS__)
#else
#define FIELD_LOG(...)
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/*!
 * @brief Stringify constants
 */
#define xstr( a ) str( a )
#define str( a ) #a

/*!
 * @brief Helper macro that returned a human-friendly message if a command does not return SMTC_MODEM_RC_OK
 *
 * @remark The macro is implemented to be used with functions returning a @ref smtc_modem_return_code_t
 *
 * @param[in] rc  Return code
 */

#define ASSERT_SMTC_MODEM_RC( rc_func )                                                     \
  do                                                                                        \
  {                                                                                         \
    smtc_modem_return_code_t rc = rc_func;                                                  \
    if( rc == SMTC_MODEM_RC_NOT_INIT )                                                      \
    {                                                                                       \
      FIELD_LOG(TS_OFF, VLEVEL_H,  "In %s - %s (line %d): %s\r\n", __FILE__, __func__, __LINE__,   \
              xstr( SMTC_MODEM_RC_NOT_INIT ) );                             \
    }                                                                                       \
    else if( rc == SMTC_MODEM_RC_INVALID )                                                  \
    {                                                                                       \
      FIELD_LOG(TS_OFF, VLEVEL_H,  "In %s - %s (line %d): %s\r\n", __FILE__, __func__, __LINE__,   \
              xstr( SMTC_MODEM_RC_INVALID ) );                              \
    }                                                                                       \
    else if( rc == SMTC_MODEM_RC_BUSY )                                                     \
    {                                                                                       \
      FIELD_LOG(TS_OFF, VLEVEL_H,  "In %s - %s (line %d): %s\r\n", __FILE__, __func__, __LINE__,   \
              xstr( SMTC_MODEM_RC_BUSY ) );                                 \
    }                                                                                       \
    else if( rc == SMTC_MODEM_RC_FAIL )                                                     \
    {                                                                                       \
      FIELD_LOG(TS_OFF, VLEVEL_H,  "In %s - %s (line %d): %s\r\n", __FILE__, __func__, __LINE__,   \
              xstr( SMTC_MODEM_RC_FAIL ) );                                 \
    }                                                                                       \
    else if( rc == SMTC_MODEM_RC_NO_TIME )                                                  \
    {                                                                                       \
      FIELD_LOG(TS_OFF, VLEVEL_L,  "In %s - %s (line %d): %s\r\n", __FILE__, __func__, __LINE__, \
              xstr( SMTC_MODEM_RC_NO_TIME ) );                            \
    }                                                                                       \
    else if( rc == SMTC_MODEM_RC_INVALID_STACK_ID )                                         \
    {                                                                                       \
      FIELD_LOG(TS_OFF, VLEVEL_H,  "In %s - %s (line %d): %s\r\n", __FILE__, __func__, __LINE__,   \
              xstr( SMTC_MODEM_RC_INVALID_STACK_ID ) );                     \
    }                                                                                       \
    else if( rc == SMTC_MODEM_RC_NO_EVENT )                                                 \
    {                                                                                       \
      FIELD_LOG(TS_OFF, VLEVEL_M,  "In %s - %s (line %d): %s\r\n", __FILE__, __func__, __LINE__,    \
              xstr( SMTC_MODEM_RC_NO_EVENT ) );                              \
    }                                                                                       \
  } while( 0 )

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(uint8_t port);

/**
  * @brief Read every configured DHT22 and optionally add values to the payload or Termite log.
  */
static void ReadDht22Sensors(bool add_to_payload, bool log_to_termite);

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
/**
  * @brief  Sleep timer callback function
  * @param  context ptr
  */
static void OnSleepTimerEvent(void *context);
#endif /* (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0) */

/**
  * @brief User callback for event
  *
  *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
  *  Several events may have to be read from the modem when this callback is called.
  */
static void EventCallback(void);

/*!
 * Restore the NVM Data context from the Flash
 *
* @param [in]  ctx_type   Type of modem context that need to be restored
* @param [in]  offset     Memory offset after ctx_type address
* @param [out] buffer     Buffer pointer to write to
* @param [in]  size       Buffer size to read in bytes
*/
static void RestoreContext(const modem_context_type_t ctx_type, uint32_t offset, uint8_t *buffer, const uint32_t size);
/*!
 * Store the NVM Data context to the Flash
 *
* @param [in] ctx_type   Type of modem context that need to be saved
* @param [in] offset     Memory offset after ctx_type address
* @param [in] buffer     Buffer pointer to write from
* @param [in] size       Buffer size to write in bytes
*/
static void StoreContext(const modem_context_type_t ctx_type, uint32_t offset, const uint8_t *buffer,
                         const uint32_t size);
/*!
 * Get Random value using the RNG module
 *
 * \retval value  Return the random value
 */
static uint32_t GetRandomValue(void);

/* USER CODE BEGIN PFP */
static void SystemReset(void);
static void E77_LED_Init(void);
static void E77_LED_AllOff(void);
static void E77_LED_BootOk(void);

/**
  * @brief Restrict AU915 joins and uplinks to the configured 8-channel FSB.
  */
static bool ConfigureAu915Fsb(void);

/**
  * @brief  LED Tx timer callback function
  * @param  context ptr of LED context
  */
static void OnTxTimerLedEvent(void *context);

/**
  * @brief  LED Rx timer callback function
  * @param  context ptr of LED context
  */
static void OnRxTimerLedEvent(void *context);

/**
  * @brief  LED Join timer callback function
  * @param  context ptr of LED context
  */
static void OnJoinTimerLedEvent(void *context);

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/

/**
  * @brief LoRaWAN User credentials
  */
static uint8_t user_dev_eui[8]      = FORMAT32_KEY(LORAWAN_DEVICE_EUI);
static uint8_t user_join_eui[8]     = FORMAT32_KEY(LORAWAN_JOIN_EUI);
static uint8_t user_gen_app_key[16] = FORMAT_KEY(LORAWAN_GEN_APP_KEY);
static uint8_t user_app_key[16]     = FORMAT_KEY(LORAWAN_APP_KEY);
/**
  * @brief  Buffer for rx payload
  */
static uint8_t                  rx_payload[SMTC_MODEM_MAX_LORAWAN_PAYLOAD_LENGTH] = { 0 };

/**
  * @brief  Size of the payload in the rx_payload buffer
  */
static uint8_t                  rx_payload_size = 0;

/**
  * @brief  Metadata of downlink
  */
static smtc_modem_dl_metadata_t rx_metadata     = { 0 };

/**
  * @brief  Remaining downlink payload in modem
  */
static uint8_t                  rx_remaining    = 0;

/**
  * @brief LoRaWAN Certification Mode
  */
static bool CertMode = LORAWAN_CERTIFICATION_MODE;

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
/**
  * @brief Timer to handle the sleep time
  */
static UTIL_TIMER_Object_t SleepTimer;
#endif /* (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0) */

/**
  * Temp buffer to store a FLASH page in RAM when partial replacement is needed
  */
static uint8_t FLASH_RAM_buffer[FLASH_IF_BUFFER_SIZE];

/**
  * @brief Handler Callbacks
  */
static Callbacks_t Callbacks =
{
  .EventCallback =                EventCallback,
  .RestoreContext =               RestoreContext,
  .StoreContext =                 StoreContext,
  .GetRandomValue =               GetRandomValue,
  .GetBatteryLevel =              GetBatteryLevel,
  .GetTemperatureLevel =          GetTemperatureLevel,
  .SystemReset =                  SystemReset,
};

/* USER CODE BEGIN PV */

static const AppDht22Sensor_t app_dht22_sensors[] =
{
  {APP_DHT22_1_PORT, APP_DHT22_1_PIN, APP_DHT22_1_TEMPERATURE_CHANNEL, APP_DHT22_1_HUMIDITY_CHANNEL},
  {APP_DHT22_2_PORT, APP_DHT22_2_PIN, APP_DHT22_2_TEMPERATURE_CHANNEL, APP_DHT22_2_HUMIDITY_CHANNEL},
};

#define APP_DHT22_SENSOR_COUNT ((uint8_t)(sizeof(app_dht22_sensors) / sizeof(app_dht22_sensors[0])))

#ifdef CAYENNE_LPP
/**
  * @brief Specifies the state of the application LED
  */
static uint8_t AppLedStateOn = RESET;
#endif /*CAYENNE_LPP*/

/**
  * @brief Timer to handle the application Tx Led to toggle
  */
static UTIL_TIMER_Object_t TxLedTimer;

/**
  * @brief Timer to handle the application Rx Led to toggle
  */
static UTIL_TIMER_Object_t RxLedTimer;

/**
  * @brief Timer to handle the application Join Led to toggle
  */
static UTIL_TIMER_Object_t JoinLedTimer;

static volatile bool lora_join_request = false;

static bool lora_engine_enabled = true;
static uint8_t lora_join_fail_count = 0;
/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void LoRaWAN_Init(void)
{
  FIELD_LOG(TS_OFF, VLEVEL_M, "\r\n================================================\r\n");
  FIELD_LOG(TS_OFF, VLEVEL_M, "%s\r\n", APP_PROJECT_TITLE);
  FIELD_LOG(TS_OFF, VLEVEL_M, "================================================\r\n");
  FIELD_LOG(TS_OFF, VLEVEL_M, "[BOOT] Sistema iniciado\r\n");
  FIELD_LOG(TS_OFF, VLEVEL_M, "[SERIAL] Termite: ST-LINK VCP / USART2 / 115200 8N1\r\n");
  E77_LED_Init();
  E77_LED_BootOk();
  /* USER CODE BEGIN LoRaWAN_Init_LV */
  lr1mac_version_t lorawan_version;
  lr1mac_version_t rp_version;

  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */
  for (uint8_t sensor_index = 0U; sensor_index < APP_DHT22_SENSOR_COUNT; sensor_index++)
  {
    DHT22_Init(app_dht22_sensors[sensor_index].port, app_dht22_sensors[sensor_index].pin);
  }

  FIELD_LOG(TS_OFF, VLEVEL_M, "[SENSORES] DHT22 + Cayenne LPP\r\n");
  FIELD_LOG(TS_OFF, VLEVEL_M, "[SENSORES] Quantidade configurada: %u\r\n",
            (unsigned int)APP_DHT22_SENSOR_COUNT);
  FIELD_LOG(TS_OFF, VLEVEL_M,
            "[LORAWAN] Limite de falhas de join: %u; nova tentativa em %u s\r\n",
            APP_LORA_JOIN_MAX_FAILS, (unsigned int)APP_LORA_JOIN_RETRY_SECONDS);

  HAL_Delay(APP_DHT22_BOOT_SETTLE_MS);
  FIELD_LOG(TS_OFF, VLEVEL_M, "[SENSORES] Leitura inicial:\r\n");
  ReadDht22Sensors(false, true);

  lora_join_request = false;
  lora_engine_enabled = true;
  lora_join_fail_count = 0;

  E77_LED_AllOff();

  FIELD_LOG(TS_OFF, VLEVEL_M, "[FIRMWARE] LoRaWAN End Node LBM\r\n");  /* Get LoRaWAN APP version*/
  FIELD_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(APP_VERSION_MAIN),
          (uint8_t)(APP_VERSION_SUB1),
          (uint8_t)(APP_VERSION_SUB2));

  /* Get MW LoRaWAN info */
  FIELD_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION:  V%X.%X.%X\r\n",
          (uint8_t)(LORAWAN_VERSION_MAIN),
          (uint8_t)(LORAWAN_VERSION_SUB1),
          (uint8_t)(LORAWAN_VERSION_SUB2));

  /* Get MW SubGhz_Phy info */
  FIELD_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
          (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* Get LoRaWAN Link Layer info */
  memset(&lorawan_version, 0, sizeof(lr1mac_version_t));
  lorawan_version = lorawan_api_get_spec_version(STACK_ID);

  FIELD_LOG(TS_OFF, VLEVEL_M, "L2_SPEC_VERSION:     V%X.%X.%X\r\n",
          (uint8_t)(lorawan_version.major),
          (uint8_t)(lorawan_version.minor),
          (uint8_t)(lorawan_version.patch));

  /* Get LoRaWAN Regional Parameters info */
  memset(&rp_version, 0, sizeof(lr1mac_version_t));
  rp_version = lorawan_api_get_regional_parameters_version(STACK_ID);
  FIELD_LOG(TS_OFF, VLEVEL_M, "RP_SPEC_VERSION:     V%X-%X.%X.%X\r\n",
          (uint8_t)(rp_version.revision),
          (uint8_t)(rp_version.major),
          (uint8_t)(rp_version.minor),
          (uint8_t)(rp_version.patch));

  UTIL_TIMER_Create(&TxLedTimer, LED_PERIOD_TIME, UTIL_TIMER_ONESHOT, OnTxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&RxLedTimer, LED_PERIOD_TIME, UTIL_TIMER_ONESHOT, OnRxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&JoinLedTimer, LED_PERIOD_TIME, UTIL_TIMER_PERIODIC, OnJoinTimerLedEvent, NULL);

  /* USER CODE END LoRaWAN_Init_1 */

  if (FLASH_IF_Init(FLASH_RAM_buffer) != FLASH_IF_OK)
  {
    Error_Handler();
  }

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
  UTIL_TIMER_Create(&SleepTimer, LED_PERIOD_TIME, UTIL_TIMER_ONESHOT, OnSleepTimerEvent, NULL);
#endif /* (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0) */
  /* Init the Lora Stack*/
  /* Init the modem and use EventCallback as event callback, please note that the callback will be */
  /* called immediately after the first call to smtc_modem_run_engine because of the reset detection */
  smtc_modem_init(&Callbacks);

  /* Certification mode is disabled by default. It can be enabled setting LORAWAN_CERTIFICATION_MODE to true */
  smtc_modem_set_certification_mode(STACK_ID, CertMode);

  /* BSP crystal accurrancy could be set to a different value. By default it is 10. */
  smtc_modem_set_crystal_error_ppm(BSP_CRYSTAL_ERROR);

  /* USER CODE BEGIN LoRaWAN_Init_Last */
  // UTIL_TIMER_Start(&JoinLedTimer);

  /* USER CODE END LoRaWAN_Init_Last */
}

void LoRaWAN_Process(void)
{
  uint32_t sleep_time_ms = 0;
  smtc_modem_return_code_t join_status;

  if (lora_join_request == true)
  {
    lora_join_request = false;
    lora_join_fail_count = 0U;
    join_status = smtc_modem_join_network(STACK_ID);

    if ((join_status == SMTC_MODEM_RC_OK) || (join_status == SMTC_MODEM_RC_BUSY))
    {
      lora_engine_enabled = true;
      FIELD_LOG(TS_ON, VLEVEL_M, "[LORAWAN] Nova tentativa de conexao solicitada (rc=%d)\r\n", join_status);
    }
    else
    {
      lora_engine_enabled = false;
      FIELD_LOG(TS_ON, VLEVEL_H, "[LORAWAN] Falha ao solicitar nova conexao (rc=%d)\r\n", join_status);
    }
  }

  /*
   * Motor LoRaWAN.
   * Depois de muitas falhas, o modem pausa ate o timer solicitar uma nova tentativa.
   */
  if (lora_engine_enabled == true)
  {
    sleep_time_ms = smtc_modem_run_engine();
  }

  /* The event callback can disable the engine during smtc_modem_run_engine(). */
  if (lora_engine_enabled == false)
  {
    sleep_time_ms = (APP_LORA_JOIN_RETRY_SECONDS * 1000U);
  }

  /*
   * Low power.
   */
  if ((lora_engine_enabled == false) || (smtc_modem_is_irq_flag_pending() == false))
  {
    if (sleep_time_ms > 0)
    {
#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
      UTIL_TIMER_SetPeriod(&SleepTimer, sleep_time_ms);
      UTIL_TIMER_Start(&SleepTimer);
      UTIL_LPM_EnterLowPower();
#endif
    }
  }
}

static void SystemReset(void)
{
  /* USER CODE BEGIN SystemReset_1 */

  /* USER CODE END SystemReset_1 */
  __disable_irq();
  HAL_NVIC_SystemReset();   /* Restart system */
  /* USER CODE BEGIN SystemReset_Last */

  /* USER CODE END SystemReset_Last */
}

static uint32_t GetRandomValue(void)
{
  uint32_t rand_nb = 0;
  // Init and enable RNG
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;

  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }

  // Wait for data ready interrupt: 42+4 RNG clock cycles
  if (HAL_RNG_GenerateRandomNumber(&hrng, &rand_nb) != HAL_OK)
  {
    Error_Handler();
  }

  // Disable RNG
  HAL_RNG_DeInit(&hrng);

  return rand_nb;
}

static void RestoreContext(const modem_context_type_t ctx_type, uint32_t offset, uint8_t *buffer,
                           const uint32_t size)
{
  /* Offset is only used for fuota and store and forward purpose and for multistack features. To avoid ram consumption */
  /* the use of hal_flash_read_modify_write is only done in these cases */
  /* USER CODE BEGIN RestoreContext_1 */

  /* USER CODE END RestoreContext_1 */
  FLASH_IF_StatusTypedef ret_status = FLASH_IF_OK;
  uint32_t context_address = 0U;
  uint32_t context_capacity = 0U;

  switch (ctx_type)
  {
    case CONTEXT_MODEM:
      context_address = (uint32_t)ADDR_FLASH_MODEM_CONTEXT;
      context_capacity = MODEM_CONTEXT_SIZE;
      break;
    case CONTEXT_LORAWAN_STACK:
      context_address = (uint32_t)ADDR_FLASH_LORAWAN_CONTEXT;
      context_capacity = LORAWAN_CONTEXT_SIZE;
      break;
    case CONTEXT_SECURE_ELEMENT:
      context_address = (uint32_t)ADDR_FLASH_SECURE_ELEMENT_CONTEXT;
      context_capacity = SECURE_ELEMENT_CONTEXT_SIZE;
      break;
    default:
      ret_status = FLASH_IF_PARAM_ERROR;
      break;
  }

  if (ret_status == FLASH_IF_OK)
  {
    if ((buffer == NULL) || (size == 0U) || (offset > context_capacity) || (size > (context_capacity - offset)))
    {
      ret_status = FLASH_IF_PARAM_ERROR;
    }
    else
    {
      ret_status = FLASH_IF_Read(buffer, (const void *)(context_address + offset), size);
    }
  }

  if (ret_status != 0)
  {
    FIELD_LOG(TS_OFF, VLEVEL_M, "restore ctx type %d, FLASH_IF return: %d\r\n", ctx_type, ret_status);
  }
  /* USER CODE BEGIN RestoreContext_Last */

  /* USER CODE END RestoreContext_Last */
}

static void StoreContext(const modem_context_type_t ctx_type, uint32_t offset, const uint8_t *buffer,
                         const uint32_t size)
{
  /* USER CODE BEGIN StoreContext_1 */

  /* USER CODE END StoreContext_1 */
  FLASH_IF_StatusTypedef ret_status = FLASH_IF_OK;
  uint32_t context_address = 0U;
  uint32_t context_capacity = 0U;

  /* Offset is only used for fuota and store and forward purpose and for multistack features. To avoid ram consumption
   * the use of hal_flash_read_modify_write is only done in these cases */
  switch (ctx_type)
  {
    case CONTEXT_MODEM:
      context_address = (uint32_t)ADDR_FLASH_MODEM_CONTEXT;
      context_capacity = MODEM_CONTEXT_SIZE;
      break;
    case CONTEXT_LORAWAN_STACK:
      context_address = (uint32_t)ADDR_FLASH_LORAWAN_CONTEXT;
      context_capacity = LORAWAN_CONTEXT_SIZE;
      break;
    case CONTEXT_SECURE_ELEMENT:
      context_address = (uint32_t)ADDR_FLASH_SECURE_ELEMENT_CONTEXT;
      context_capacity = SECURE_ELEMENT_CONTEXT_SIZE;
      break;
    default:
      ret_status = FLASH_IF_PARAM_ERROR;
      break;
  }

  if (ret_status == FLASH_IF_OK)
  {
    if ((buffer == NULL) || (size == 0U) || (offset > context_capacity) || (size > (context_capacity - offset)))
    {
      ret_status = FLASH_IF_PARAM_ERROR;
    }
    else
    {
      ret_status = FLASH_IF_Write((void *)(context_address + offset), buffer, size);
    }
  }

  if (ret_status != 0)
  {
    FIELD_LOG(TS_OFF, VLEVEL_M, "store ctx type %d, FLASH_IF return: %d\r\n", ctx_type, ret_status);
  }
  /* USER CODE BEGIN StoreContext_Last */

  /* USER CODE END StoreContext_Last */
}

static void EventCallback(void)
{
  smtc_modem_event_t current_event = {0};
  uint8_t            event_pending_count = 0U;
  uint8_t            stack_id = STACK_ID;
  smtc_modem_status_mask_t status_mask = 0;
  smtc_modem_return_code_t event_status;

  /* Continue to read modem event until all event has been processed */
  do
  {
    /* Read modem event */
    event_status = smtc_modem_get_event(&current_event, &event_pending_count);
    if (event_status != SMTC_MODEM_RC_OK)
    {
      ASSERT_SMTC_MODEM_RC(event_status);
      break;
    }

    switch (current_event.event_type)
    {
      case SMTC_MODEM_EVENT_RESET:
        FIELD_LOG(TS_OFF, VLEVEL_M, "[LORAWAN] Evento: RESET\r\n");

        // GetUniqueId(user_dev_eui);

        /* Set user credentials */
        ASSERT_SMTC_MODEM_RC(smtc_modem_set_deveui(stack_id, user_dev_eui));
        ASSERT_SMTC_MODEM_RC(smtc_modem_set_joineui(stack_id, user_join_eui));
        ASSERT_SMTC_MODEM_RC(smtc_modem_set_appkey(stack_id, user_gen_app_key));
        ASSERT_SMTC_MODEM_RC(smtc_modem_set_nwkkey(stack_id, user_app_key));

        /* Set user region */
        ASSERT_SMTC_MODEM_RC(smtc_modem_set_region(stack_id, ACTIVE_REGION));

#if defined(REGION_AU915)
        if (ConfigureAu915Fsb() == false)
        {
          FIELD_LOG(TS_ON, VLEVEL_H, "[LORAWAN] Falha ao configurar AU915 FSB%u\r\n",
                    (unsigned int)APP_LORA_AU915_FSB);
          Error_Handler();
        }

        FIELD_LOG(TS_OFF, VLEVEL_M, "[LORAWAN] AU915 FSB%u ativo (8 canais + canal 500 kHz)\r\n",
                  (unsigned int)APP_LORA_AU915_FSB);
#endif

#if APP_PRINT_LORAWAN_KEYS
        SecureElementPrintKeys(stack_id);
#endif
        CertMode = (smtc_modem_is_certification_port_disabled(STACK_ID)) ? 0 : CertMode;
        if (CertMode == false)
        {
          /* Schedule a Join LoRaWAN network */
          ASSERT_SMTC_MODEM_RC(smtc_modem_join_network(stack_id));
        }
        break;

      case SMTC_MODEM_EVENT_ALARM:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "[LORAWAN] Evento: ALARME\r\n");
        if (CertMode == true)
        {
          ASSERT_SMTC_MODEM_RC(smtc_modem_alarm_clear_timer());
        }
        else
        {
          /* Send periodical uplink */
          SendTxData(LORAWAN_USER_APP_PORT);
        }
        break;

      case SMTC_MODEM_EVENT_JOINED:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "[LORAWAN] Conectado a rede\r\n");
        /* USER CODE BEGIN EventCallback_1 */
        lora_join_fail_count = 0;
        lora_join_request = false;
        lora_engine_enabled = true;

        if (JoinLedTimer.IsRunning)
        {
          UTIL_TIMER_Stop(&JoinLedTimer);
          HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
        }
        /* USER CODE END EventCallback_1 */
        if (CertMode == false)
        {
          /* Send first periodical uplink */
          SendTxData(LORAWAN_USER_APP_PORT);
        }
        break;

      case SMTC_MODEM_EVENT_TXDONE:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "[LORAWAN] Transmissao concluida\r\n");
        smtc_modem_get_status(STACK_ID, &status_mask);
        /* USER CODE BEGIN EventCallback_2 */
        /* Check if the device has already joined a network */
        if ((JoinLedTimer.IsRunning) && (status_mask & SMTC_MODEM_STATUS_JOINED) == SMTC_MODEM_STATUS_JOINED)
        {
          UTIL_TIMER_Stop(&JoinLedTimer);
          HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET); /* LED_RED */
        }
        /* USER CODE END EventCallback_2 */
        break;

      case SMTC_MODEM_EVENT_DOWNDATA:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "[LORAWAN] Dados recebidos\r\n");
        /* USER CODE BEGIN EventCallback_3 */
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); /* LED_BLUE */
        UTIL_TIMER_Start(&RxLedTimer);
        /* USER CODE END EventCallback_3 */
        /* Get downlink data */
        ASSERT_SMTC_MODEM_RC(smtc_modem_get_downlink_data(rx_payload, &rx_payload_size, &rx_metadata, &rx_remaining));
        FIELD_LOG(TS_OFF, VLEVEL_M, "[LORAWAN] Porta do downlink: %u\r\n",
                  (unsigned int)rx_metadata.fport);
        /* FIELD_LOG(TS_OFF, VLEVEL_M, "Received payload", rx_payload, rx_payload_size ); */
        break;

      case SMTC_MODEM_EVENT_JOINFAIL:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "[LORAWAN] Falha ao conectar na rede\r\n");
        smtc_modem_get_status(STACK_ID, &status_mask);
        /* USER CODE BEGIN EventCallback_4 */
        lora_join_fail_count++;

        FIELD_LOG(TS_ON, VLEVEL_M, "[LORAWAN] Falhas consecutivas: %u\r\n",
                  (unsigned int)lora_join_fail_count);

        if (lora_join_fail_count >= APP_LORA_JOIN_MAX_FAILS)
        {
          lora_engine_enabled = false;

          if (JoinLedTimer.IsRunning)
          {
            UTIL_TIMER_Stop(&JoinLedTimer);
          }

          FIELD_LOG(TS_ON, VLEVEL_M, "[BAIXO CONSUMO] LoRaWAN pausado; nova tentativa em %u s\r\n",
                    (unsigned int)APP_LORA_JOIN_RETRY_SECONDS);
          FIELD_LOG(TS_ON, VLEVEL_M, "[BAIXO CONSUMO] DHT22 em espera; nenhuma leitura durante a pausa\r\n");
        }
        /* Check if the device has already joined a network */
        // if ((!JoinLedTimer.IsRunning) && (status_mask & SMTC_MODEM_STATUS_JOINED) != SMTC_MODEM_STATUS_JOINED)
        // {
        //   UTIL_TIMER_Start(&JoinLedTimer);
        // }
        /* USER CODE END EventCallback_4 */
        break;

      case SMTC_MODEM_EVENT_ALCSYNC_TIME:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: ALCSync service TIME\r\n");
        break;

      case SMTC_MODEM_EVENT_LINK_CHECK:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: LINK_CHECK\r\n");
        break;

      case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: CLASS_B_PING_SLOT_INFO\r\n");
        break;

      case SMTC_MODEM_EVENT_CLASS_B_STATUS:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: CLASS_B_STATUS\r\n");
        break;

      case SMTC_MODEM_EVENT_LORAWAN_MAC_TIME:
        FIELD_LOG(TS_OFF, VLEVEL_L,  "Event received: LORAWAN MAC TIME\r\n");
        break;

      case SMTC_MODEM_EVENT_LORAWAN_FUOTA_DONE:
      {
        bool status = current_event.event_data.fuota_status.successful;
        if (status == true)
        {
          FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: FUOTA SUCCESSFUL\r\n");
        }
        else
        {
          FIELD_LOG(TS_OFF, VLEVEL_L,  "Event received: FUOTA FAIL\r\n");
        }
        break;
      }

      case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_C:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: MULTICAST CLASS_C STOP\r\n");
        break;

      case SMTC_MODEM_EVENT_NO_MORE_MULTICAST_SESSION_CLASS_B:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: MULTICAST CLASS_B STOP\r\n");
        break;

      case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_C:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: New MULTICAST CLASS_C \r\n");
        break;

      case SMTC_MODEM_EVENT_NEW_MULTICAST_SESSION_CLASS_B:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: New MULTICAST CLASS_B\r\n");
        break;

      case SMTC_MODEM_EVENT_FIRMWARE_MANAGEMENT:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: FIRMWARE_MANAGEMENT\r\n");
        if (current_event.event_data.fmp.status == SMTC_MODEM_EVENT_FMP_REBOOT_IMMEDIATELY)
        {
          HAL_NVIC_SystemReset();
        }
        break;

      case SMTC_MODEM_EVENT_STREAM_DONE:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: STREAM_DONE\r\n");
        break;

      case SMTC_MODEM_EVENT_UPLOAD_DONE:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: UPLOAD_DONE\r\n");
        break;

      case SMTC_MODEM_EVENT_DM_SET_CONF:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: DM_SET_CONF\r\n");
        break;

      case SMTC_MODEM_EVENT_MUTE:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: MUTE\r\n");
        break;
      case SMTC_MODEM_EVENT_REGIONAL_DUTY_CYCLE:
      {
        uint8_t duty_cycle_status = current_event.event_data.regional_duty_cycle.status;
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Event received: DUTY_CYCLE busy %d\r\n", duty_cycle_status);
      }
      break;
      default:
        FIELD_LOG(TS_OFF, VLEVEL_M,  "Unknown event %u\r\n", current_event.event_type);
        break;
    }
  } while (event_pending_count > 0);
}

static void ReadDht22Sensors(bool add_to_payload, bool log_to_termite)
{
  for (uint8_t sensor_index = 0U; sensor_index < APP_DHT22_SENSOR_COUNT; sensor_index++)
  {
    float temperature = 0.0f;
    float humidity = 0.0f;
    DHT22_Status_t sensor_status = DHT22_Read(app_dht22_sensors[sensor_index].port,
                                              app_dht22_sensors[sensor_index].pin,
                                              &temperature, &humidity);

    if (sensor_status == DHT22_STATUS_OK)
    {
      if (add_to_payload == true)
      {
        CayenneLppAddTemperature(app_dht22_sensors[sensor_index].temperature_channel, temperature);
        CayenneLppAddRelativeHumidity(app_dht22_sensors[sensor_index].humidity_channel, humidity);
      }

      if (log_to_termite == true)
      {
        int32_t temperature_tenths = (int32_t)(temperature * 10.0f);
        uint32_t temperature_absolute = (temperature_tenths < 0) ?
                                        (uint32_t)(-temperature_tenths) : (uint32_t)temperature_tenths;
        uint32_t humidity_tenths = (uint32_t)(humidity * 10.0f);

        FIELD_LOG(TS_OFF, VLEVEL_M,
                  "[DHT22 %u] Temperatura: %s%u.%u C | Umidade: %u.%u %%\r\n",
                  (unsigned int)(sensor_index + 1U), (temperature_tenths < 0) ? "-" : "",
                  (unsigned int)(temperature_absolute / 10U),
                  (unsigned int)(temperature_absolute % 10U),
                  (unsigned int)(humidity_tenths / 10U),
                  (unsigned int)(humidity_tenths % 10U));
      }
    }
    else if (log_to_termite == true)
    {
      FIELD_LOG(TS_OFF, VLEVEL_M, "[DHT22 %u] Falha na leitura (status=%d)\r\n",
                (unsigned int)(sensor_index + 1U), sensor_status);
    }
  }
}

static void SendTxData(uint8_t port)
{
  smtc_modem_return_code_t uplink_status;
  uint8_t battery_level = GetBatteryLevel();

  CayenneLppInit();
  ReadDht22Sensors(true, false);

  CayenneLppAddAnalogInput(APP_BATTERY_CHANNEL, (float)battery_level / 254.0f);

  uint8_t *payload = CayenneLppGetBuffer();
  uint8_t length = CayenneLppGetSize();

  uplink_status = smtc_modem_request_uplink(STACK_ID, port, false, payload, length);
  FIELD_LOG(TS_ON, VLEVEL_M, "[LORAWAN] Uplink solicitado: porta=%u bytes=%u rc=%d\r\n",
            (unsigned int)port, (unsigned int)length, uplink_status);

  if (uplink_status != SMTC_MODEM_RC_OK)
  {
    ASSERT_SMTC_MODEM_RC(uplink_status);
  }

  {
    smtc_modem_status_mask_t status_mask = 0;
    smtc_modem_get_status(STACK_ID, &status_mask);

    if (CertMode || ((status_mask & SMTC_MODEM_STATUS_JOINED) != SMTC_MODEM_STATUS_JOINED))
    {
      ASSERT_SMTC_MODEM_RC(smtc_modem_alarm_start_timer(CERT_TX_DUTYCYCLE));
    }
    else
    {
      ASSERT_SMTC_MODEM_RC(smtc_modem_alarm_start_timer(APP_TELEMETRY_PERIOD_SECONDS));
    }
  }
}


/* USER CODE BEGIN PrFD_LedEvents */
static bool ConfigureAu915Fsb(void)
{
#if defined(REGION_AU915)
  lr1_stack_mac_t *lr1mac = lorawan_api_stack_mac_get(STACK_ID);
  const uint16_t fsb_mask = (uint16_t)(1UL << (APP_LORA_AU915_FSB - 1U));

  if ((lr1mac == NULL) || (lr1mac->real == NULL))
  {
    return false;
  }

  /* ChMaskCntl=5 selects complete AU915 banks: bit 0=FSB1 ... bit 7=FSB8. */
  if (smtc_real_build_channel_mask(lr1mac->real, 5U, fsb_mask) != OKCHANNEL)
  {
    return false;
  }

  smtc_real_set_channel_mask(lr1mac->real);
  return true;
#else
  return false;
#endif
}

static void OnTxTimerLedEvent(void *context)
{
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); /* LED_GREEN */
}

static void OnRxTimerLedEvent(void *context)
{
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); /* LED_BLUE */
}

static void OnJoinTimerLedEvent(void *context)
{
  HAL_GPIO_TogglePin(APP_E77_LED2_PORT, APP_E77_LED2_PIN);
}

static void E77_LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(APP_E77_LED1_PORT, APP_E77_LED1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(APP_E77_LED2_PORT, APP_E77_LED2_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = APP_E77_LED1_PIN | APP_E77_LED2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void E77_LED_AllOff(void)
{
  HAL_GPIO_WritePin(APP_E77_LED1_PORT, APP_E77_LED1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(APP_E77_LED2_PORT, APP_E77_LED2_PIN, GPIO_PIN_RESET);
}

static void E77_LED_BootOk(void)
{
  for (uint8_t i = 0; i < 3; i++)
  {
    HAL_GPIO_WritePin(APP_E77_LED1_PORT, APP_E77_LED1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(APP_E77_LED2_PORT, APP_E77_LED2_PIN, GPIO_PIN_SET);
    HAL_Delay(120);

    HAL_GPIO_WritePin(APP_E77_LED1_PORT, APP_E77_LED1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(APP_E77_LED2_PORT, APP_E77_LED2_PIN, GPIO_PIN_RESET);
    HAL_Delay(120);
  }
}
/* USER CODE END PrFD_LedEvents */

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
static void OnSleepTimerEvent(void *context)
{
  /* USER CODE BEGIN OnSleepTimerEvent_1 */

  if (lora_engine_enabled == false)
  {
    lora_join_request = true;
    FIELD_LOG(TS_ON, VLEVEL_M, "[BAIXO CONSUMO] Pausa encerrada; tentando conectar novamente\r\n");
  }

  /* USER CODE END OnSleepTimerEvent_1 */
  FIELD_LOG(TS_ON, VLEVEL_H, "[BAIXO CONSUMO] Timer de despertar acionado\r\n");

  /* USER CODE BEGIN OnSleepTimerEvent_Last */

  /* USER CODE END OnSleepTimerEvent_Last */
}
#endif /*(LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0) */

/* --- EOF ------------------------------------------------------------------ */
