/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "stm32_lpm.h"
#include "dht22.h"
#include "mhz19e.h"
#if 0
#include "se-identity.h"
#include "radio_board_if.h"
#include "radio_board_if.c"
#include "radio_conf.h"
#endif

#include "math.h"
#include "LoRaMac.h"
/* USER CODE END Includes */


/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

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

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */
static const char *slotStrings[] = { "1", "2", "C", "C_MC", "P", "P_MC" };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief callback when LoRaWAN application has sent a frame
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRaWAN application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
  * @brief callback when LoRaWAN Beacon status is updated
  * @param params status of Last Beacon
  */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
  * @brief callback when system time has been updated
  */
static void OnSysTimeUpdate(void);

/**
  * @brief callback when LoRaWAN application Class is changed
  * @param deviceClass new class
  */
static void OnClassChange(DeviceClass_t deviceClass);

/**
  * @brief  LoRa store context in Non Volatile Memory
  */
static void StoreContext(void);

/**
  * @brief  stop current LoRa execution to switch into non default Activation mode
  */
static void StopJoin(void);

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */
static void OnStopJoinTimerEvent(void *context);

/**
  * @brief  Notifies the upper layer that the NVM context has changed
  * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
  */
static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
  * @brief  Store the NVM Data context to the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were stored
  */
static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * @brief  Restore the NVM Data context from the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were restored
  */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * Will be called each time a Radio IRQ is handled by the MAC layer
  *
  */
static void OnMacProcessNotify(void);

/**
  * @brief Change the periodicity of the uplink frames
  * @param periodicity uplink frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnTxPeriodicityChanged(uint32_t periodicity);

/**
  * @brief Change the confirmation control of the uplink frames
  * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
  * @note Compliance test protocol callbacks
  */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
  * @brief Change the periodicity of the ping slot frames
  * @param pingSlotPeriodicity ping slot frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/**
  * @brief Will be called to reset the system
  * @note Compliance test protocol callbacks
  */
static void OnSystemReset(void);

/* USER CODE BEGIN PFP */
static void ReadDht22Sensors(bool add_to_payload, bool log_to_termite, int8_t active_sensor_index);
/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief LoRaWAN default activation type
  */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN force rejoin even if the NVM context is restored
  */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =              GetBatteryLevel,
  .GetTemperature =               GetTemperatureLevel,
  .GetUniqueId =                  GetUniqueId,
  .GetDevAddr =                   GetDevAddr,
  .OnRestoreContextRequest =      OnRestoreContextRequest,
  .OnStoreContextRequest =        OnStoreContextRequest,
  .OnMacProcess =                 OnMacProcessNotify,
  .OnNvmDataChange =              OnNvmDataChange,
  .OnJoinRequest =                OnJoinRequest,
  .OnTxData =                     OnTxData,
  .OnRxData =                     OnRxData,
  .OnBeaconStatusChange =         OnBeaconStatusChange,
  .OnSysTimeUpdate =              OnSysTimeUpdate,
  .OnClassChange =                OnClassChange,
  .OnTxPeriodicityChanged =       OnTxPeriodicityChanged,
  .OnTxFrameCtrlChanged =         OnTxFrameCtrlChanged,
  .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
  .OnSystemReset =                OnSystemReset,
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .TxPower =                  LORAWAN_DEFAULT_TX_POWER,
  .PingSlotPeriodicity =      LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
  .RxBCTimeout =              LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT
};

/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_TIMER;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/**
  * @brief Tx Timer period
  */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
  * @brief Join Timer period
  */
static UTIL_TIMER_Object_t StopJoinTimer;


/* USER CODE BEGIN PV */
static const AppDht22Sensor_t app_dht22_sensors[] =
{
  {APP_DHT22_1_PORT, APP_DHT22_1_PIN, APP_DHT22_1_TEMPERATURE_CHANNEL, APP_DHT22_1_HUMIDITY_CHANNEL},
  {APP_DHT22_2_PORT, APP_DHT22_2_PIN, APP_DHT22_2_TEMPERATURE_CHANNEL, APP_DHT22_2_HUMIDITY_CHANNEL},
};

#define APP_DHT22_SENSOR_COUNT ((uint8_t)(sizeof(app_dht22_sensors) / sizeof(app_dht22_sensors[0])))

/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };
/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_LV */
	  uint32_t feature_version = 0UL;
	  static MibRequestConfirm_t mibReq;
  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */
  for (uint8_t sensor_index = 0U; sensor_index < APP_DHT22_SENSOR_COUNT; sensor_index++)
  {
    DHT22_Init(app_dht22_sensors[sensor_index].port, app_dht22_sensors[sensor_index].pin);
  }
  HAL_Delay(APP_DHT22_BOOT_SETTLE_MS);

	 /* Get LoRaWAN APP version*/
	  APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
	          (uint8_t)(APP_VERSION_MAIN),
	          (uint8_t)(APP_VERSION_SUB1),
	          (uint8_t)(APP_VERSION_SUB2));

	  /* Get MW LoRaWAN info */
	  APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION:  V%X.%X.%X\r\n",
	          (uint8_t)(LORAWAN_VERSION_MAIN),
	          (uint8_t)(LORAWAN_VERSION_SUB1),
	          (uint8_t)(LORAWAN_VERSION_SUB2));

	  /* Get MW SubGhz_Phy info */
	  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
	          (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
	          (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
	          (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

	  /* Get LoRaWAN Link Layer info */
	  LmHandlerGetVersion(LORAMAC_HANDLER_L2_VERSION, &feature_version);
	  APP_LOG(TS_OFF, VLEVEL_M, "L2_SPEC_VERSION:     V%X.%X.%X\r\n",
	          (uint8_t)(feature_version >> 24),
	          (uint8_t)(feature_version >> 16),
	          (uint8_t)(feature_version >> 8));

	  /* Get LoRaWAN Regional Parameters info */
	  LmHandlerGetVersion(LORAMAC_HANDLER_REGION_VERSION, &feature_version);
	  APP_LOG(TS_OFF, VLEVEL_M, "RP_SPEC_VERSION:     V%X-%X.%X.%X\r\n",
	          (uint8_t)(feature_version >> 24),
	          (uint8_t)(feature_version >> 16),
	          (uint8_t)(feature_version >> 8),
	          (uint8_t)(feature_version));

	  if (FLASH_IF_Init(NULL) != FLASH_IF_OK)
	  {
	    Error_Handler();
	  }

  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT, OnStopJoinTimerEvent, NULL);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU, StopJoin);

  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
	 // Modificacoes para LoRaWAN TTN AU915 - A.Lemke 17/08/2024
	 // Obs: TTN's channel mask is 02 0000 0000 0000 FF00 (FSB2)
	 uint16_t ChannelsMask[6] = { 0xFF00, 0x0000, 0x0000, 0x0000, 0x0002, 0x0000 };   //Alterado p/ teste A.Lemke 23/08/2024

	 mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
	 mibReq.Param.ChannelsMask = ChannelsMask;
	 LoRaMacMibSetRequestConfirm(&mibReq);
	 
	 // Força o desligamento do ADR e o Data Rate para DR_3, ignorando a memória NVM
	 mibReq.Type = MIB_ADR;
	 mibReq.Param.AdrEnable = false;
	 LoRaMacMibSetRequestConfirm(&mibReq);

	 mibReq.Type = MIB_CHANNELS_DATARATE;
	 mibReq.Param.ChannelsDatarate = DR_2;
	 LoRaMacMibSetRequestConfirm(&mibReq);
	 
	 LmHandlerSetAdrEnable(false);
	 LmHandlerSetTxDatarate(DR_2);
  /* USER CODE END LoRaWAN_Init_2 */

	 /* Impede a placa de dormir profundamente durante o Join para não perder a janela de RX */
	 UTIL_LPM_SetStopMode((1 << CFG_LPM_APPLI_Id), UTIL_LPM_DISABLE);

     /* Inicializa o MH-Z19E via USART1 (PB6=TX, PB7=RX) logo no inicio */
     if (MHZ19E_Init() == HAL_OK)
     {
       APP_LOG(TS_OFF, VLEVEL_M, "[MHZ19E] UART USART1 PB6/PB7 @ 9600 inicializada com SUCESSO!\r\n");
     }
     else
     {
       APP_LOG(TS_OFF, VLEVEL_M, "[MHZ19E] ERRO GRAVE: Falha ao inicializar USART1 (HAL_ERROR)!\r\n");
     }

     /* Verifica e exibe o estado dos sensores no boot com retry */
     APP_LOG(TS_OFF, VLEVEL_M, "\r\n[DBG] Verificando Sensores DHT22 no Boot...\r\n");
     
     /* Tentativa dupla para o DHT22 devido a flutuacoes no checksum */
     for (uint8_t sensor_idx = 0; sensor_idx < APP_DHT22_SENSOR_COUNT; sensor_idx++)
     {
         uint8_t retries = 3;
         while (retries > 0)
         {
             float temp = 0, hum = 0;
             DHT22_Status_t st = DHT22_Read(app_dht22_sensors[sensor_idx].port, app_dht22_sensors[sensor_idx].pin, &temp, &hum);
             if (st == DHT22_STATUS_OK)
             {
                 int32_t t_tenths = (int32_t)(temp * 10.0f);
                 uint32_t t_abs = (t_tenths < 0) ? (uint32_t)(-t_tenths) : (uint32_t)t_tenths;
                 uint32_t h_tenths = (uint32_t)(hum * 10.0f);
                 APP_LOG(TS_OFF, VLEVEL_M, "[DHT22 %u] Temperatura: %s%u.%u C | Umidade: %u.%u %%\r\n", 
                         (unsigned int)(sensor_idx + 1), (t_tenths < 0) ? "-" : "", 
                         (unsigned int)(t_abs / 10U), (unsigned int)(t_abs % 10U),
                         (unsigned int)(h_tenths / 10U), (unsigned int)(h_tenths % 10U));
                 break; /* Sucesso, sai do loop de retry */
             }
             else
             {
                 retries--;
                 if (retries == 0) {
                     APP_LOG(TS_OFF, VLEVEL_M, "[DHT22 %u] Falha na leitura (status=%d) apos tentativas.\r\n", (unsigned int)(sensor_idx + 1), (int)st);
                 } else {
                     HAL_Delay(2000); /* DHT22 exige 2s entre leituras */
                 }
             }
         }
     }

     APP_LOG(TS_OFF, VLEVEL_M, "\r\n[MHZ19E] Aguardando 60 segundos de aquecimento (WARM-UP) antes da primeira leitura...\r\n");
     HAL_Delay(60000U); /* Pausa o boot por 1 minuto para o sensor estabilizar */
     
     uint16_t boot_co2 = 0;
     MHZ19E_Status_t boot_co2_status = MHZ19E_Read(&boot_co2);
     if (boot_co2_status == MHZ19E_STATUS_OK)
     {
         APP_LOG(TS_OFF, VLEVEL_M, "[MHZ19E] Leitura de Boot CO2: %u ppm\r\n", (unsigned int)boot_co2);
     }
     else
     {
         APP_LOG(TS_OFF, VLEVEL_M, "[MHZ19E] Falha na leitura de Boot CO2 (status=%d)\r\n", (int)boot_co2_status);
     }

     APP_LOG(TS_OFF, VLEVEL_M, "------------------------------------------\r\n");

  LmHandlerJoin(ActivationType, ForceRejoin);


  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */

#if 0 /* User should remove the #if 0 statement and adapt the below code according with his needs*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case  BUT1_Pin:
      /* Note: when "EventType == TX_ON_TIMER" this GPIO is not initialized */
      if (EventType == TX_ON_EVENT)
      {
        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
      }
      break;
    case  BUT2_Pin:
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
      break;
    case  BUT3_Pin:
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
      break;
    default:
      break;
  }
}
#endif

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
  uint8_t RxPort = 0;

  if (params != NULL)
  {
	if (params->IsMcpsIndication)
	{
	  if (appData != NULL)
	  {
		RxPort = appData->Port;
		if (appData->Buffer != NULL)
		{
		  switch (appData->Port)
		  {
			case LORAWAN_SWITCH_CLASS_PORT:
			  /*this port switches the class*/
			  if (appData->BufferSize == 1)
			  {
				switch (appData->Buffer[0])
				{
				  case 0:
				  {
					LmHandlerRequestClass(CLASS_A);
					break;
				  }
				  case 1:
				  {
					LmHandlerRequestClass(CLASS_B);
					break;
				  }
				  case 2:
				  {
					LmHandlerRequestClass(CLASS_C);
					break;
				  }
				  default:
					break;
				}
			  }
			  break;
			case LORAWAN_USER_APP_PORT:
			  if (appData->BufferSize == 1)
			     {

			     }
			  break;

			default:

			  break;
		  }
		}
	  }
	}
	if (params->RxSlot < RX_SLOT_NONE)
	{
	  APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | PORT:%d | DR:%d | SLOT:%s | RSSI:%d | SNR:%d\r\n",
			  params->DownlinkCounter, RxPort, params->Datarate, slotStrings[params->RxSlot],
			  params->Rssi, params->Snr);
	}
  }
  /* USER CODE END OnRxData_1 */
}

static void ReadDht22Sensors(bool add_to_payload, bool log_to_termite, int8_t active_sensor_index)
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
      if (add_to_payload == true && (active_sensor_index < 0 || active_sensor_index == sensor_index))
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

        APP_LOG(TS_OFF, VLEVEL_M,
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
      APP_LOG(TS_OFF, VLEVEL_M, "[DHT22 %u] Falha na leitura (status=%d)\r\n",
                (unsigned int)(sensor_index + 1U), sensor_status);
    }
  }
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
	  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
	  UTIL_TIMER_Time_t nextTxIn = 0;

	  APP_LOG(TS_ON, VLEVEL_M, "\r\n======== SendTxData INICIO ========\r\n");

	  bool busy = LmHandlerIsBusy();
	  APP_LOG(TS_OFF, VLEVEL_M, "[DBG] LmHandlerIsBusy = %s\r\n", busy ? "TRUE (abortando)" : "FALSE (ok)");

	  if (busy == false)
	     {
          /* Acende o LED verde (PB4) durante a transmissao */
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
          // BLINDAGEM MÁXIMA: Força DR_3 e ADR OFF através da API oficial do LmHandler
          // garantindo que a biblioteca interna atualize seus parâmetros.
          LmHandlerSetAdrEnable(false);
          LmHandlerSetTxDatarate(DR_2);
          MibRequestConfirm_t mibReq;
          mibReq.Type = MIB_ADR;
          mibReq.Param.AdrEnable = false;
          LoRaMacMibSetRequestConfirm(&mibReq);
          mibReq.Type = MIB_CHANNELS_DATARATE;
          mibReq.Param.ChannelsDatarate = DR_2;
          LoRaMacMibSetRequestConfirm(&mibReq);

          /* Verifica DR e ADR efetivos apos forcar */
          mibReq.Type = MIB_CHANNELS_DATARATE;
          LoRaMacMibGetRequestConfirm(&mibReq);
          APP_LOG(TS_OFF, VLEVEL_M, "[DBG] DR efetivo = DR_%d\r\n", mibReq.Param.ChannelsDatarate);
          mibReq.Type = MIB_ADR;
          LoRaMacMibGetRequestConfirm(&mibReq);
          APP_LOG(TS_OFF, VLEVEL_M, "[DBG] ADR efetivo = %s\r\n", mibReq.Param.AdrEnable ? "ON" : "OFF");

          uint8_t battery_level = LmHandlerCallbacks.GetBatteryLevel();
          APP_LOG(TS_OFF, VLEVEL_M, "[DBG] Battery level = %u\r\n", (unsigned int)battery_level);

          CayenneLppInit();
          
          /* O valor -1 avisa a funcao para adicionar TODOS os sensores no pacote */
          ReadDht22Sensors(true, true, -1);

          /* Leitura do CO2 (MH-Z19E via LPUART1) */
          uint16_t co2_ppm = 0U;
          MHZ19E_Status_t co2_status = MHZ19E_Read(&co2_ppm);
          if (co2_status == MHZ19E_STATUS_OK)
          {
            APP_LOG(TS_OFF, VLEVEL_M, "[MHZ19E] CO2: %u ppm\r\n", (unsigned int)co2_ppm);
            /* Cayenne LPP Analog Input (resolucao 0.01): passar ppm/100 para nao estourar int16.
             * No servidor: valor exibido x 100 = ppm real. Ex: 12.50 = 1250 ppm. */
            CayenneLppAddAnalogInput(APP_MHZ19E_CO2_CHANNEL, (float)co2_ppm / 100.0f);
          }
          else
          {
            APP_LOG(TS_OFF, VLEVEL_M, "[MHZ19E] Falha na leitura CO2 (status=%d)\r\n", (int)co2_status);
          }

          CayenneLppAddAnalogInput(APP_BATTERY_CHANNEL, (float)battery_level / 254.0f);


          uint8_t cayenneSize = CayenneLppGetSize();
          APP_LOG(TS_OFF, VLEVEL_M, "[DBG] CayenneLpp size = %u bytes\r\n", (unsigned int)cayenneSize);

          CayenneLppCopy(AppData.Buffer);
          AppData.BufferSize = cayenneSize;
	      AppData.Port = LORAWAN_USER_APP_PORT;

          APP_LOG(TS_OFF, VLEVEL_M, "[DBG] AppData.Port = %u | AppData.BufferSize = %u\r\n",
                  (unsigned int)AppData.Port, (unsigned int)AppData.BufferSize);

          /* Hex dump do payload */
          APP_LOG(TS_OFF, VLEVEL_M, "[DBG] Payload hex: ");
          for (uint8_t i = 0; i < AppData.BufferSize; i++)
          {
            APP_LOG(TS_OFF, VLEVEL_M, "%02X ", AppData.Buffer[i]);
          }
          APP_LOG(TS_OFF, VLEVEL_M, "\r\n");

          /* Verifica se o payload cabe no DR atual */
          LoRaMacTxInfo_t txInfoDbg;
          LoRaMacStatus_t txPossible = LoRaMacQueryTxPossible(AppData.BufferSize, &txInfoDbg);
          APP_LOG(TS_OFF, VLEVEL_M, "[DBG] LoRaMacQueryTxPossible = %d (0=OK) | MaxSize=%u | CurrentSize=%u\r\n",
                  txPossible, (unsigned int)txInfoDbg.MaxPossibleApplicationDataSize,
                  (unsigned int)txInfoDbg.CurrentPossiblePayloadSize);

          APP_LOG(TS_OFF, VLEVEL_M, "[DBG] IsTxConfirmed = %d (0=UNCONFIRMED, 1=CONFIRMED)\r\n",
                  LmHandlerParams.IsTxConfirmed);

	      status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);

          APP_LOG(TS_OFF, VLEVEL_M, "[DBG] LmHandlerSend retornou = %d\r\n", status);
          switch (status)
          {
            case LORAMAC_HANDLER_SUCCESS:
              APP_LOG(TS_ON, VLEVEL_M, "[DBG] >>> SEND REQUEST OK <<<\r\n");
              break;
            case LORAMAC_HANDLER_BUSY_ERROR:
              APP_LOG(TS_ON, VLEVEL_M, "[DBG] SEND FALHOU: BUSY\r\n");
              break;
            case LORAMAC_HANDLER_NO_NETWORK_JOINED:
              APP_LOG(TS_ON, VLEVEL_M, "[DBG] SEND FALHOU: NOT JOINED. TENTANDO RECONECTAR...\r\n");
              LmHandlerJoin(ActivationType, true);
              break;
            case LORAMAC_HANDLER_COMPLIANCE_RUNNING:
              APP_LOG(TS_ON, VLEVEL_M, "[DBG] SEND FALHOU: COMPLIANCE TEST ATIVO!\r\n");
              break;
            case LORAMAC_HANDLER_CRYPTO_ERROR:
              APP_LOG(TS_ON, VLEVEL_M, "[DBG] SEND FALHOU: CRYPTO ERROR\r\n");
              break;
            case LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED:
              nextTxIn = LmHandlerGetDutyCycleWaitTime();
              APP_LOG(TS_ON, VLEVEL_M, "[DBG] SEND FALHOU: DUTY CYCLE (~%d s)\r\n", (int)(nextTxIn / 1000));
              break;
            case LORAMAC_HANDLER_PAYLOAD_LENGTH_RESTRICTED:
              APP_LOG(TS_ON, VLEVEL_M, "[DBG] SEND: PAYLOAD TRUNCADO (muito grande p/ DR)\r\n");
              break;
            default:
              APP_LOG(TS_ON, VLEVEL_M, "[DBG] SEND FALHOU: ERRO GENERICO (%d)\r\n", status);
              break;
          }
	     }

	  if (EventType == TX_ON_TIMER)
	  {
	    UTIL_TIMER_Stop(&TxTimer);
	    UTIL_TIMER_SetPeriod(&TxTimer, MAX(nextTxIn, TxPeriodicity));
	    UTIL_TIMER_Start(&TxTimer);
	  }

	  APP_LOG(TS_ON, VLEVEL_M, "======== SendTxData FIM ========\r\n");
  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */

/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
 if ((params != NULL))
  {
	APP_LOG(TS_OFF, VLEVEL_M, "\r\n[DBG-TX] OnTxData chamado | IsMcpsConfirm=%d | Status=%d\r\n",
	        params->IsMcpsConfirm, params->Status);
	/* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
	if (params->IsMcpsConfirm != 0)
	{
      /* Apaga o LED verde (PB4) pois a transmissao/recepcao acabou */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	  APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
	  APP_LOG(TS_OFF, VLEVEL_M, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d | SIZE:%d",
			  params->UplinkCounter,
			  params->AppData.Port, params->Datarate, params->TxPower,
			  params->AppData.BufferSize);

	  APP_LOG(TS_OFF, VLEVEL_M, " | MSG TYPE:");
	  if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
	  {
		APP_LOG(TS_OFF, VLEVEL_M, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
	  }
	  else
	  {
		APP_LOG(TS_OFF, VLEVEL_M, "UNCONFIRMED\r\n");
	  }
	}
	else
	{
	  APP_LOG(TS_OFF, VLEVEL_M, "[DBG-TX] MLME event (nao eh uplink de dados)\r\n");
	}
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
   if (joinParams != NULL)
	  {
	    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
	    {
	      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
	      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
	      {
	        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
	      }
	      else
	      {
	        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
	      }

	      /* Debug: mostra a mascara de canais ANTES de re-aplicar */
	      MibRequestConfirm_t mibDbg;
	      mibDbg.Type = MIB_CHANNELS_MASK;
	      LoRaMacMibGetRequestConfirm(&mibDbg);
	      APP_LOG(TS_OFF, VLEVEL_M, "[DBG] ChMask ANTES: %04X %04X %04X %04X %04X %04X\r\n",
	              mibDbg.Param.ChannelsMask[0], mibDbg.Param.ChannelsMask[1],
	              mibDbg.Param.ChannelsMask[2], mibDbg.Param.ChannelsMask[3],
	              mibDbg.Param.ChannelsMask[4], mibDbg.Param.ChannelsMask[5]);

	      /* Re-aplica FSB2, DR_3 e ADR OFF apos join (TTN pode sobrescrever via CFList) */
	      uint16_t ChannelsMask[6] = { 0xFF00, 0x0000, 0x0000, 0x0000, 0x0002, 0x0000 };
	      MibRequestConfirm_t mibReq;

	      mibReq.Type = MIB_CHANNELS_MASK;
	      mibReq.Param.ChannelsMask = ChannelsMask;
	      LoRaMacMibSetRequestConfirm(&mibReq);

	      mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
	      mibReq.Param.ChannelsMask = ChannelsMask;
	      LoRaMacMibSetRequestConfirm(&mibReq);

	      mibReq.Type = MIB_ADR;
	      mibReq.Param.AdrEnable = false;
	      LoRaMacMibSetRequestConfirm(&mibReq);

	      mibReq.Type = MIB_CHANNELS_DATARATE;
	      mibReq.Param.ChannelsDatarate = DR_2;
	      LoRaMacMibSetRequestConfirm(&mibReq);

	      LmHandlerSetAdrEnable(false);
	      LmHandlerSetTxDatarate(DR_2);

          /* Configura o LED verde (PB4) como Output e inicia o Timer de 5s */
          GPIO_InitTypeDef GPIO_InitStruct = {0};
          GPIO_InitStruct.Pin = GPIO_PIN_4; // APP_E77_LED1_PIN
          GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
          GPIO_InitStruct.Pull = GPIO_NOPULL;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
          HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	      /* Debug: mostra a mascara de canais DEPOIS de re-aplicar */
	      mibDbg.Type = MIB_CHANNELS_MASK;
	      LoRaMacMibGetRequestConfirm(&mibDbg);
	      APP_LOG(TS_OFF, VLEVEL_M, "[DBG] ChMask DEPOIS: %04X %04X %04X %04X %04X %04X\r\n",
	              mibDbg.Param.ChannelsMask[0], mibDbg.Param.ChannelsMask[1],
	              mibDbg.Param.ChannelsMask[2], mibDbg.Param.ChannelsMask[3],
	              mibDbg.Param.ChannelsMask[4], mibDbg.Param.ChannelsMask[5]);

	      /* Agenda a primeira leitura para 5 segundos apos o Join para evitar colisão na MAC layer! */
	      UTIL_TIMER_Stop(&TxTimer);
	      UTIL_TIMER_SetPeriod(&TxTimer, 5000);
	      UTIL_TIMER_Start(&TxTimer);
	    }
	    else
	    {
	      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED. TENTANDO DE NOVO EM 15s...\r\n");
	      UTIL_TIMER_Stop(&TxTimer);
	      UTIL_TIMER_SetPeriod(&TxTimer, APP_LORA_JOIN_RETRY_SECONDS * 1000U);
	      UTIL_TIMER_Start(&TxTimer);
	    }
	    
	    /* Independente de sucesso ou falha, o processo de Join acabou (ou vai demorar pra tentar de novo).
	       Podemos liberar a placa para dormir profundamente de novo! */
	    UTIL_LPM_SetStopMode((1 << CFG_LPM_APPLI_Id), UTIL_LPM_ENABLE);

	    APP_LOG(TS_OFF, VLEVEL_M, "###### U/L FRAME:JOIN | DR:%d | PWR:%d\r\n", joinParams->Datarate, joinParams->TxPower);
	  }
  /* USER CODE END OnJoinRequest_1 */
}

static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params)
{
  /* USER CODE BEGIN OnBeaconStatusChange_1 */
   if (params != NULL)
	  {
	    switch (params->State)
	    {
	      default:
	      case LORAMAC_HANDLER_BEACON_LOST:
	      {
	        APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON LOST\r\n");
	        break;
	      }
	      case LORAMAC_HANDLER_BEACON_RX:
	      {
	        APP_LOG(TS_OFF, VLEVEL_M,
	                "\r\n###### BEACON RECEIVED | DR:%d | RSSI:%d | SNR:%d | FQ:%d | TIME:%d | DESC:%d | "
	                "INFO:02X%02X%02X %02X%02X%02X\r\n",
	                params->Info.Datarate, params->Info.Rssi, params->Info.Snr, params->Info.Frequency,
	                params->Info.Time.Seconds, params->Info.GwSpecific.InfoDesc,
	                params->Info.GwSpecific.Info[0], params->Info.GwSpecific.Info[1],
	                params->Info.GwSpecific.Info[2], params->Info.GwSpecific.Info[3],
	                params->Info.GwSpecific.Info[4], params->Info.GwSpecific.Info[5]);
	        break;
	      }
	      case LORAMAC_HANDLER_BEACON_NRX:
	      {
	        APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON NOT RECEIVED\r\n");
	        break;
	      }
	    }
	  }
  /* USER CODE END OnBeaconStatusChange_1 */
}

static void OnSysTimeUpdate(void)
{
  /* USER CODE BEGIN OnSysTimeUpdate_1 */

  /* USER CODE END OnSysTimeUpdate_1 */
}

static void OnClassChange(DeviceClass_t deviceClass)
{
  /* USER CODE BEGIN OnClassChange_1 */
  APP_LOG(TS_OFF, VLEVEL_M, "Switch to Class %c done\r\n", "ABC"[deviceClass]);
  /* USER CODE END OnClassChange_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

static void OnTxPeriodicityChanged(uint32_t periodicity)
{
  /* USER CODE BEGIN OnTxPeriodicityChanged_1 */

  /* USER CODE END OnTxPeriodicityChanged_1 */
  TxPeriodicity = periodicity;

  if (TxPeriodicity == 0)
  {
    /* Revert to application default periodicity */
    TxPeriodicity = APP_TX_DUTYCYCLE;
  }

  /* Update timer periodicity */
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, TxPeriodicity);
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxPeriodicityChanged_2 */

  /* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
  /* USER CODE BEGIN OnTxFrameCtrlChanged_1 */

  /* USER CODE END OnTxFrameCtrlChanged_1 */
  LmHandlerParams.IsTxConfirmed = isTxConfirmed;
  /* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

  /* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */

  /* USER CODE END OnPingSlotPeriodicityChanged_1 */
  LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

  /* USER CODE END OnPingSlotPeriodicityChanged_2 */
}

static void OnSystemReset(void)
{
  /* USER CODE BEGIN OnSystemReset_1 */

  /* USER CODE END OnSystemReset_1 */
  if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt()) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
  {
    NVIC_SystemReset();
  }
  /* USER CODE BEGIN OnSystemReset_Last */

  /* USER CODE END OnSystemReset_Last */
}

static void StopJoin(void)
{
  /* USER CODE BEGIN StopJoin_1 */

  /* USER CODE END StopJoin_1 */

  UTIL_TIMER_Stop(&TxTimer);

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop())
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
    if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP)
    {
      ActivationType = ACTIVATION_TYPE_OTAA;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
    }
    else
    {
      ActivationType = ACTIVATION_TYPE_ABP;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
    }
    LmHandlerConfigure(&LmHandlerParams);
    LmHandlerJoin(ActivationType, true);
    UTIL_TIMER_Start(&TxTimer);
  }
  UTIL_TIMER_Start(&StopJoinTimer);
  /* USER CODE BEGIN StopJoin_Last */

  /* USER CODE END StopJoin_Last */
}

static void OnStopJoinTimerEvent(void *context)
{
  /* USER CODE BEGIN OnStopJoinTimerEvent_1 */

  /* USER CODE END OnStopJoinTimerEvent_1 */
  if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE)
  {
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
  }
  /* USER CODE BEGIN OnStopJoinTimerEvent_Last */

  /* USER CODE END OnStopJoinTimerEvent_Last */
}

static void StoreContext(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  /* USER CODE BEGIN StoreContext_1 */

  /* USER CODE END StoreContext_1 */
  status = LmHandlerNvmDataStore();

  if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
  }
  else if (status == LORAMAC_HANDLER_ERROR)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
  }
  /* USER CODE BEGIN StoreContext_Last */

  /* USER CODE END StoreContext_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
  /* USER CODE BEGIN OnNvmDataChange_1 */

  /* USER CODE END OnNvmDataChange_1 */
  if (state == LORAMAC_HANDLER_NVM_STORE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
  }
  /* USER CODE BEGIN OnNvmDataChange_Last */

  /* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */

  /* USER CODE END OnStoreContextRequest_1 */
  /* store nvm in flash */
  if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
  {
    FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (const void *)nvm, nvm_size);
  }
  /* USER CODE BEGIN OnStoreContextRequest_Last */

  /* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnRestoreContextRequest_1 */

  /* USER CODE END OnRestoreContextRequest_1 */
  FLASH_IF_Read(nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size);
  /* USER CODE BEGIN OnRestoreContextRequest_Last */

  /* USER CODE END OnRestoreContextRequest_Last */
}



