/**
  ******************************************************************************
  * @file    mhz19e.c
  * @brief   Driver para sensor de CO2 MH-Z19E via LPUART1
  *          Pinos: PB10 = LPUART1_RX (← sensor TXD, Pin 6)
  *                 PB11 = LPUART1_TX (→ sensor RXD, Pin 5)
  *          Protocolo: 9600 baud, 8N1
  ******************************************************************************
  */

#include "mhz19e.h"
#include "stm32wlxx_hal.h"

/* -------------------------------------------------------------------------- */
/* Private defines                                                             */
/* -------------------------------------------------------------------------- */

/** Timeout para TX e RX UART, em ms. */
#define MHZ19E_UART_TIMEOUT_MS   1000U

/** Byte de início de resposta válida do sensor. */
#define MHZ19E_FRAME_START       0xFFU
#define MHZ19E_FRAME_CMD_READ    0x86U

/* -------------------------------------------------------------------------- */
/* Private variables                                                           */
/* -------------------------------------------------------------------------- */

/** Handle UART do USART1, interno ao driver. */
static UART_HandleTypeDef huart1_mhz19e;

/** Comando de leitura de CO2 (9 bytes, checksum pré-calculado). */
static const uint8_t s_cmd_read_co2[9] = {
  0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79
};

/* -------------------------------------------------------------------------- */
/* Public functions                                                            */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef MHZ19E_Init(void)
{
  /* Configuração do USART1: 9600 baud, 8N1, sem controle de fluxo.
   * O HAL_UART_MspInit() (em stm32wlxx_hal_msp.c) configura PB6 (TX) e PB7 (RX). */
  huart1_mhz19e.Instance                    = USART1;
  huart1_mhz19e.Init.BaudRate               = 9600U;
  huart1_mhz19e.Init.WordLength             = UART_WORDLENGTH_8B;
  huart1_mhz19e.Init.StopBits               = UART_STOPBITS_1;
  huart1_mhz19e.Init.Parity                 = UART_PARITY_NONE;
  huart1_mhz19e.Init.Mode                   = UART_MODE_TX_RX;
  huart1_mhz19e.Init.HwFlowCtl             = UART_HWCONTROL_NONE;
  huart1_mhz19e.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart1_mhz19e.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1_mhz19e.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  huart1_mhz19e.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  return HAL_UART_Init(&huart1_mhz19e);
}

MHZ19E_Status_t MHZ19E_Read(uint16_t *co2_ppm)
{
  uint8_t rx_buf[9] = {0};
  uint8_t checksum  = 0xFFU;
  MHZ19E_Status_t last_status = MHZ19E_STATUS_TIMEOUT;

  if (co2_ppm == NULL)
  {
    return MHZ19E_STATUS_INVALID_FRAME;
  }

  *co2_ppm = 0U;

  for (uint8_t attempt = 0U; attempt < 2U; attempt++)
  {
    /* 0. Limpa ORE (Overrun Error), ruído e framing que possam ter acumulado durante
     *    o delay de 60s ou no boot, e esvazia quaisquer bytes residuais no RDR.
     *    Sem isso, o ORE bloqueia a recepção de novos bytes no hardware STM32! */
    __HAL_UART_CLEAR_FLAG(&huart1_mhz19e, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    while (__HAL_UART_GET_FLAG(&huart1_mhz19e, UART_FLAG_RXNE) == SET)
    {
      (void)huart1_mhz19e.Instance->RDR;
    }

    /* 1. Envia o comando de leitura (9 bytes) */
    if (HAL_UART_Transmit(&huart1_mhz19e,
                          (uint8_t *)s_cmd_read_co2,
                          9U,
                          MHZ19E_UART_TIMEOUT_MS) != HAL_OK)
    {
      last_status = MHZ19E_STATUS_TIMEOUT;
      HAL_Delay(100U);
      continue;
    }

    /* 2. Aguarda e recebe a resposta de 9 bytes */
    if (HAL_UART_Receive(&huart1_mhz19e,
                         rx_buf,
                         9U,
                         MHZ19E_UART_TIMEOUT_MS) != HAL_OK)
    {
      last_status = MHZ19E_STATUS_TIMEOUT;
      HAL_Delay(100U);
      continue;
    }

    /* 3. Valida bytes de início do frame */
    if ((rx_buf[0] != MHZ19E_FRAME_START) || (rx_buf[1] != MHZ19E_FRAME_CMD_READ))
    {
      last_status = MHZ19E_STATUS_INVALID_FRAME;
      HAL_Delay(100U);
      continue;
    }

    /* 4. Calcula e valida o checksum: 0xFF - soma(bytes[1..7]) + 1 */
    checksum = 0xFFU;
    for (uint8_t i = 1U; i <= 7U; i++)
    {
      checksum -= rx_buf[i];
    }
    checksum += 1U;

    if (rx_buf[8] != checksum)
    {
      last_status = MHZ19E_STATUS_CHECKSUM_ERROR;
      HAL_Delay(100U);
      continue;
    }

    /* 5. Extrai o valor de CO2: byte[2] = HIGH, byte[3] = LOW */
    *co2_ppm = ((uint16_t)rx_buf[2] << 8U) | (uint16_t)rx_buf[3];

    return MHZ19E_STATUS_OK;
  }

  return last_status;
}
