#ifndef MHZ19E_H
#define MHZ19E_H

#include <stdint.h>
#include "stm32wlxx_hal.h"

/**
  * @brief MH-Z19E driver return status codes.
  */
typedef enum
{
  MHZ19E_STATUS_OK             = 0, /*!< Leitura OK, checksum válido               */
  MHZ19E_STATUS_TIMEOUT        = 1, /*!< Sem resposta dentro do timeout             */
  MHZ19E_STATUS_CHECKSUM_ERROR = 2, /*!< Resposta recebida, mas checksum inválido   */
  MHZ19E_STATUS_INVALID_FRAME  = 3  /*!< Bytes de início de frame inválidos         */
} MHZ19E_Status_t;

/**
  * @brief  Inicializa o USART1 (PB6=TX, PB7=RX) para comunicação com o MH-Z19E.
  *         Chamar uma única vez no boot, antes de qualquer leitura.
  * @retval HAL_OK se a UART foi configurada com sucesso, HAL_ERROR caso contrário.
  */
HAL_StatusTypeDef MHZ19E_Init(void);

/**
  * @brief  Lê a concentração de CO2 do sensor MH-Z19E via UART.
  * @param  co2_ppm  Ponteiro para armazenar o valor de CO2 em ppm (400–5000).
  * @retval MHZ19E_Status_t
  * @note   Para payload Cayenne LPP, passar (float)co2_ppm / 100.0f para
  *         CayenneLppAddAnalogInput() — o servidor exibirá X.XX (× 100 = ppm real).
  */
MHZ19E_Status_t MHZ19E_Read(uint16_t *co2_ppm);

#endif /* MHZ19E_H */
