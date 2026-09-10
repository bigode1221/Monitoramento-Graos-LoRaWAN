---
description: Contexto geral e particularidades críticas do projeto de Monitoramento Inteligente da Qualidade de Grãos (LoRaWAN).
globs: *
---

# Contexto do Projeto
Projeto "Monitoramento Inteligente da Qualidade de Grãos" rodando em módulo EByte E77-900M22S (STM32WL CubeIDE).

# Hardware / Sensores
* Lê 2 sensores DHT22 nos pinos `PA0` e `PA1`.
* Lê 1 sensor de CO2 MH-Z19E via USART1 nos pinos `PB6` (TX) e `PB7` (RX).
* Lê a voltagem da bateria pelo ADC interno.
* O Payload é empacotado no formato Cayenne LPP (DHT1 ch1, DHT2 ch2, CO2 ch3 [ppm/100], Bateria ch5).
* Possui um LED indicador de TX no pino `PB4`.

# Rede LoRaWAN
* Região: **AU915**, Sub-banda: **FSB2**.
* Ativação via **OTAA**.
* Tempo de telemetria é de 30 minutos.

# Particularidades Críticas no Código
1. O **Data Rate é forçado em DR_2** e o **ADR está desativado** (hardcoded em `lora_app.c` no envio e no init).
2. O **Sleep Mode (StopMode) fica desativado** durante o Join para o microcontrolador não dormir e não perder as janelas de RX.

# Repositório GitHub
* Repositório: `https://github.com/bigode1221/Monitoramento-Graos-LoRaWAN`
* Branch principal: `main`
* Conta: `bigode1221` (Ryan Kevin <ryankevin374@gmail.com>)
