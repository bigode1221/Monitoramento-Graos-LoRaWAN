---
description: Datasheet completo do MH-Z19E CO2 NDIR (400-5000ppm) lido do site oficial Winsen. Referência para implementação do driver no projeto de Monitoramento de Grãos NO TRANSPORTE (dentro do caminhão).
globs: *
---

## ⚠️ CONTEXTO DE USO: TRANSPORTE EM CAMINHÃO (não silo estático)

O dispositivo fica **dentro do baú/carroceria do caminhão** que transporta os grãos. Isso muda várias considerações:

### ABC (Auto-Baseline Correction) — REAVALIAÇÃO
- Diferente do silo fechado, **o caminhão abre e fecha** (carga/descarga). Quando aberto, o CO2 volta a ~400 ppm naturalmente.
- Portanto, o **ABC pode ser mantido ATIVO**, pois o sensor verá ar fresco periodicamente.
- **Decisão recomendada**: Manter ABC ativo (não enviar o comando de desativação).

### Cobertura LoRaWAN em Movimento
- O caminhão se move → cobertura LoRaWAN pode ser **intermitente** em estradas rurais/rodovias.
- Transmissões a cada 30 minutos podem falhar se não houver gateway na rota.
- **Considerar**: buffering local de leituras ou reduzir o período de telemetria (ex: 10 min) para aumentar a chance de transmissão bem-sucedida.

### Temperatura Interna do Baú
- Interior de caminhão no Brasil pode atingir **60–70°C** no verão.
- Limite do MH-Z19E: **50°C de operação** → risco real de dano ou leitura errônea.
- Limite do DHT22: **80°C** (ok), mas STM32WLE5 também tem limite de ~85°C.
- **Considerar**: posicionamento do sensor longe de superfícies metálicas quentes.

### Poeira de Grãos
- O datasheet avisa: **"Do not use the sensor in high dusty environment for long time"**.
- Caminhões graneleiros têm muita poeira durante carga/descarga.
- **Considerar**: encapsulamento do sensor com filtro de membrana (e.g., Gore-Tex ou similar).

### Vibração e Trepidação
- Caminhões em estradas rurais geram vibração intensa.
- O MH-Z19E possui câmara óptica frágil → **evitar pressão mecânica** conforme o datasheet.
- Fixar o módulo com material anti-vibração (espuma, borracha).

### Alimentação — Arquitetura real do hardware
- **Bateria**: 18650 Li-Ion — nominal **3.7V**, máximo **4.2V** (carga completa), mínimo ~**3.0V** (cutoff)
- **Regulador 1** → 3.3V para o STM32WLE5 e DHT22 (step-down/LDO, OK pois Vbat > 3.3V)
- **Regulador 2** → **5V para o MH-Z19E** ← **OBRIGATÓRIO ser um BOOST CONVERTER (step-up)**, pois a bateria (3.7V) está ABAIXO dos 5V necessários. Um LDO comum não funciona aqui!

#### Estimativa de consumo e autonomia (1x 18650 ~2500mAh):
| Componente | Corrente |
|---|---|
| MH-Z19E (média) | ~40mA @ 5V → ~54mA @ 3.7V (boost ~85% efic.) |
| MH-Z19E (pico) | 125mA @ 5V → ~200mA @ 3.7V |
| STM32 TX LoRa | ~80mA @ 3.3V (pico breve) |
| STM32 idle | ~5mA @ 3.3V |
| DHT22 | ~1.5mA (breve, desprezível) |
| **Total médio estimado** | **~60–80mA @ 3.7V** |
| **Autonomia estimada** | **~30–40 horas contínuas** |

> ⚠️ Com sensor sempre ligado (sem duty-cycle de energia), a autonomia pode ser insuficiente para múltiplas viagens. **Considerar desligar o MH-Z19E via MOSFET/transistor entre leituras** para economizar bateria (aguardar 1 min de warm-up antes de cada leitura).

# MH-Z19E – CO2 NDIR Sensor (Datasheet Oficial Winsen v1.0)
Fonte: https://www.winsen-sensor.com/sensors/co2-sensor/mh-z19e.html
Manual PDF: https://www.winsen-sensor.com/d/files/mh-z19e-co2-sensor-manual-v1_0.pdf

---

## Especificações Principais (Table 1)

| Parâmetro | Valor |
|-----------|-------|
| Modelo | MH-Z19E |
| Gás detectado | CO2 |
| Tensão de operação | **5.0 ± 0.1 V DC** |
| Corrente média | < 40 mA @ 5V |
| Corrente de pico | 125 mA @ 5V |
| Nível de interface | **3.3V** (compatível com 5V) |
| Saídas disponíveis | **UART (TTL 3.3V)** e PWM |
| Tempo de pré-aquecimento | **1 minuto** (mínimo para começar a responder) |
| Tempo de resposta | T90 < 120s |
| Temperatura de operação | -10 ~ 50 °C |
| Umidade de operação | 0 ~ 95% RH (sem condensação) |
| Temperatura de armazenamento | -20 ~ 60 °C |
| Peso | 5 g |
| Vida útil | > 10 anos |

> **NOTA IMPORTANTE**: O pré-aquecimento indicado no datasheet é **1 minuto**, não 3 minutos como o MH-Z19B. Porém, para leituras estáveis e precisas em aplicação de grãos, recomenda-se aguardar ao menos 3 minutos.

---

## Faixas de Detecção e Precisão (Table 2)

| Gás | Faixa | Resolução | Precisão |
|-----|-------|-----------|----------|
| CO2 | 400 ~ 2000 ppm | 1 ppm | ± (50 ppm + 5% do valor lido) |
| CO2 | **400 ~ 5000 ppm** ← **nosso módulo** | 1 ppm | ± (50 ppm + 5% do valor lido) |
| CO2 | 400 ~ 10000 ppm | 1 ppm | ± (50 ppm + 5% do valor lido) |

**Exemplo de precisão para nosso módulo (400-5000 ppm):**
- A 1000 ppm: ± (50 + 50) = ± 100 ppm
- A 2000 ppm: ± (50 + 100) = ± 150 ppm
- A 5000 ppm: ± (50 + 250) = ± 300 ppm

---

## Pinagem – Terminal Connection Type (conforme imagem do usuário)

| Pino | Função | Detalhe |
|------|--------|---------|
| Pin 4 | Vin | Alimentação positiva **5V** |
| Pin 3 | GND | Terra |
| Pin 2 | Reserved | Deixar desconectado |
| Pin 7 | PWM | Saída PWM – não usado neste projeto |
| Pin 1 | HD | Zero-point calibration: nível LOW por > 7s ativa calibração; deixar flutuando ou HIGH em operação |
| Pin 5 | UART RXD | Entrada de dados TTL → conectar ao **STM32 TX** |
| Pin 6 | UART TXD | Saída de dados TTL → conectar ao **STM32 RX** |

---

## Protocolo UART

- **Baud rate**: 9600 bps
- **Data bits**: 8
- **Stop bits**: 1
- **Paridade**: Nenhuma (8N1)
- **Nível lógico**: 3.3V TTL

### Comando: Ler concentração de CO2
**TX (MCU → Sensor):** `FF 01 86 00 00 00 00 00 79` (9 bytes)

**RX (Sensor → MCU):** `FF 86 HIGH LOW TEMP 00 00 00 CHECKSUM` (9 bytes)
- CO2 (ppm) = `byte[2] × 256 + byte[3]`
- Temperatura interna (raw) = `byte[4] - 40` → temperatura em °C do sensor (não usar para leitura ambiental)

### Verificação de Checksum:
```c
// Checksum dos bytes [1] a [7] (índice 0-based)
uint8_t checksum = 0xFF - (buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]) + 1;
// buf[8] deve ser igual a checksum
```

### Comando: Desativar ABC (Auto-Baseline Correction)
**TX:** `FF 01 79 00 00 00 00 00 86` (9 bytes)
> **CRÍTICO para aplicação de grãos**: O ABC assume que o ambiente atinge 400 ppm periodicamente (ar fresco). Em ambiente fechado (silo, granário), isso nunca acontece, corrompendo a calibração. **DEVE SER DESATIVADO** logo após o boot.

### Comando: Ativar ABC
**TX:** `FF 01 79 A0 00 00 00 00 E6` (9 bytes)

### Comando: Calibração de zero point (span 0)
**TX:** `FF 01 87 00 00 00 00 00 78` (9 bytes)
> Alternativa ao pino HD. Não usar em operação normal.

### Comando: Calibração de span (ex: 2000 ppm)
**TX:** `FF 01 88 07 D0 00 00 00 A0` (9 bytes)
> Span value = 2000 = 0x07D0. Não usar em operação normal.

---

## Integração no Projeto (STM32WLE5 / E77-900M22S)

- **UART**: Usar **USART1** nos pinos **PB6** (TX) e **PB7** (RX) com AF7 (USART2 ocupada com debug/trace, PB10/PB11 não expostos no módulo E77)
- **Pinagem**:
  - STM32 PB6 (USART1_TX) -> MH-Z19E Pin 5 (RXD)
  - STM32 PB7 (USART1_RX) -> MH-Z19E Pin 6 (TXD)
  - Boost 5V -> MH-Z19E Pin 4 (Vin)
  - GND -> MH-Z19E Pin 3 (GND)
- **Alimentação**: 5V externo obrigatório no Pin 4; Pin 5/6 são 3.3V → compatível direto com STM32
- **Cayenne LPP Channel**: Canal 3, tipo `Analog Input` (0x02), valor = ppm / 100.0f
- **Sequência de boot**:
  1. Inicializar USART1 a 9600 baud 8N1
  2. Aguardar 1 minuto (pré-aquecimento mínimo)
  3. Enviar comando de leitura e processar resposta
- **Corrente de pico de 125mA**: Alimentar pelo regulador boost de 5V; não usar o 3.3V do STM32
