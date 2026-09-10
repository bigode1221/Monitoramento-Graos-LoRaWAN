# Monitoramento Inteligente da Qualidade de Grãos no Transporte Rodoviário

Sistema embarcado em formato de sonda IoT projetado para monitorar a massa de grãos (milho) durante o transporte rodoviário pós-colheita em carrocerias de caminhões.

O objetivo do projeto é identificar a deterioração biológica da carga antes que ela chegue ao destino (silos e armazéns), medindo o microclima intergranular e os biomarcadores respiratórios dos grãos para evitar a Perda de Massa Seca (DML - Dry Matter Loss).

---

## O Problema

Durante viagens de média e longa distância em carrocerias de caminhão, o milho sofre com variações térmicas externas e retenção de umidade. Essas condições aceleram a respiração celular do grão e estimulam a proliferação de fungos. 

Esse processo consome a massa seca do milho de forma invisível. Quando a carga chega ao armazém, o dano muitas vezes já está consolidado. Esta sonda foi feita para diagnosticar esse estresse em trânsito e subsidiar decisões rápidas de aeração ou secagem imediata no descarregamento.

---

## Estrutura Mecânica da Sonda

O sistema é montado dentro de um tubo de PVC de 150 cm de comprimento, pensado para ser cravado diretamente no milho na carroceria:

* Diametro e furação: Furos de 4 a 5 mm de diâmetro (menores que os grãos de milho para evitar entupimento), espaçados entre 15 e 20 mm em padrão zigue-zague para preservar a rigidez do tubo.
* Zonas de amostragem: Três pontos específicos de leitura:
  * Topo: sensor de temperatura e umidade.
  * Centro (faixa de 10 a 15 cm): sensor de CO2.
  * Base: sensor de temperatura e umidade.
* Bordas de segurança: De 15 a 20 cm em ambas as extremidades sem furos. A ponta superior protege a eletrônica e a bateria; a ponta inferior garante resistência mecânica na penetração da massa de grãos e contra as vibrações da viagem.
* Filtros contra poeira: Malha de nylon/PTFE interna nos furos da região central para impedir que o pó fino do milho atinja e suje a câmara óptica do sensor de CO2.

---

## Hardware e Eletrônica

* Microcontrolador / Rádio LoRa: Módulo Ebyte E77-900M22S baseado no SoC STM32WLE5CCU6 (ARM Cortex-M4 + rádio LoRa integrado).
* Sensores de Temperatura e Umidade: 2x DHT22 (um no topo e outro na base do tubo).
* Sensor de CO2: MH-Z19E NDIR (faixa de 400 a 5000 ppm), posicionado no meio da sonda.
* Alimentação: Bateria Li-Ion 18650 (3.7V nominal).
  * Linha de 3.3V via LDO para o STM32 e sensores DHT22.
  * Linha de 5.0V via conversor Step-Up (Boost) dedicada ao MH-Z19E.
* Chaveamento de carga (em validação): Chaveamento via transistor/MOSFET para alimentar o sensor de CO2 somente durante as janelas de leitura, reduzindo o consumo em viagens longas.

### Mapeamento de Pinos no STM32WLE5

| Sinal / Periférico | Pino STM32 | Conexão Externa |
|---|---|---|
| DHT22 Superior (Dados) | PA0 | Linha de dados do DHT22 do topo |
| DHT22 Inferior (Dados) | PA1 | Linha de dados do DHT22 da base |
| USART1 TX | PB6 | Pino RXD (Pino 5) do MH-Z19E |
| USART1 RX | PB7 | Pino TXD (Pino 6) do MH-Z19E |
| LED Status TX | PB4 | Indicador visual de transmissão |
| Medição de Bateria | ADC interno | Monitoramento de nível da 18650 |

---

## Comunicação e Rede LoRaWAN

* Região: AU915 (padrão Brasil).
* Sub-banda: FSB2 (canais de uplink 8 a 15 e canal 65).
* Ativação: OTAA (Over-The-Air Activation).
* Taxa de Dados: Travado em DR_2 (SF10 / 125 kHz) com ADR desativado para garantir entrega em trajetos rodoviários.
* Intervalo de Envio: 30 minutos em operação normal (primeiro envio 5 segundos após confirmação do Join).
* Formato do Payload: Cayenne LPP (22 bytes por pacote):
  * Canal 1: Temperatura e Umidade (DHT22 Topo)
  * Canal 2: Temperatura e Umidade (DHT22 Base)
  * Canal 3: Analog Input com valor de CO2 em ppm dividido por 100 (ex: 15.97 representa 1597 ppm)
  * Canal 5: Analog Input com nível normalizado da bateria

---

## Parâmetros Biofísicos de Referência

A interpretação agronômica dos dados coletados segue critérios estabelecidos na conservação pós-colheita:

### Umidade
* Umidade do grão: Faixa segura entre 13% e 14%. Acima de 15%, o risco de fungos se eleva rapidamente.
* Umidade Relativa Intergranular (UR): Deve permanecer abaixo de 65% para que o Teor de Umidade de Equilíbrio (EMC) se mantenha na zona estável.

### Temperatura
* Zona de segurança: Abaixo de 15 °C (dormência metabólica).
* Zona de risco: Entre 25 °C e 30 °C ocorre o pico de respiração destrutiva. Na regra prática agronômica, a cada aumento de 5,5 °C na massa de grãos, o tempo seguro de armazenagem cai pela metade.

### Dióxido de Carbono (CO2)
O CO2 é o biomarcador mais rápido de deterioração:
* 400 a 500 ppm: Nível basal atmosférico normal. Carga estável e saudável.
* 600 a 1000 ppm: Ponto de atenção. Início de respiração celular acelerada, estresse térmico ou foco inicial de fungos.
* 1200 a 1500+ ppm: Deterioração crítica. Confirmação de atividade microbiana ativa e perda irreversível de matéria seca. Exige direcionamento imediato da carga para secagem e aeração forçada no armazém.

---

## Próximos Passos: Modelagem e Machine Learning

A próxima etapa do projeto consiste no desenvolvimento dos modelos preditivos alimentados pelas telemetrias coletadas:

1. Cálculo em software do Teor de Umidade de Equilíbrio (EMC) e da taxa de Perda de Massa Seca acumulada (DML) ao longo da rota.
2. Treinamento de modelos de Machine Learning correlacionando o histórico de tempo, variações térmicas e curva de CO2 da viagem com a qualidade do grão na entrega.

---

## Fontes e Referências Científicas

Os parâmetros agronômicos, biomarcadores de respiração celular e a metodologia de predição utilizados nesta sonda baseiam-se na literatura científica e normas técnicas:

* Rodrigues, D. M., Coradi, P. C., Teodoro, L. P. R., Teodoro, P. E., Moraes, R. S., & Leal, M. M. (2024). *Monitoring and predicting corn grain quality on the transport and post-harvest operations in storage units using sensors and machine learning models*. **Scientific Reports (Nature)**, 14(1), 6157.  
  Link: [https://www.nature.com/articles/s41598-024-56879-5](https://www.nature.com/articles/s41598-024-56879-5)  
  DOI: `10.1038/s41598-024-56879-5`

* ASABE Standards (2012). *ASAE D245.7: Moisture Relationships of Plant-based Agricultural Products*. American Society of Agricultural and Biological Engineers, St. Joseph, MI. (Referência para as equações de equilíbrio higroscópico - EMC de milho).

* Embrapa Milho e Sorgo. *Manejo e Conservação de Grãos de Milho Armazenados*. Empresa Brasileira de Pesquisa Agropecuária.  
  Link: [https://www.embrapa.br/milho-e-sorgo](https://www.embrapa.br/milho-e-sorgo)

* Bern, C. J., Steele, J. L., & Kumar, A. (2002). *Carbon dioxide evolution as an indicator of grain deterioration*. Applied Engineering in Agriculture, 18(4), 485-489. (Fundamentação do CO2 como biomarcador precoce de respiração celular e perda de massa seca).

---

## Como Compilar o Firmware

1. Abra o projeto no STM32CubeIDE.
2. Verifique as credenciais LoRaWAN (DevEUI, JoinEUI, AppKey) no arquivo `LoRaWAN/App/se-identity.h`.
3. Ajuste os tempos de telemetria se necessário em `LoRaWAN/App/app_config.h`.
4. Compile para o target STM32WLE5CCUx e grave utilizando ST-Link via SWD.
