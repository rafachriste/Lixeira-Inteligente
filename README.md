# Lixeira Automática com Arduino Uno

Projeto acadêmico de um sistema embarcado para abrir uma lixeira sem contato e indicar quando ela está cheia. O sistema também monitora luminosidade, temperatura e umidade, mostra informações em um LCD e recebe comandos pelo Monitor Serial.

## Funcionamento

O sensor ultrassônico detecta a aproximação da mão e o servo movimenta a tampa. O sensor infravermelho fica voltado para o interior da lixeira, na altura escolhida como limite de enchimento.

No modo automático:

- A tampa abre após duas leituras consecutivas de distância de até 20 cm.
- Enquanto houver presença detectada, a tampa permanece aberta.
- O afastamento é confirmado após três leituras de 25 cm ou mais.
- Após o afastamento, a tampa aguarda 3 segundos antes de começar a fechar.
- Se detectar aproximação durante o fechamento, a tampa reabre.

Leituras entre 20 e 25 cm preservam a condição anterior para evitar mudanças repetidas perto do limite. Após três ciclos sem eco, o programa também libera a condição de presença. A ausência de eco pode representar espaço livre ou falha na medição.

Para confirmar que a lixeira está cheia:

1. A tampa deve estar fechada.
2. O sistema aguarda 1 segundo para estabilização.
3. O infravermelho deve detectar um obstáculo por 1 segundo contínuo.

Quando confirma o enchimento, o LED vermelho acende e o buzzer emite um bipe de 200 ms a cada 2 segundos. Quando confirma que não está cheia, o LED verde acende.

Durante a abertura, o fechamento e o período de estabilização, o sistema mantém o último estado de enchimento confirmado.

## Componentes

- Arduino Uno R3
- Sensor ultrassônico HC-SR04
- Sensor infravermelho de obstáculos KY-032
- Microservo SG90
- Sensor de umidade DHT11
- Termistor NTC de 10 kΩ
- LDR
- LCD 16×2 com interface I2C
- Botão
- LEDs vermelho e verde
- Buzzer ativo
- Resistores, protoboard e jumpers

## Pinagem

| Componente | Conexão no Arduino |
|---|---|
| LDR — ponto central do divisor de tensão | A0 |
| NTC — ponto central do divisor de tensão | A1 |
| Botão | D2 |
| HC-SR04 — ECHO | D3 |
| HC-SR04 — TRIG | D4 |
| LED vermelho | D5 |
| KY-032 — OUT | D7 |
| Servo — sinal | D8 |
| DHT11 — dados | D10 |
| LED verde | D11 |
| Buzzer ativo — controle | D12 |
| LCD — SDA | A4 |
| LCD — SCL | A5 |

O botão utiliza `INPUT_PULLUP` e deve ser conectado entre D2 e GND. Cada LED precisa de um resistor em série.

O KY-032 utiliza saída ativa em nível baixo. Com o jumper de habilitação instalado, o pino EN externo fica sem conexão.

Os componentes devem compartilhar o GND. Para o servo, recomenda-se alimentação regulada adequada à sua corrente, com GND conectado ao do Arduino.

### Entradas analógicas

O código considera estes divisores de tensão:

- **LDR:** 5V → LDR → ponto de leitura A0 → resistor → GND.
- **NTC:** 5V → resistor fixo → ponto de leitura A1 → NTC → GND.

O cálculo de temperatura está configurado para resistor fixo de 10 kΩ, NTC de 10 kΩ a 25 °C e coeficiente Beta de 3950. Esses parâmetros precisam corresponder aos componentes utilizados.

A luminosidade é um índice relativo de 0 a 100, não uma medida em lux. As referências configuradas são 50 para escuro e 950 para claro na leitura analógica.

## Display e botão

O LCD usa o endereço I2C `0x27` e possui duas telas:

1. Estado de enchimento, modo de operação e estado da tampa.
2. Umidade relativa do ar e luminosidade.

O botão alterna entre as telas. Também é possível selecioná-las pela serial.

`N/D` indica que ainda não existe uma leitura válida. Um asterisco junto ao estado de enchimento indica que o resultado está sendo mantido enquanto a leitura do infravermelho aguarda liberação.

A temperatura do NTC é informada pelo relatório serial.

## Modos de operação

### Automático

A aproximação e o afastamento da mão controlam a abertura e o fechamento.

### Manual

A abertura e o fechamento são solicitados por comandos seriais. O programa mantém verificações de presença para evitar comandar o fechamento sobre um objeto detectado.

## Comunicação serial

Configure o Monitor Serial para:

- **Velocidade:** 115200 baud
- **Final de linha:** Nova linha ou CR e LF

| Comando | Função |
|---|---|
| `HELP` | Lista os comandos disponíveis |
| `STATUS` | Solicita o relatório do sistema |
| `AUTO` | Ativa o modo automático |
| `MANUAL` | Ativa o modo manual |
| `ABRIR` | Solicita abertura no modo manual |
| `FECHAR` | Solicita fechamento no modo manual |
| `TELA1` | Seleciona a primeira tela |
| `TELA2` | Seleciona a segunda tela |

Os comandos são convertidos para maiúsculas, portanto `status` e `STATUS` são equivalentes.

O sistema também envia relatórios periódicos com leituras dos sensores, estados e valores de PWM.

No relatório, `TEMP_NTC_X10` representa a temperatura multiplicada por dez: por exemplo, `253` corresponde a 25,3 °C. O valor `-999` indica leitura inválida.

## Organização do código

O programa está dividido em funções para:

- Ler os sensores.
- Controlar a tampa.
- Confirmar o estado de enchimento.
- Atualizar LEDs e buzzer.
- Tratar o botão e alternar as telas.
- Receber comandos e enviar respostas pela serial.
- Atualizar o LCD.

As interrupções registram o acionamento do botão e as mudanças do sinal ECHO do ultrassônico.

O controle de tempo das tarefas utiliza `millis()` e `micros()`. A leitura do ultrassônico é dividida em etapas para evitar espera bloqueante pelo eco.

A biblioteca do DHT11 utiliza pequenas esperas internas e desabilita interrupções durante parte da leitura. Portanto, o programa não é integralmente não bloqueante.

## Controle de brilho

Os dois LEDs utilizam PWM, com brilho ajustado pela luminosidade:

- Vermelho: de 35 a 255 quando aceso.
- Verde: de 200 a 255 quando aceso.

O verde utiliza um mínimo maior para melhorar sua visibilidade na montagem.

## Instalação

1. Instale a Arduino IDE.
2. Instale as bibliotecas:
   - Servo
   - LiquidCrystal_I2C
   - DHT sensor library, da Adafruit
   - Adafruit Unified Sensor
3. Abra `LixeiraAutomatica/LixeiraAutomatica.ino`.
4. Selecione a placa Arduino Uno e a porta USB correspondente.
5. Confira a montagem e os parâmetros dos sensores.
6. Envie o programa.
7. Abra o Monitor Serial em 115200 baud.

Ao iniciar, o programa comanda a posição aberta da tampa antes de começar o funcionamento automático.

## Requisitos técnicos contemplados

| Requisito | Implementação |
|---|---|
| Arduino Uno | Plataforma do projeto |
| Código organizado em funções | Rotinas separadas por responsabilidade |
| Interrupções | Botão e captura do ECHO |
| Pelo menos 5 I/O digitais | Sensores, botão, servo, LEDs e buzzer |
| Pelo menos 2 entradas analógicas | LDR em A0 e NTC em A1 |
| Pelo menos 2 saídas PWM | LEDs em D5 e D11 |
| Serial bidirecional | Recebimento de comandos e envio de relatórios |
| Dispositivo I2C | LCD 16×2 |
| Display com duas telas | Seleção pelo botão ou pela serial |

**Disciplina:** Sistemas Embarcados e Prototipação 
**Instituição:** Escola Politécnica de Pernambuco
