#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <DHT.h>
#include <util/atomic.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

const byte PINO_LDR = A0, PINO_NTC = A1;
const byte PINO_BOTAO = 2, PINO_ECHO = 3, PINO_TRIG = 4;
const byte PINO_LED_VERMELHO = 5, PINO_LED_VERDE = 11;
const byte PINO_CHEIA_IR = 7, PINO_SERVO = 8, PINO_DHT = 10, PINO_BUZZER = 12;
const byte ENDERECO_LCD = 0x27;
const byte NIVEL_ATIVO_IR = LOW;
const byte ANGULO_FECHADO = 10, ANGULO_ABERTO = 100;
const unsigned long TEMPO_FECHAMENTO_MS = 3000, PASSO_SERVO_MS = 20;
const unsigned long DEBOUNCE_MS = 40, CONFIRMACAO_CHEIA_MS = 1000;
const unsigned long INTERVALO_ANALOGICO_MS = 100, INTERVALO_DHT_MS = 2000;
const unsigned long INTERVALO_ULTRASSOM_MS = 100, ESTABILIZACAO_MS = 1000;
const unsigned long TIMEOUT_ECHO_US = 30000;
const unsigned long INTERVALO_LCD_MS = 250, INTERVALO_RELATORIO_MS = 1000;
const float DISTANCIA_ABRIR_CM = 20.0f, DISTANCIA_LIBERAR_CM = 25.0f;
const byte AMOSTRAS_PARA_ABRIR = 2, AMOSTRAS_PARA_LIBERAR = 3;
const int LDR_ESCURO_ADC = 50, LDR_CLARO_ADC = 950;
const float RESISTOR_NTC = 10000.0f, NTC_NOMINAL = 10000.0f;
const float NTC_BETA = 3950.0f, NTC_NOMINAL_K = 298.15f;
static_assert(ANGULO_ABERTO > ANGULO_FECHADO && ANGULO_ABERTO <= 180, "Rever angulos da tampa");
static_assert(LDR_CLARO_ADC > LDR_ESCURO_ADC, "Rever calibracao LDR");

LiquidCrystal_I2C lcd(ENDERECO_LCD, 16, 2);
Servo servoTampa;
DHT dht(PINO_DHT, DHT11);
enum EstadoTampa : byte { FECHADA, ABRINDO, ABERTA, FECHANDO };
enum EstadoUltrassom : byte { US_OCIOSO, US_DISPARO, US_ECO };
EstadoTampa estadoTampa = FECHADA;
EstadoUltrassom estadoUS = US_OCIOSO;
bool automatico = true, presenca = false, presencaBruta = false;
bool cheia = false, leituraCheiaValida = false, lcdDisponivel = false;
bool displayPrecisaAtualizar = true, distanciaValida = false;
bool leituraIrBruta = false, candidatoCheia = false, candidatoIrAtivo = false;
byte telaAtual = 1, anguloAtual = ANGULO_FECHADO, anguloAlvo = ANGULO_FECHADO;
byte pwmVermelho = 0, pwmVerde = 0, ciclosSemEco = 0;
byte confirmacoesPresenca = 0, confirmacoesAusencia = 0;
int adcLdr = 0, adcNtc = 0, luminosidade = 0, umidade = -1, distanciaMaoMm = -1;
float temperaturaNtc = NAN;
unsigned long ultimaPresenca = 0, candidatoIrDesde = 0, ultimoPassoServo = 0;
unsigned long fechadaDesde = 0, ultimaAnalogica = 0, ultimoDht = 0;
unsigned long ultimoUltrassom = 0, ultimoLcd = 0, ultimoRelatorio = 0;
unsigned long inicioUS = 0;

volatile bool eventoBotao = false;
volatile bool ecoAtivo = false, ecoSubiu = false, ecoPronto = false;
volatile unsigned long subidaEco = 0, duracaoEco = 0;
// Comandos curtos; resposta dimensionada para o relatorio completo.
char comando[16], transmissao[224];
byte tamanhoComando = 0;
bool descartandoComando = false;
uint16_t tamanhoTx = 0, posicaoTx = 0;
char ultimaLinha[2][17] = {{0}, {0}};

void aoPressionarBotao() { eventoBotao = true; }
void aoMudarEcho() {
    if (!ecoAtivo) return;
    const unsigned long agora = micros();
    if (digitalRead(PINO_ECHO) == HIGH) {
        subidaEco = agora;
        ecoSubiu = true;
    } else if (ecoSubiu) {
        duracaoEco = agora - subidaEco;
        ecoPronto = true;
        ecoAtivo = false;
    }
}

bool leituraIrRetida() {
    return estadoTampa != FECHADA || millis() - fechadaDesde < ESTABILIZACAO_MS;
}

void selecionarTela(byte tela) {
    telaAtual = tela;
    displayPrecisaAtualizar = true;
}

void atualizarBotao(unsigned long agora) {
    static bool ultimaLeitura = HIGH, estavel = HIGH, eventoPendente = false;
    static unsigned long ultimaMudanca = 0;
    bool evento;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { evento = eventoBotao; eventoBotao = false; }
    if (evento) eventoPendente = true;
    const bool leitura = digitalRead(PINO_BOTAO);
    if (leitura != ultimaLeitura) { ultimaMudanca = agora; ultimaLeitura = leitura; }
    if (agora - ultimaMudanca >= DEBOUNCE_MS) {
        if (leitura != estavel) {
            estavel = leitura;
            if (estavel == LOW && eventoPendente) selecionarTela(telaAtual == 1 ? 2 : 1);
            eventoPendente = false;
        } else if (estavel == HIGH) eventoPendente = false;
    }
}

void atualizarCheia(unsigned long agora) {
    leituraIrBruta = digitalRead(PINO_CHEIA_IR) == NIVEL_ATIVO_IR;
    // Ignorar a passagem da mao/lixo durante abertura; conservar ultimo estado.
    if (leituraIrRetida()) { candidatoIrAtivo = false; return; }
    if (!candidatoIrAtivo || candidatoCheia != leituraIrBruta) {
        candidatoCheia = leituraIrBruta;
        candidatoIrDesde = agora;
        candidatoIrAtivo = true;
    }
    if (agora - candidatoIrDesde >= CONFIRMACAO_CHEIA_MS &&
        (!leituraCheiaValida || cheia != candidatoCheia)) {
        cheia = candidatoCheia;
        leituraCheiaValida = true;
        displayPrecisaAtualizar = true;
    }
}

void definirAlvoTampa(bool abrir) {
    const byte alvo = abrir ? ANGULO_ABERTO : ANGULO_FECHADO;
    if (anguloAlvo == alvo) return;
    anguloAlvo = alvo;
    estadoTampa = abrir ? ABRINDO : FECHANDO;
    displayPrecisaAtualizar = true;
    candidatoIrAtivo = false;
}

void atualizarTampa(unsigned long agora) {
    if (presencaBruta || presenca) ultimaPresenca = agora;
    if ((automatico && presenca) || (estadoTampa == FECHANDO && presencaBruta)) {
        definirAlvoTampa(true);
    }
    if (automatico && estadoTampa == ABERTA && !presenca && !presencaBruta &&
        agora - ultimaPresenca >= TEMPO_FECHAMENTO_MS) definirAlvoTampa(false);
    if (anguloAtual == anguloAlvo || agora - ultimoPassoServo < PASSO_SERVO_MS) return;
    ultimoPassoServo = agora;
    if (anguloAtual < anguloAlvo) ++anguloAtual; else --anguloAtual;
    servoTampa.write(anguloAtual);
    if (anguloAtual == anguloAlvo) {
        estadoTampa = anguloAlvo == ANGULO_ABERTO ? ABERTA : FECHADA;
        if (estadoTampa == FECHADA) fechadaDesde = agora;
        else ultimaPresenca = agora; // Garantir tempo minimo aberto.
        displayPrecisaAtualizar = true;
    }
}

float calcularTemperaturaNtc(int adc) {
    if (adc <= 0 || adc >= 1023) return NAN;
    const float resistencia = RESISTOR_NTC * adc / (1023.0f - adc);
    return 1.0f / (1.0f / NTC_NOMINAL_K + log(resistencia / NTC_NOMINAL) / NTC_BETA) - 273.15f;
}

void atualizarAnalogicos(unsigned long agora) {
    if (agora - ultimaAnalogica < INTERVALO_ANALOGICO_MS) return;
    ultimaAnalogica = agora;
    adcLdr = analogRead(PINO_LDR);
    adcNtc = analogRead(PINO_NTC);
    // Indice relativo; nao e uma medida em lux.
    luminosidade = constrain(map(adcLdr, LDR_ESCURO_ADC, LDR_CLARO_ADC, 0, 100), 0, 100);
    // Leituras nos extremos do ADC sao consideradas invalidas.
    temperaturaNtc = adcNtc <= 0 || adcNtc >= 1023 ? NAN : calcularTemperaturaNtc(adcNtc);
    displayPrecisaAtualizar = true;
}

void atualizarDht(unsigned long agora) {
    if (agora - ultimoDht < INTERVALO_DHT_MS) return;
    if (estadoUS != US_OCIOSO) return;

    float leitura = dht.readHumidity();
    if (isnan(leitura)) {
        umidade = -1;
    } else {
        umidade = leitura;
    }

    ultimoDht = millis();
    displayPrecisaAtualizar = true;
}

void registrarAusencia() {
    confirmacoesPresenca = 0;
    if (confirmacoesAusencia < AMOSTRAS_PARA_LIBERAR) ++confirmacoesAusencia;
    if (confirmacoesAusencia >= AMOSTRAS_PARA_LIBERAR) presenca = presencaBruta = false;
}

void registrarSemEco() {
    distanciaValida = false;
    distanciaMaoMm = -1;
    if (ciclosSemEco < 255) ++ciclosSemEco;
    // Sem eco pode ser espaco livre ou sensor desconectado; nao distingue os dois.
    registrarAusencia();
    displayPrecisaAtualizar = true;
}

void registrarDistancia(float distancia, unsigned long agora) {
    if (!isfinite(distancia) || distancia <= 0.0f || distancia > 400.0f) {
        registrarSemEco(); return;
    }
    ciclosSemEco = 0;
    distanciaValida = true;
    distanciaMaoMm = int(distancia * 10.0f + 0.5f);
    // Pulso muito curto (<2cm, fora da precisao nominal) e tratado como perto,
    // nunca como espaco livre. Ajustar posicionamento para evitar zona cega.
    if (distancia <= DISTANCIA_ABRIR_CM) {
        presencaBruta = true; ultimaPresenca = agora;
        confirmacoesAusencia = 0;
        if (confirmacoesPresenca < AMOSTRAS_PARA_ABRIR) ++confirmacoesPresenca;
        if (confirmacoesPresenca >= AMOSTRAS_PARA_ABRIR) presenca = true;
    } else if (distancia >= DISTANCIA_LIBERAR_CM) {
        registrarAusencia();
    } else {
        confirmacoesPresenca = confirmacoesAusencia = 0;
    }
    displayPrecisaAtualizar = true;
}

void atualizarUltrassom(unsigned long agora) {
    // A mao deve ser monitorada tambem enquanto a tampa abre ou fecha.
    if (estadoUS == US_OCIOSO) {
        if (agora - ultimoUltrassom < INTERVALO_ULTRASSOM_MS) return;
        ultimoUltrassom = agora;
        digitalWrite(PINO_TRIG, HIGH); inicioUS = micros(); estadoUS = US_DISPARO;
    } else if (estadoUS == US_DISPARO) {
        if (micros() - inicioUS < 10) return;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ecoPronto = ecoSubiu = false; ecoAtivo = true;
        }
        digitalWrite(PINO_TRIG, LOW); inicioUS = micros(); estadoUS = US_ECO;
    } else {
        bool pronto; unsigned long duracao;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pronto = ecoPronto; duracao = duracaoEco; }
        if (pronto) {
            estadoUS = US_OCIOSO;
            registrarDistancia(duracao * 0.0343f / 2.0f, agora);
        } else if (micros() - inicioUS >= TIMEOUT_ECHO_US) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ecoAtivo = false; }
            estadoUS = US_OCIOSO; registrarSemEco();
        }
    }
}

void atualizarIndicadores(unsigned long agora) {
    const byte brilho = map(luminosidade, 0, 100, 35, 255);
    // Manter o verde visivel mesmo em ambiente escuro.
    const byte brilhoVerde = map(luminosidade, 0, 100, 200, 255);
    pwmVermelho = leituraCheiaValida && cheia ? brilho : 0;
    pwmVerde = leituraCheiaValida && !cheia ? brilhoVerde : 0;
    analogWrite(PINO_LED_VERMELHO, pwmVermelho);
    analogWrite(PINO_LED_VERDE, pwmVerde);
    digitalWrite(PINO_BUZZER, cheia && agora % 2000 < 200 ? HIGH : LOW);
}

const char *nomeTampa() {
    switch (estadoTampa) {
        case FECHADA: return "FECHADA";
        case ABRINDO: return "ABRINDO";
        case ABERTA: return "ABERTA";
        default: return "FECHANDO";
    }
}

void prepararResposta(const char *texto) {
    strncpy(transmissao, texto, sizeof(transmissao) - 1);
    transmissao[sizeof(transmissao) - 1] = '\0';
    tamanhoTx = strlen(transmissao); posicaoTx = 0;
}

void prepararStatus() {
    const int temp10 = isnan(temperaturaNtc) ? -999 : int(round(temperaturaNtc * 10));
    snprintf(transmissao, sizeof(transmissao), "STATUS MODO=%s TAMPA=%s PRES=%u DIST_MAO_MM=%d US_VALIDO=%u SEM_ECO=%u "
        "CHEIA=%d IR_BRUTO=%u IR_RETIDO=%u UMID=%d TEMP_NTC_X10=%d LUZ=%d "
        "ADC_LDR=%d ADC_NTC=%d PWM_R=%u PWM_V=%u LCD=%u TELA=%u\n",
        automatico ? "AUTO" : "MANUAL", nomeTampa(), presencaBruta,
        distanciaMaoMm, distanciaValida, ciclosSemEco,
        leituraCheiaValida ? int(cheia) : -1, leituraIrBruta, leituraIrRetida(),
        umidade, temp10/10, luminosidade, adcLdr, adcNtc, pwmVermelho, pwmVerde,
        lcdDisponivel, telaAtual);
    tamanhoTx = strlen(transmissao); posicaoTx = 0;
}

// strcmp retorna zero quando os dois textos sao iguais.
void executarComando(unsigned long agora) {    
    if (strcmp(comando, "STATUS") == 0) {
        prepararStatus();
    } else if (strcmp(comando, "HELP") == 0) {
        prepararResposta("HELP STATUS AUTO MANUAL ABRIR FECHAR TELA 1 TELA 2\n"
            "ABRIR/FECHAR: modo MANUAL. Envie um comando e aguarde resposta.\n");
    } else if (strcmp(comando, "AUTO") == 0) {
        automatico = true;
        ultimaPresenca = agora;
        prepararResposta("OK AUTO\n");
    } else if (strcmp(comando, "MANUAL") == 0) {
        automatico = false;
        prepararResposta("OK MANUAL\n");
    } else if (strcmp(comando, "TELA 1") == 0) {
        selecionarTela(1);
        prepararResposta("OK TELA\n");
    } else if (strcmp(comando, "TELA 2") == 0) {
        selecionarTela(2);
        prepararResposta("OK TELA\n");
    } else if (strcmp(comando, "ABRIR") == 0) {
        if (automatico) {
            prepararResposta("ERR MODO\n");
        } else {
            definirAlvoTampa(true);
            prepararResposta("OK TAMPA\n");
        }
    } else if (strcmp(comando, "FECHAR") == 0) {
        if (automatico) {
            prepararResposta("ERR MODO\n");
        } else if (presencaBruta || presenca) {
            prepararResposta("ERR PRESENCA\n");
        } else if (estadoTampa == ABRINDO) {
            prepararResposta("ERR OCUPADO\n");
        } else {
            definirAlvoTampa(false);
            prepararResposta("OK TAMPA\n");
        }
    } else {
        prepararResposta("ERR COMANDO\n");
    }

    displayPrecisaAtualizar = true;
}

void processarSerial(unsigned long agora) {
    if (posicaoTx < tamanhoTx) return; // Uma resposta por comando, sem sobrescrever.
    for (byte n = 0; n < 16 && Serial.available(); ++n) {
        const char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (descartandoComando) prepararResposta("ERR LINHA_LONGA_OU_INVALIDA\n");
            else if (tamanhoComando > 0) { comando[tamanhoComando] = '\0'; executarComando(agora); }
            tamanhoComando = 0; descartandoComando = false;
            break;
        }
        if (descartandoComando) continue;
        if (c < 32 || c > 126 || tamanhoComando >= sizeof(comando) - 1) {
            descartandoComando = true; continue;
        }
        comando[tamanhoComando++] = toupper(static_cast<unsigned char>(c));
    }
}

void enviarSerial(unsigned long agora) {
    if (posicaoTx == tamanhoTx && tamanhoComando == 0 && !descartandoComando &&
        !Serial.available() && agora - ultimoRelatorio >= INTERVALO_RELATORIO_MS) {
        ultimoRelatorio = agora; prepararStatus();
    }
    const uint16_t disponivel = static_cast<uint16_t>(Serial.availableForWrite());
    uint16_t quantidade = tamanhoTx - posicaoTx;
    if (quantidade > 16) quantidade = 16;
    if (quantidade > disponivel) quantidade = disponivel;
    if (quantidade) {
        Serial.write(reinterpret_cast<const uint8_t *>(transmissao + posicaoTx), quantidade);
        posicaoTx += quantidade;
    }
}

void escreverLinha(byte linha, const char *texto) {
    char completa[17]; memset(completa, ' ', 16); completa[16] = '\0';
    const byte comprimento = min(strlen(texto), size_t(16));
    memcpy(completa, texto, comprimento);
    if (!strcmp(completa, ultimaLinha[linha])) return;
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(ENDERECO_LCD);
    const byte erro = Wire.endTransmission();
    if (erro || Wire.getWireTimeoutFlag()) { lcdDisponivel = false; return; }
    lcd.setCursor(0, linha);
    if (Wire.getWireTimeoutFlag()) { lcdDisponivel = false; return; }
    for (byte i = 0; i < 16; ++i) {
        lcd.write(completa[i]);
        // Interromper na primeira falha; nao multiplicar timeout por toda a linha.
        if (Wire.getWireTimeoutFlag()) { lcdDisponivel = false; return; }
    }
    strcpy(ultimaLinha[linha], completa);
}

void atualizarDisplay(unsigned long agora) {
    if (!lcdDisponivel || !displayPrecisaAtualizar || agora - ultimoLcd < INTERVALO_LCD_MS) return;
    ultimoLcd = agora; displayPrecisaAtualizar = false;
    char linha[24];
    if (telaAtual == 1) {
        const char *situacao = !leituraCheiaValida ? "N/D" : (cheia ? "SIM" : "NAO");
        snprintf(linha, sizeof(linha), "Cheia:%s%c %s", situacao,
            leituraIrRetida() ? '*' : ' ', automatico ? "AUTO" : "MAN");
        escreverLinha(0, linha);
        snprintf(linha, sizeof(linha), "Tampa: %s", nomeTampa());
    } else {
        if (umidade >= 0) snprintf(linha, sizeof(linha), "Umid:%3d%%UR", umidade);
        else strcpy(linha, "Umid: N/D");
        escreverLinha(0, linha);
        snprintf(linha, sizeof(linha), "Luz:%3d%%", luminosidade);
    }
    if (lcdDisponivel) escreverLinha(1, linha);
}

void setup() {
    pinMode(PINO_BOTAO, INPUT_PULLUP); pinMode(PINO_CHEIA_IR, INPUT_PULLUP);
    pinMode(PINO_ECHO, INPUT); pinMode(PINO_TRIG, OUTPUT);
    pinMode(PINO_LED_VERMELHO, OUTPUT); pinMode(PINO_LED_VERDE, OUTPUT);
    pinMode(PINO_BUZZER, OUTPUT);
    dht.begin();
    digitalWrite(PINO_TRIG, LOW); digitalWrite(PINO_BUZZER, LOW);
    Serial.begin(115200);
    Wire.begin(); Wire.setWireTimeout(3000, true);
    Wire.beginTransmission(ENDERECO_LCD);
    lcdDisponivel = Wire.endTransmission() == 0;
    if (lcdDisponivel) {
        // Inicializacao unica do controlador LCD requer esperas da biblioteca.
        // O servo ainda nao esta ativo; nao ha delay no ciclo de operacao.
        lcd.init(); lcd.backlight();
        if (Wire.getWireTimeoutFlag()) lcdDisponivel = false;
    }
    attachInterrupt(digitalPinToInterrupt(PINO_BOTAO), aoPressionarBotao, FALLING);
    attachInterrupt(digitalPinToInterrupt(PINO_ECHO), aoMudarEcho, CHANGE);
    const unsigned long agora = millis();
    // Iniciar aberta para nao fechar antes das primeiras leituras da mao.
    // Posicao fisica inicial nao e conhecida; testar primeiro sem acoplar a tampa.
    anguloAtual = anguloAlvo = ANGULO_ABERTO;
    estadoTampa = ABERTA;
    // Gravar alvo antes de attach evita um pulso inicial no angulo padrao 90.
    servoTampa.write(anguloAtual); servoTampa.attach(PINO_SERVO);
    ultimaPresenca = fechadaDesde = ultimoDht = agora;
    ultimoUltrassom = agora - INTERVALO_ULTRASSOM_MS;
    ultimaAnalogica = agora - INTERVALO_ANALOGICO_MS;
    prepararResposta("LIXEIRA KY032 + HC-SR04. HELP lista comandos. * = IR retido.\n");
}

void loop() {
    unsigned long agora = millis();
    // Pulso TRIG tem prioridade por apenas alguns microssegundos; sem espera ativa.
    if (estadoUS == US_DISPARO) { atualizarUltrassom(agora); return; }
    atualizarBotao(agora);
    atualizarDht(agora);
    // A leitura DHT leva alguns milissegundos: atualizar a referencia de tempo.
    agora = millis();
    atualizarUltrassom(agora);
    if (estadoUS == US_DISPARO) return;
    atualizarTampa(agora);
    atualizarCheia(agora);
    processarSerial(agora);
    atualizarIndicadores(agora);
    atualizarAnalogicos(agora);
    atualizarDisplay(agora);
    enviarSerial(agora);
}
