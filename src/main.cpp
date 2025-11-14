/*
  RadioLib LoRaWAN ABP Example

  ABP = Activation by Personalisation, an alternative
  to OTAA (Over the Air Activation). OTAA is preferable.

  This example will send uplink packets to a LoRaWAN network. 
  Before you start, you will have to register your device at 
  https://www.thethingsnetwork.org/
  After your device is registered, you can run this example.
  The device will join the network and start uploading data.

  LoRaWAN v1.0.4/v1.1 requires the use of persistent storage.
  As this example does not use persistent storage, running this 
  examples REQUIRES you to check "Resets frame counters"
  on your LoRaWAN dashboard. Refer to the notes or the 
  network's documentation on how to do this.
  To comply with LoRaWAN's persistent storage, refer to
  https://github.com/radiolib-org/radiolib-persistence

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/

  For LoRaWAN details, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/LoRaWAN

*/


#include "configABP.h"
#include <arduinoFFT.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <U8g2lib.h>
#define AMOSTRAS (860) //amostras a serem coletadas (amostras mais precisas, mas com maior tempo de execução)
#define AMAXSENS 100       // A corrente máxima do sensor neste caso é o SCT013, que oferece 30 A máx. a 1000 mV.
#define MVMAXSENS 512    // MV máximo oferecido pelo sensor em sua corrente máxima suportada
#define VOLTRED 220       // Tensão da rede

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

float multiplier = 0.010162022;
float Peficaz;
float Int_calculada;

uint32_t counter = 0;

float med_Ieficaz();
float refeito_med_Ieficaz(float *frequency);
float hugo_med_Ieficaz();

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 15
#define OLED_RST_PIN 16

#define ADS_SDA 21
#define ADS_SCL 22

// --- Velocidade I2C Segura ---
// Vamos travar em 100kHz para garantir compatibilidade
#define I2C_BUS_SPEED 100000

//TwoWire Wire1 = TwoWire(1); // Cria o segundo barramento I2C

// Crie o objeto u8g2
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* rst=*/ OLED_RST_PIN);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ I2C_SCL_PIN, /* data=*/ I2C_SDA_PIN, /* reset=*/ OLED_RST_PIN);

#define USE_SERIAL 1

// Variável global para o offset. Agora é FLOAT.
float ADC_OFFSET_ZERO = 0.0; // Palpite inicial para modo diferencial
float filtered_I_last = 0.0; // Armazena a última amostra filtrada

const uint16_t N_AMOSTRAS_REAIS = 860; // Suas amostras
const uint16_t N_AMOSTRAS_FFT = 1024;  // Próxima potência de 2 (para o padding)

// Arrays para a FFT (precisam ser globais ou static)
double vReal[N_AMOSTRAS_FFT];
double vImag[N_AMOSTRAS_FFT];

// Instância da FFT
// (TAXA_SPS não é mais 860. A taxa "efetiva" da FFT mudou)
// Taxa_SPS_FFT = TAXA_SPS_ADC * (N_AMOSTRAS_FFT / N_AMOSTRAS_REAIS)
// Taxa_SPS_FFT = 860 * (1024 / 860) = 1024 SPS
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, N_AMOSTRAS_FFT, 860);

void setup() { 
  SetupBoard();
  SerialInit();
  RadioBeginSPI();

  // --- 1. Acorde o Display (Exatamente como o Scanner fez) ---
  // Isso é essencial ANTES de iniciar o I2C
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, LOW);
  delay(20);
  digitalWrite(OLED_RST_PIN, HIGH);
  delay(20);
  Serial.println("Display (pino 16) acordado.");

  // --- 2. Inicie o I2C (Barramento 'Wire') ---
  // Especifique os pinos e TRAVE a velocidade
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_BUS_SPEED);
  u8g2.setBusClock(I2C_BUS_SPEED);
  u8g2.begin();
  /*
  // --- INICIALIZE O DISPLAY AQUI ---
  u8g2.begin();
  u8g2.enableUTF8Print(); // Habilita o print de caracteres UTF-8
  u8g2.setFont(u8g2_font_ncenB08_tr); // Define uma fonte
  u8g2.clearBuffer();
  u8g2.setCursor(0, 10);
  u8g2.print("Display OK!");
  u8g2.sendBuffer(); // Envia o buffer para a tela
  delay(1000);
  // ----------------------------------
  */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.setGain(GAIN_TWO);         // 2x gain   +/- 2.048V  1 bit =     0.0625mV
  Wire1.begin(ADS_SDA, ADS_SCL, 100000); // 100kHz
  ads.begin(0x48, &Wire1);
  ads.setDataRate(RATE_ADS1115_860SPS);
  delay(5000);  // Give time to switch to the serial monitor
  serial.println("\nSetup ... ");
  serial.println("Initialise the radio");
  int state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);
  serial.println(F("Initialise LoRaWAN Network credentials"));
  node.setDutyCycle(false);
  node.setDwellTime(false);
  node.beginABP(devAddr, NULL, NULL, nwkSEncKey, appSKey);
  node.activateABP();
  debug(state != RADIOLIB_ERR_NONE, F("Activate ABP failed"), state, true);
  serial.println(F("Ready!\n"));
  serial.printf("0x%x\n\r", node.getDevAddr());

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13_tf);
  u8g2.drawStr((64-strlen("Display OK!"))/2, 40, "Display OK!");
  u8g2.sendBuffer();
  //delay(500);
  delay(500);
  u8g2.clearBuffer();
}

float v_burden[AMOSTRAS] = {};

float maior_f = -MAXFLOAT;
float menor_f = MAXFLOAT;

void loop() { 
  serial.printf("\nSending uplink\n\r");

  node.setDatarate(DR_SF7);

  /*
  if(counter < 1000) {
    node.setDatarate(DR_SF7);
    serial.printf("Set SF for SF7\n\r");
  } else if ( counter < 2000) {
    node.setDatarate(DR_SF8);
    serial.printf("Set SF for SF9\n\r");
  } else if (counter < 3000) {
    node.setDatarate(DR_SF9);
    serial.printf("Set SF for SF12\n\r");
  } else if (counter < 4000) {
    node.setDatarate(DR_SF10);
    serial.printf("Set SF for SF12\n\r");
  } else if (counter < 5000) {
    node.setDatarate(DR_SF11);
    serial.printf("Set SF for SF12\n\r");
  } else if (counter < 6000) {
    node.setDatarate(DR_SF12);
    serial.printf("Set SF for SF12\n\r");
  } 
  else {
    node.setDatarate(DR_SF7);
    serial.printf("Set SF for SF7\n\r");
    counter = 0;
  }
  */

  float frequency;
  Int_calculada = refeito_med_Ieficaz(&frequency);
  Serial.println(Int_calculada);
  Int_calculada = Int_calculada; // é multiplicado por um valor de correção baseado em medições reais
  Peficaz = Int_calculada * VOLTRED;  //P=V*I

  Serial.printf("%.6f", Int_calculada);
  Serial.println(" A");
  Serial.print(Peficaz);
  Serial.println(" W");

  if (frequency > maior_f) {
    maior_f = frequency;
  }

  if (frequency < menor_f) {
    menor_f = frequency;
  }

  if(frequency < 59) {
    while(1){
      Serial.printf("error frequency: %.6f\n", frequency);
    }
  }

  Serial.printf("M: %.6f > Avg: %.6f > M: %.6f\n", maior_f, frequency, menor_f);


  //u8g2.clearBuffer();
  char buffer[65];

  u8g2.clearBuffer();

  size_t buffer_size = 0;
  sprintf(buffer+buffer_size, "F: %.6f Hz", frequency);
  u8g2.drawStr(0, 16, buffer+buffer_size);
  
  buffer_size = strlen(buffer);
  sprintf(buffer+buffer_size, "I: %.6f A", Int_calculada);
  u8g2.drawStr(0, 32, buffer+buffer_size);

  buffer_size = strlen(buffer);
  sprintf(buffer+buffer_size, "P: %.6f W", Peficaz);
  u8g2.drawStr(0, 48, buffer+buffer_size);

  buffer_size = strlen(buffer);
  sprintf(buffer+buffer_size, "V: %.6f V", Int_calculada+300);
  u8g2.drawStr(0, 64, buffer+buffer_size);
  Serial.printf("4size: %ld\nsize+buffer: %ld\n", strlen(buffer), strlen(buffer+buffer_size));
  printf("I = %.6f\nP = %.6f\nP*0,7 = %.6f\nP*0,9 = %.6f\n", Int_calculada, Peficaz, Peficaz*0.7f, Peficaz*0.9f);

  u8g2.sendBuffer(); 
  //delay(500);

  /*
  for(int i = 0; i< N_AMOSTRAS_FFT;i++){
    Serial.printf("%lf ", vReal[i]/860);
  }
  Serial.println("\n fim vReal");
  */

  // Build payload byte array
  uint8_t uplinkPayload[242];
  char message[100];
  //Int_calculada = 0.000121;
  //float valor = 2.32;
  snprintf(message, sizeof(message), "c|%f", Int_calculada);
  //snprintf(message + strlen(message), sizeof(message), "%f", valor);
  //printf("message: %s\nsizeof: %ld\n", message, strlen(message));
  serial.println(message);
  strcpy((char*) uplinkPayload, message);
  // Reseta contador do pluviômetro
  // Perform an uplink
  /*
  int state = node.sendReceive(uplinkPayload, strlen((char*) uplinkPayload));
  debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);
  counter++;
  // Check if a downlink was received 
  // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
  if(state > 0) {
    serial.println(F("Received a downlink"));
  } else {
    serial.println("No downlink received");
  }
  */
  // Wait until next uplink - observing legal & TTN FUP constraints
  delay(uplinkIntervalSeconds * 0);
}


float med_Ieficaz() {                         
  int16_t bitsads;
  float mVporbit = 0.0625F;
  float Ieficaz;
  float Iinstant;
  float mVinstant;
  float sumIinstant=0;

  long sample_interval_us = (1000*1000) / 860; // Aprox. 1163 µs
  long next_sample_time = micros();
  
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
  
  float tempoinicio = millis();                       // para medir quanto tempo leva para realizar as medições
  for (int i = 0; i < AMOSTRAS; i++) {
    //bitsads = ads.readADC_Differential_0_1();
    // 2. Espere até o momento da próxima amostra
    while (micros() < next_sample_time) {
      // Loop de espera ativa.
      // Em um sistema mais avançado, o MCU poderia dormir aqui.
    }
    // Define o tempo para a *próxima* amostra
    next_sample_time += sample_interval_us;

    bitsads = ads.getLastConversionResults();

    mVinstant = bitsads * mVporbit;
    Iinstant = mVinstant * AMAXSENS / MVMAXSENS;   // regra de três baseada no sensor conectado já que o sensor oferece tensão e a passamos diretamente proporcional à intensidade
    sumIinstant += sq(Iinstant);                   // soma dos quadrados
  }
  float tempofim = millis();

  Ieficaz = sqrt(sumIinstant / AMOSTRAS);        // raiz quadrada da soma dos quadrados dividida pelo número de amostras

  Serial.print((tempofim - tempoinicio) / 1000.0);
  Serial.println(" segundos para medir");
  return (Ieficaz);
}

float refeito_med_Ieficaz(float* frequency) {                         
  int16_t bitsads;
  float mVporbit = 0.0625F;
  float Ieficaz;
  float Iinstant;
  float mVinstant;
  float sumIinstant=0;
  
  // cruzamento
  int16_t valorAnterior = 0;
  /*
  bool primeiraCruzada = true;
  double tZeroAnterior = 0, tZeroAtual = 0;
  */
  // --- controle de zero crossings ---
  const int N_CICLOS = 10; // quantos ciclos usar na média
  /* media simples*/
  int ciclosDetectados = 0;
  double tZeroInicial = 0, tZeroFinal = 0;
  bool primeiraCruzada = true;

  /* media movel
  */
  double periodos_us[N_CICLOS];
  int idxPeriodo = 0;
  bool bufferCheio = false;

  
  float freq = 0.0;

  // Constante do filtro passa-baixa (LPF). 
  // 0.1 = filtro forte. 0.5 = filtro médio. 0.9 = filtro fraco.
  // Comece com um valor médio e ajuste
  const float ALPHA_LPF = 0.7;
  
  // Constante do filtro (alpha). 
  // Um valor menor = filtro mais lento e estável (bom).
  const float ALPHA_FILTRO_OFFSET = 0.001;

  long sample_interval_us = (1000*1000) / 860; // Aprox. 1163 µs
  long next_sample_time = micros();
  
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
  float tempoinicio = millis();                       // para medir quanto tempo leva para realizar as medições

  for (int i = 0; i < AMOSTRAS; i++) {
    //bitsads = ads.readADC_Differential_0_1();
    // 2. Espere até o momento da próxima amostra
    while (micros() < next_sample_time) {
      // Loop de espera ativa.
      // Em um sistema mais avançado, o MCU poderia dormir aqui.
    }
    unsigned long t2 = micros();
    // Define o tempo para a *próxima* amostra
    next_sample_time += sample_interval_us;

    bitsads = ads.getLastConversionResults();

    // --- DETECÇÃO DE CRUZAMENTO DE ZERO ---
    // --- Detecção de zero cross ---
    if ((valorAnterior < 0 && bitsads >= 0)) {
      float y1 = valorAnterior;
      float y2 = bitsads;
      unsigned long t1 = t2 - sample_interval_us;

      double frac = (0 - y1) / (y2 - y1);
      double tZero = t1 + frac * (t2 - t1);

      //Serial.printf("[t1: %lu]\n", t1);

      /*
      if (!primeiraCruzada) {
        double periodo_us = tZero - tZeroAnterior;
        freq = 1e6 / periodo_us;
      } else {
        primeiraCruzada = false;
      }

      tZeroAnterior = tZero;
      */

      /* media simples
      if (primeiraCruzada) {
        tZeroInicial = tZero;
        primeiraCruzada = false;
      } else {
        ciclosDetectados++;
        tZeroFinal = tZero;

        // Quando atingir N_CICLOS, calcula a média
        if (ciclosDetectados >= N_CICLOS) {
          double periodoMedio_us = (tZeroFinal - tZeroInicial) / N_CICLOS;
          freq = 1e6 / periodoMedio_us;
          
          // Reinicia contagem para estabilizar continuamente
          ciclosDetectados = 0;
          tZeroInicial = tZeroFinal;
        }
      }
      */
      /* media movel
      */
      if (primeiraCruzada) {
        tZeroInicial = tZero;
        primeiraCruzada = false;
      } else {
        double periodo_us = tZero - tZeroInicial;
        tZeroInicial = tZero;

        // Armazena no buffer circular
        periodos_us[idxPeriodo] = periodo_us;
        idxPeriodo = (idxPeriodo + 1) % N_CICLOS;
        if (idxPeriodo == 0) bufferCheio = true;

        // Calcula média móvel
        double soma = 0;
        int n = bufferCheio ? N_CICLOS : idxPeriodo;
        for (int j = 0; j < n; j++) soma += periodos_us[j];
        double periodoMedio_us = soma / n;

        freq = 1e6 / periodoMedio_us;
      }
    }

    valorAnterior = bitsads;
    // --- DETECÇÃO DE CRUZAMENTO DE ZERO ---
    
    // 2. ATUALIZA O OFFSET (A MÁGICA)
    // O offset atual é 99.9% do valor antigo + 0.1% do novo valor
    ADC_OFFSET_ZERO = (ADC_OFFSET_ZERO * (1.0 - ALPHA_FILTRO_OFFSET)) + (bitsads * ALPHA_FILTRO_OFFSET);
    //Serial.printf("o: %.6f ", ADC_OFFSET_ZERO);

    mVinstant = (bitsads - ADC_OFFSET_ZERO) * (mVporbit/1000);
    Iinstant = mVinstant/200;   // regra de três baseada no sensor conectado já que o sensor oferece tensão e a passamos diretamente proporcional à intensidade
    Iinstant *= 2000; 

    // --- Etapa 3: Aplicar o LPF (Remover Ruído de HF) ---
    float I_filtrado = (Iinstant * ALPHA_LPF) + (filtered_I_last * (1.0 - ALPHA_LPF));
    filtered_I_last = I_filtrado; // Salva o valor atual para a próxima iteração

    //v_burden[i] = Iinstant;
    vReal[i] = I_filtrado; // Armazena o valor em Amperes no array da FFT
    vImag[i] = 0;        // Zera o array imaginário
    sumIinstant += sq(I_filtrado);                   // soma dos quadrados
  }
  
  Ieficaz = sqrt(sumIinstant / AMOSTRAS);        // raiz quadrada da soma dos quadrados dividida pelo número de amostras

  // 1. Preenche o resto do array da FFT com zeros (Padding)
  for (int i = N_AMOSTRAS_REAIS; i < N_AMOSTRAS_FFT; i++) {
    vReal[i] = 0;
    vImag[i] = 0;
  }
  
  // 2. Executa a FFT no array de 1024 amostras
  FFT.compute(FFT_FORWARD); 
  FFT.complexToMagnitude();

  float tempofim = millis();

  Serial.print((tempofim - tempoinicio) / 1000.0);
  Serial.println(" segundos para medir");

  // Se houve pelo menos dois cruzamentos
  
  *frequency = freq;

  return (Ieficaz);
}
