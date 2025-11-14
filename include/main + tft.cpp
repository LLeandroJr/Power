#include "arduinoFFT.h" // Você precisará desta biblioteca
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// --- SEU HARDWARE ---
// (Defina suas constantes R_BURDEN_OHMS, TURNS_RATIO, GAIN_RANGE_V, etc. aqui)
// (Defina seu CALIBRATION_FACTOR aqui)
const float NOISE_THRESHOLD_AMPS = 1.5; // (Ajuste seu limiar de ruído)

// --- NOVA ESTRATÉGIA DE AMOSTRAGEM ---
const int TAXA_SPS = 512;
const uint16_t N_AMOSTRAS = 512; // Tem que ser potência de 2
long sample_interval_us = 1000000 / TAXA_SPS; // Aprox. 1953 µs

// --- Variáveis Globais ---
float Ieficaz_TrueRMS;     // Objetivo 1 (Potência Total)
float Frequencia_Rede;     // Objetivo 3 (Frequência)
float ADC_OFFSET_ZERO = 0.0; // Offset dinâmico

// Arrays para a FFT (Objetivo 2)
double vReal[N_AMOSTRAS];
double vImag[N_AMOSTRAS];
arduinoFFT FFT = arduinoFFT(vReal, vImag, N_AMOSTRAS, TAXA_SPS);

// ... (seu setup() aqui) ...

void loop() {
  // 1. Executa a medição e todos os cálculos
  medirSinais(); 

  // 2. Imprime ou envia os resultados via LoRa
  Serial.print("RMS (True): "); Serial.print(Ieficaz_TrueRMS, 2); Serial.println(" A");
  Serial.print("Frequência: "); Serial.print(Frequencia_Rede, 2); Serial.println(" Hz");

  // (Aqui você pode ver os resultados da FFT, como as harmônicas)
  // Ex: Serial.print("Energia na 3ª Harmônica (180Hz): ");
  // Serial.println(vReal[180]); // O bin 180 corresponde a 180 Hz
  
  // (Seu código LoRaWAN iria aqui)
}

/*
 * Esta função executa 3 tarefas em ~1 segundo:
 * 1. Calcula o True RMS da corrente (para Potência)
 * 2. Calcula a FFT (para análise de Harmônicas)
 * 3. Encontra a Frequência da Rede (a partir da FFT)
 */
void medirSinais() {
  float sumIinstant = 0;
  int16_t bitsads;

  // --- ETAPA 1: COLETAR AMOSTRAS (Dura 1 segundo) ---
  long next_sample_time = micros();
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, true);

  for (int i = 0; i < N_AMOSTRAS; i++) {
    // Espera sincronizada
    while (micros() < next_sample_time) {}
    next_sample_time += sample_interval_us;
    
    bitsads = ads.getLastConversionResults();
    
    // Atualiza o offset dinâmico
    ADC_OFFSET_ZERO = (ADC_OFFSET_ZERO * 0.999) + (bitsads * 0.001);
    
    float valor_sem_offset = (float)bitsads - ADC_OFFSET_ZERO;
    float Iinstant = valor_sem_offset * CALIBRATION_FACTOR;
    
    // --- Preparação para Cálculo 1: True RMS ---
    sumIinstant += sq(Iinstant);
    
    // --- Preparação para Cálculo 2: FFT ---
    vReal[i] = Iinstant;
    vImag[i] = 0;
  }
  ads.stopADCReading();

  // --- ETAPA 2: PROCESSAR OS DADOS (Rápido, < 50ms) ---
  
  // --- OBJETIVO 1: Calcular True RMS ---
  Ieficaz_TrueRMS = sqrt(sumIinstant / N_AMOSTRAS);
  
  // Aplicar o Noise Gate que discutimos
  if (Ieficaz_TrueRMS < NOISE_THRESHOLD_AMPS) {
    Ieficaz_TrueRMS = 0.0;
  }
  
  // --- OBJETIVO 2: Calcular a FFT ---
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude(); // Converte para magnitude de energia

  // --- OBJETIVO 3: Calcular a Frequência ---
  // Acha o pico de energia (ignorando DC/Bin 0)
  double max_magnitude = 0;
  int bin_fundamental = 0;
  // (Procuramos apenas na faixa esperada, ex: 50 a 70 Hz)
  for (int i = 50; i < 70; i++) {
    if (vReal[i] > max_magnitude) {
      max_magnitude = vReal[i];
      bin_fundamental = i;
    }
  }
  
  // A resolução do bin é 1Hz, então o índice do bin é a frequência
  Frequencia_Rede = (float)bin_fundamental;
}