#include <driver/i2s.h>
#include "asetaury.h"
#include <SSD1306Wire.h>

#define LEQ_PERIOD        1           // second(s)
#define WEIGHTING         A_weighting // Also available: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS         "LAeq"      // customize based on above weighting used
#define DB_UNITS          "dBA"       // customize based on above weighting used
#define USE_DISPLAY       1

#define MIC_EQUALIZER     ICS43434    // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB     3.0103      // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

#define MIC_SENSITIVITY   -26         // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB        94.0        // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB   116.0       // dB - Acoustic overload point
#define MIC_NOISE_DB      29          // dB - Noise floor
#define MIC_BITS          24          // valid number of bits in I2S data
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT  0           // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

#define I2S_WS            25
#define I2S_SCK           32
#define I2S_SD            33
#define I2S_PORT          I2S_NUM_0

#if (USE_DISPLAY > 0)
  #define OLED_GEOMETRY     GEOMETRY_64_48
  #define OLED_FLIP_V       1
  SSD1306Wire display(0x3c, SDA, SCL, OLED_GEOMETRY);
#endif

// Properly initialize filters without designated initializers
SOS_IIR_Filter DC_BLOCKER = { 
  1.0, 
  {
    {-1.0, 0.0, +0.9992, 0}
  }
};

SOS_IIR_Filter ICS43434 = { 
  0.477326418836803,
  { 
   {0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128},
   {-1.98905931743624453, 0.98908924206960169, 1.99755331853906037, -0.99755481510122113}
  }
};

SOS_IIR_Filter A_weighting = {
  0.169994948147430, 
  { 
    {-2.00026996133106, 1.00027056142719, -1.060868438509278, -0.163987445885926},
    {4.35912384203144, 3.09120265783884, 1.208419926363593, -0.273166998428332},
    {-0.70930303489759, -0.29071868393580, 1.982242159753048, -0.982298594928989}
  }
};

#define SAMPLE_RATE       48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS       32    // bits
#define SAMPLE_T          int32_t 
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

struct sum_queue_t {
  float sum_sqr_SPL;
  float sum_sqr_weighted;
  uint32_t proc_ticks;
};
QueueHandle_t samples_queue;

float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

void mic_i2s_init() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BANKS,
    .dma_buf_len = DMA_BANK_SIZE,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,  
    .ws_io_num = I2S_WS,    
    .data_out_num = -1, 
    .data_in_num = I2S_SD   
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  #if (MIC_TIMING_SHIFT > 0) 
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  
  #endif
  
  i2s_set_pin(I2S_PORT, &pin_config);
}

#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048

void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();

  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();
    
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for(int i = 0; i < SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);
    q.proc_ticks = xTaskGetTickCount() - start_tick;

    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

void setup() {
  setCpuFrequencyMhz(80); 

  Serial.begin(112500);
  delay(1000); 
  
  #if (USE_DISPLAY > 0)
    display.init();
    #if (OLED_FLIP_V > 0)
      display.flipScreenVertically();
    #endif
    display.setFont(ArialMT_Plain_16);
  #endif

  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);

  sum_queue_t q;
  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  double Leq_dB = 0;

  while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);

      #if (USE_DISPLAY > 0)
        display.clear();
        display.drawString(0, 0, LEQ_UNITS + String(" = ") + String(Leq_dB, 1) + String(" ") + DB_UNITS);
        display.display();
      #endif

      Serial.printf("%.1f\n", Leq_dB);
      
      Leq_samples = 0;
      Leq_sum_sqr = 0;
    }
  }
}

void loop() {
  delay(1000);
}
