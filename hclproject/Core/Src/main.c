#include "main.h"
#include "dma.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "arm_math.h"
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 1024
#define FFT_SIZE 1024
#define GAIN 2500
#define FILTER_TAP_NUM 32
#define FFT_SCALE_FACTOR (2.0f/FFT_SIZE)

#define MIN_FREQ 20
#define MAX_FREQ 8000
#define SAMPLING_RATE 8000.0f

//Calibration
#define CALIBRATION_TIME 10000
static float32_t calibration_sum = 0.0f;
static uint32_t calibration_samples = 0;
int calibrated = 0;
float32_t prev_average_magnitude = 0;

char temp[64];
// Add these at the top
#define VOCAL_START_BIN 7   // 50Hz (7*7.8Hz = 54.6Hz)
#define VOCAL_END_BIN 39    // 300Hz (39*7.8Hz = 304.2Hz)
#define BIN_GROUP_SIZE 3
#define SPIKE_THRESHOLD 4   // Adjust based on the room noise
uint8_t spike_counter = 0;
#define NO_OF_SPIKES 3

//Alert transmission
volatile uint8_t alert_pending = 0;
#define UART_FLUSH_DELAY 20


int32_t i2s_rx_buffer[2][BUFFER_SIZE];
static float32_t fft_input[FFT_SIZE] __attribute__((aligned(4)));
static float32_t fft_output[FFT_SIZE] __attribute__((aligned(4)));
static float32_t mag_output[FFT_SIZE / 2] __attribute__((aligned(4)));

volatile uint8_t active_buffer = 0;
volatile uint8_t fft_ready = 0;

// Optimized bandpass filter coefficients
const float32_t bp_filter[FILTER_TAP_NUM] = {
    -0.001747f, -0.003912f, -0.004215f, 0.000000f, 0.009645f,
    0.017835f, 0.011983f, -0.015430f, -0.042257f, -0.042351f,
    0.000000f, 0.063517f, 0.098550f, 0.063517f, 0.000000f,
    -0.042351f, -0.042257f, -0.015430f, 0.011983f, 0.017835f,
    0.009645f, 0.000000f, -0.004215f, -0.003912f, -0.001747f
};

float32_t hanning_window[FFT_SIZE];
float32_t fir_state[FFT_SIZE + FILTER_TAP_NUM - 1];
arm_rfft_fast_instance_f32 fft_instance;
arm_fir_instance_f32 fir_instance;

void SystemClock_Config(void);

// DMA Callbacks
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    active_buffer = 0;
    fft_ready = 1;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
    active_buffer = 1;
    fft_ready = 1;
}

void init_filters() {
    arm_fir_init_f32(&fir_instance, FILTER_TAP_NUM, bp_filter, fir_state, FFT_SIZE);
}

void bandpass_filter(float32_t *input, float32_t *output) {
    arm_fir_f32(&fir_instance, input, output, FFT_SIZE);
}

void send_frequency_data(float* mag_output) {
    const int start_bin = (int)(MIN_FREQ * FFT_SIZE / SAMPLING_RATE);
    const int end_bin = (int)(MAX_FREQ * FFT_SIZE / SAMPLING_RATE);
    char buffer[1024];
    int offset = 0;

    // Skip bin 0 if it's below the desired frequency range
    for (int i = (start_bin > 0 ? start_bin : 1); i <= end_bin; i++) {
        float freq = i * (SAMPLING_RATE / FFT_SIZE);  // 125Hz per bin
        float mag = mag_output[i];
        offset += snprintf(buffer + offset, sizeof(buffer) - offset,
                           "%.1fhz:%.2f ", freq, mag);
        if (offset >= sizeof(buffer) - 20) break; // Prevent overflow
    }

    if (offset > 0) {
        buffer[offset-1] = '\r'; // Replace last space
        buffer[offset] = '\n';
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, offset+1, 100);
    }
}


int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_I2S2_Init();
    MX_USART2_UART_Init();
    __HAL_UART_FLUSH_DRREGISTER(&huart2);

    // FFT initialization
    if (arm_rfft_fast_init_1024_f32(&fft_instance) != ARM_MATH_SUCCESS) {
        char debug_buf[50];
        sprintf(debug_buf, "FFT Init Failed!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)debug_buf, strlen(debug_buf), 100);
        while(1);
    }
    init_filters();
    HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*)i2s_rx_buffer, BUFFER_SIZE * 2);

    while(1) {
        if(fft_ready) {
            fft_ready = 0;
            int32_t *processing_buffer = i2s_rx_buffer[active_buffer];

            // DC offset calculation
            int32_t dc_offset = 0;
            for(int i=0; i<FFT_SIZE; i++) {
                dc_offset += (processing_buffer[i] << 8) >> 8;
            }
            dc_offset /= FFT_SIZE;

            // Convert to float
            for(int i=0; i<FFT_SIZE; i++) {
                int32_t pcm_data = ((processing_buffer[i] << 8) >> 8) - dc_offset;
                pcm_data *= GAIN;
                fft_input[i] = (float32_t)pcm_data / 8388608.0f;
            }

            // Filtering and windowing
            float32_t filtered_output[FFT_SIZE];
            bandpass_filter(fft_input, filtered_output);
            arm_hanning_f32(hanning_window, FFT_SIZE);
            for(int i=0; i<FFT_SIZE; i++) {
                fft_input[i] = filtered_output[i] * hanning_window[i];
            }

            // FFT processing
            arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);
            arm_cmplx_mag_f32(fft_output, mag_output, FFT_SIZE/2);

            for(int i=0; i<FFT_SIZE/2; i++) {
                mag_output[i] *= FFT_SCALE_FACTOR * GAIN;
            }

            send_frequency_data(mag_output);

            if(!calibrated) {
                if(HAL_GetTick() <= CALIBRATION_TIME) {
                    // Store baseline for each bin group
                    for(int i=VOCAL_START_BIN; i<VOCAL_END_BIN; i+=BIN_GROUP_SIZE) {
                        float group_sum = 0;
                        for(int j=0; j<BIN_GROUP_SIZE; j++){
                            if(i+j >= FFT_SIZE/2) break;
                            group_sum += mag_output[i+j];
                        }
                        calibration_sum += group_sum/BIN_GROUP_SIZE;
                        calibration_samples++;
                    }
                }
                else {
                    // Calculate average per bin group
                    prev_average_magnitude = calibration_sum / calibration_samples;
                    calibrated = 1;

                    char cal_msg[64];
                    snprintf(cal_msg, sizeof(cal_msg), "CALIBRATED:%.2f\r\n", prev_average_magnitude);

                    HAL_UART_Transmit(&huart2, (uint8_t*)cal_msg, strlen(cal_msg), 100);
                }
            }
            else {
                float current_spike_sum = 0;
                uint8_t spiked_groups = 0;

                // Check each bin group in vocal range
                for(int i=VOCAL_START_BIN; i<VOCAL_END_BIN; i+=BIN_GROUP_SIZE) {
                    float group_sum = 0;

                    // Calculate current group average
                    for(int j=0; j<BIN_GROUP_SIZE; j++){
                        if(i+j >= FFT_SIZE/2) break;
                        group_sum += mag_output[i+j];
                    }
                    float group_avg = group_sum/BIN_GROUP_SIZE;

                    // Detect spikes in this bin group
                    if(group_avg > (prev_average_magnitude * SPIKE_THRESHOLD)) {
                        spiked_groups++;
                        current_spike_sum += group_avg;
                    }
                }

                // If at least 2 bin groups spiked
                if(spiked_groups >= 2) {
                    spike_counter++;

                    // Require 3 consecutive detections
                    if(spike_counter >= NO_OF_SPIKES) {
                        float current_avg = current_spike_sum/spiked_groups;
                        char alert_msg[64];
                        snprintf(alert_msg, sizeof(alert_msg),"ALERT:%.2f/%.2f\r\n",
                                current_avg, prev_average_magnitude);
                        HAL_UART_Transmit(&huart2, (uint8_t*)alert_msg, strlen(alert_msg), 100);
                        spike_counter = 0;
                    }
                }
                else {
                    // Decay spike counter
                    spike_counter = spike_counter > 0 ? spike_counter-1 : 0;
                }

                if(spike_counter == 0) {
                    prev_average_magnitude = 0.95f * prev_average_magnitude + 0.05f * current_spike_sum;
                }
            }


        }
        HAL_Delay(1);
    }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    __HAL_UART_FLUSH_DRREGISTER(huart);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)"FATAL_ERROR\r\n", 13, 100);
      HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
