/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2s.h"
#include "lwip.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum GLOBAL_STATE {
  STATE_ERROR,
  STATE_IDLE,
  STATE_INIT,
  STATE_FILL_BUF,
  STATE_START_I2S,
  STATE_PLAYING
} ;

enum I2S_STATE {
  STATE_NONE,
  STATE_HALFCPLT,
  STATE_CPLT
} ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RECV_BUF_SIZE 64000
#define I2S_BUF_SIZE 4000
#define I2S_BUF_SIZE_HALF (I2S_BUF_SIZE/2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum GLOBAL_STATE global_state = STATE_IDLE;
volatile enum I2S_STATE i2s_state = STATE_NONE;
volatile bool wav_data_available = false;

struct {
  void* ptr;
  uint16_t len;
} udp_payload;
uint8_t *recv_buf_ptr_write;
uint8_t *recv_buf_ptr_read;

uint8_t recv_buffer[RECV_BUF_SIZE];
uint8_t i2s_buffer[I2S_BUF_SIZE];

int num_samples_received = 0;

#pragma pack(1)
typedef struct {
  unsigned char ckid_riff[4];
  unsigned int cksize;
  unsigned char ckid_wave[4];
  unsigned char ckid_fmt[4];
  unsigned int samplesize;
  unsigned short wFormatTag;
  unsigned short nChannels;
  unsigned int nSamplesPerSec;
  unsigned int nAvgBytesPerSec;
  unsigned short nBlockAlign;
  unsigned short wBitsPerSample;
  unsigned char ckid_data[4];
  unsigned int datasize;
} WAVEFILE_HDR;

#pragma pack(1)
WAVEFILE_HDR* wavhdr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void udp_recv_fn_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
  if (port != 1337) {
    return;
  }
  memcpy(recv_buf_ptr_write, (uint8_t*)p->payload, p->len);
  recv_buf_ptr_write = (recv_buf_ptr_write+p->len >= recv_buffer+RECV_BUF_SIZE) ? recv_buffer : recv_buf_ptr_write+p->len;
  num_samples_received += p->len;
  pbuf_free(p);
  wav_data_available = true;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  i2s_state = STATE_HALFCPLT;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  i2s_state = STATE_CPLT;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2S2_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  
  recv_buf_ptr_write = recv_buffer;
  recv_buf_ptr_read = recv_buffer;

  int samples_counter = 0;

  // allocate memory for header
  wavhdr = (WAVEFILE_HDR*)malloc(sizeof(WAVEFILE_HDR));

  // temp
  memset(i2s_buffer, 0, I2S_BUF_SIZE);
  HAL_I2S_MspInit(&hi2s2);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MX_LWIP_Process();
    switch (global_state)
    {
      case STATE_IDLE:
        if(wav_data_available) {          
          global_state = STATE_INIT;
        } 
        break;

      case STATE_INIT:
        // here we assume that if we have just entered init, we are guaranteed to have at least WAVE header data available
        memcpy((uint8_t*)wavhdr, recv_buf_ptr_read, sizeof(WAVEFILE_HDR));
        recv_buf_ptr_read += sizeof(WAVEFILE_HDR);
        // copy initial wav data
        if(recv_buf_ptr_write-recv_buffer >= I2S_BUF_SIZE+sizeof(WAVEFILE_HDR)) {
          memcpy(i2s_buffer, recv_buf_ptr_read, I2S_BUF_SIZE);
          global_state = STATE_START_I2S;
        } else {
          global_state = STATE_FILL_BUF;
        }
        break;

      case STATE_FILL_BUF:
        if(recv_buf_ptr_write-recv_buffer < I2S_BUF_SIZE+sizeof(WAVEFILE_HDR)) {
          // not enough data received (yet?)
          global_state = STATE_FILL_BUF;
        } else {
          memcpy(i2s_buffer, recv_buf_ptr_read, I2S_BUF_SIZE);
          global_state = STATE_START_I2S;
        }
        break;

      case STATE_START_I2S:
        HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)i2s_buffer, I2S_BUF_SIZE_HALF);
        global_state = STATE_PLAYING;
        break;

      case STATE_PLAYING:
        switch (i2s_state) {

          case STATE_HALFCPLT:
            memcpy(i2s_buffer, recv_buf_ptr_read, I2S_BUF_SIZE_HALF);            
            recv_buf_ptr_read = (recv_buf_ptr_read+I2S_BUF_SIZE >= recv_buffer+RECV_BUF_SIZE) ? recv_buffer : recv_buf_ptr_read+I2S_BUF_SIZE_HALF;
  
            i2s_state = STATE_NONE;
            break;

          case STATE_CPLT:
            memcpy((uint8_t*)(i2s_buffer+I2S_BUF_SIZE_HALF), recv_buf_ptr_read, I2S_BUF_SIZE_HALF);
            recv_buf_ptr_read = (recv_buf_ptr_read+I2S_BUF_SIZE >= recv_buffer+RECV_BUF_SIZE) ? recv_buffer : recv_buf_ptr_read+I2S_BUF_SIZE_HALF;

            //samples_counter += I2S_BUF_SIZE;

            if (samples_counter >= wavhdr->datasize) {
              samples_counter = 0;
              HAL_I2S_DMAStop(&hi2s2);
              global_state = STATE_IDLE;
              wav_data_available = false;
            }
            i2s_state = STATE_NONE;
            break;

          case STATE_NONE:
          default:
            break;
        }
        break;
        
      case STATE_ERROR:
        break;

      default:
        break;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
