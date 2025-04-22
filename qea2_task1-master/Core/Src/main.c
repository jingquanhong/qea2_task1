/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "atk_md0350.h"
#include "fft.h"
#include "bsp_dwt.h"
////dac�����ر���
//const uint16_t Sine12bit_50[50] = {
//    0x0800,0x0901,0x09FD,0x0AF2,0x0BDA,0x0CB3,0x0D79,0x0E29,0x0EC0,0x0F3C,
//    0x0F9B,0x0FDB,0x0FFB,0x0FDB,0x0F3C,0x0EC0,0x0E29,0x0D79,0x0CB3,0x0BDA,
//    0x0AF2,0x09FD,0x0901,0x0800,0x06FF,0x0603,0x050E,0x0426,0x034D,0x0287,
//    0x01D7,0x0140,0x00C4,0x0065,0x0025,0x0005,0x0025,0x0065,0x00C4,0x0140,
//    0x01D7,0x0287,0x034D,0x0426,0x050E,0x0603,0x06FF,0x0800
//};
const uint16_t Sine12bit_50[50] = {
    0x0800, 0x084D, 0x0899, 0x08E3, 0x092A, 0x096E, 0x09AD, 0x09E6, 0x0A18, 0x0A42,
    0x0A63, 0x0A7C, 0x0A8B, 0x0A7C, 0x0A42, 0x0A18, 0x09E6, 0x09AD, 0x096E, 0x092A,
    0x08E3, 0x0899, 0x084D, 0x0800, 0x07B3, 0x0767, 0x071D, 0x06D6, 0x0692, 0x0653,
    0x061A, 0x05E8, 0x05BE, 0x059D, 0x0584, 0x0575, 0x0584, 0x059D, 0x05BE, 0x05E8,
    0x061A, 0x0653, 0x0692, 0x06D6, 0x071D, 0x0767, 0x07B3, 0x0800
};

//uint32_t TIM8_arr=839;
uint32_t TIM8_arr=335;
uint32_t current_freq;
uint32_t target_freq=10000;
void start_dac_output(void);
void frequency_change(int delt_freq,int flag1,int flag2);

int key_flag1,key_flag2,key_flag3=0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**my define**/
#define GRAPH_WIDTH 500     // 曲线显示的宽�?
#define GRAPH_HEIGHT 120    // 曲线显示的高�?
#define GRAPH_X_OFFSET 50    // 曲线显示的X偏移 ԭ����0
#define GRAPH_Y_OFFSET 30    // 曲线显示的Y偏移
#define GRAPH_Y_OFFSET_FFT 190    // 曲线显示的Y偏移


#define GRAPH_COLOR ATK_MD0350_BLUE  // 曲线的颜�?
#define BACKGROUND_COLOR ATK_MD0350_WHITE  // 背景颜色
#define GRID_COLOR ATK_MD0350_LIGHTGRAY  // 网格颜色
/*****/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern float fft_outputbuf[FFT_LENGTH*2];
extern uint16_t ADC_1_Value_DMA[FFT_LENGTH];//ADC��������
extern SignalInfo_t signal_info_real;//�����ź���Ϣ
int16_t show_value1;
float show_vpp;
int16_t show_value2;
int16_t size;
int16_t size_t_;
volatile uint8_t task1_done = 0;  // ���� 1 ״̬��־
/***********************/
void DrawGrid(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
//    // 绘制水平�?
//    for (uint16_t i = 0; i <= height; i += 20) {
//        atk_md0350_draw_line(x, y + i, x + width, y + i, color);
//    }
//    // 绘制垂直�?
//    for (uint16_t i = 0; i <= width; i += 40) {
//        atk_md0350_draw_line(x + i, y, x + i, y + height, color);
//    }
}

// 绘制动�?�曲�?
// 绘制动�?�曲线（支持 float 数组�?
void DrawDynamicGraph_float(uint16_t x, uint16_t y, uint16_t width, uint16_t height, float *data, uint32_t data_size) {
    static uint16_t prev_x = 0;
    static uint16_t prev_y = 0;
    uint16_t current_x, current_y;
    
    // 1. 找到数组的最大�?�（用于归一化显示）
    float max_value = 0.0f;
    for (uint32_t i = 0; i < data_size; i++) {
        if (data[i] > max_value) {
            max_value = data[i];
        }
    }
    if (max_value == 0.0f) max_value = 1.0f; // 避免除以0

    // 2. 清空曲线区域
    atk_md0350_fill(x, y, x + width, y + height, BACKGROUND_COLOR);
    // 3. 绘制网格（可选）
    DrawGrid(x, y, width, height, ATK_MD0350_BLACK);

    // 4. 绘制曲线
    for (uint32_t i = 0; i < data_size; i++) {
        current_x = x + (i * width) / data_size;
        current_y = y + height - (uint16_t)((data[i] * height) / max_value);

        if (i > 0) {
            atk_md0350_draw_line(prev_x, prev_y, current_x, current_y, GRAPH_COLOR);
        }
        prev_x = current_x;
        prev_y = current_y;
    }
}
void DrawDynamicGraph(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *data, uint32_t data_size) {
    static uint16_t prev_x = 0;
    static uint16_t prev_y = 0;
    uint16_t current_x, current_y;
    uint16_t max_value = 4095;  // 假设ADC�?12位，�?大�??4095

    // 清空曲线区域
    atk_md0350_fill(x, y, x + width, y + height, BACKGROUND_COLOR);
    // 绘制网格
    DrawGrid(x, y, width, height, ATK_MD0350_BLACK);

    // 绘制曲线
    for (uint32_t i = 0; i < data_size; i++) {
        current_x = x + (i * width) / data_size;
        current_y = y + height - (data[i] * height) / max_value;

        if (i > 0) {
            atk_md0350_draw_line(prev_x, prev_y, current_x, current_y, GRAPH_COLOR);
        }

        prev_x = current_x;
        prev_y = current_y;
    }
}

/***********************/
void Restart_ADC_DMA(void) {
    if (task1_done) {
        // ���¿��� DMA �ɼ�
        __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TC);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_1_Value_DMA, FFT_LENGTH);
    }
}


void key_scan(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static unsigned int task_count = 0;
static float wasteT = 0; //单位秒s
static int time_count =0;//计数变量 用于控制频率


	int key0 = 0, key0_press = 0;
  int key1 = 0, key1_press = 0;
  int key2 = 0, key2_press = 0;

int SAMPLING_RATE =20000;


void Task1_Start(void) {
    // ģ������ 1 ���У�����������ݴ���?
	
//    HAL_Delay(500);  // ���ﻻ��ʵ�ʵ������߼�
	
	
   signal_info_real=capture_and_FFT( FFT_LENGTH, ADC_1_Value_DMA,  SAMPLING_RATE);
	
  size = sizeof(ADC_1_Value_DMA) / sizeof(ADC_1_Value_DMA[0]);//显示数组ADC_1_Value_DMA
	size_t_ = sizeof(fft_outputbuf) / sizeof(fft_outputbuf[0]); //显示傅里叶变换的数组
	
	DrawDynamicGraph(GRAPH_X_OFFSET, GRAPH_Y_OFFSET, GRAPH_WIDTH, GRAPH_HEIGHT, ADC_1_Value_DMA, size);
    // ���� 1 ����
    task1_done = 1;

    // ���¿��� DMA �ɼ�
    Restart_ADC_DMA();
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
		HAL_Delay(1000);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
	    HAL_Delay(100);
			DWT_Init(168);
	atk_md0350_init();
  atk_md0350_clear(ATK_MD0350_WHITE);
	
//		HAL_Delay(100);

//	char msg_author[64];
//  snprintf(msg_author, sizeof(msg_author), "baoli");
//  //atk_md0350_fill(180, 160, 180 + 250, 160 + 16, ATK_MD0350_BLACK);
//  atk_md0350_show_string(0, 0, 300, 40, msg_author, ATK_MD0350_LCD_FONT_32, ATK_MD0350_RED);
//	atk_md0350_draw_rect(0, 0, 300, 30, ATK_MD0350_BLACK);
  FFT_INIT();
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);//�Լ���������
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
	
	
  start_dac_output();//����dac���
	HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		DWT_GetDeltaT(&task_count);
		
			if(time_count%1==0){
				
		signal_info_real=capture_and_FFT( FFT_LENGTH, ADC_1_Value_DMA,  SAMPLING_RATE);
		size = sizeof(ADC_1_Value_DMA) / sizeof(ADC_1_Value_DMA[0]);
		size_t_ = sizeof(fft_outputbuf) / sizeof(fft_outputbuf[0]); //显示傅里叶变换的数组
	  DrawDynamicGraph(GRAPH_X_OFFSET, GRAPH_Y_OFFSET_FFT, GRAPH_WIDTH-GRAPH_X_OFFSET, GRAPH_HEIGHT, ADC_1_Value_DMA, size/5);//160
		DrawDynamicGraph_float(GRAPH_X_OFFSET, GRAPH_Y_OFFSET, GRAPH_WIDTH-GRAPH_X_OFFSET, GRAPH_HEIGHT, fft_outputbuf, size_t_);
				
				
			}
			
			
			
		if(time_count%2==0){
			key_scan();
		}
			
			
		
		
		if(time_count%400==0){
			
			if(key0_press)
			{
				SAMPLING_RATE+=1000;//�������Ƶ��
				
			}
			
	  	frequency_change(100,key2_press,key1_press);//һ��ʼ10Khz
			

		
			
		}			
		
		if(time_count%500==0){
			
			
    char msg_author[64];
		
		show_value1 = signal_info_real.main_freq;
		
    snprintf(msg_author, sizeof(msg_author), "main_freq: %d", show_value1);  // 示例：显示第�?个ADC�?
		
    // 3. 在LCD上显示字符串
    atk_md0350_show_string(0, 0, 300, 40, msg_author, ATK_MD0350_LCD_FONT_32, ATK_MD0350_RED);
		
			
		
		

//    char vpp[64];
//		
//		show_vpp = 3.3;
//		
//    snprintf(vpp, sizeof(vpp), "%.2f", show_vpp);  // 示例：显示第�?个ADC�?
//		
//    // 3. 在LCD上显示字符串
//    atk_md0350_show_string(10, 190, 10, 10, vpp, ATK_MD0350_LCD_FONT_32, ATK_MD0350_RED);
		

		atk_md0350_fill(0, 0, 300, 40, ATK_MD0350_WHITE);
		
	}
		

		
		
		
		
		float Task_T = DWT_GetDeltaT(&task_count);
    wasteT = 0.001f - Task_T;
		if(wasteT>=0)
    DWT_Delay(wasteT);
    time_count++;//1000hz自增
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

void start_dac_output(void)//����dac���
{
	    __HAL_TIM_SET_AUTORELOAD(&htim8, TIM8_arr);
        HAL_TIM_Base_Start(&htim8);
        HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Sine12bit_50, 50, DAC_ALIGN_12B_R);

}

void frequency_change(int delt_freq,int flag1,int flag2)
{

		 current_freq = 3360000 / (TIM8_arr + 1);//��ʼֵ4000 ʵ��Ƶ����4000/50==80

		if(flag1==1&&flag2==0)
		{
			 target_freq += delt_freq;
       if (target_freq > 10000) target_freq = 10000;
		}
		if(flag1==0&&flag2==1)
		{
       target_freq -= delt_freq;
       if (target_freq < 100) target_freq = 100;			
			
		}
		if(flag1==flag2)
		{
			//do nothing
			
		}
		TIM8_arr = (3360000 / target_freq) - 1;
		__HAL_TIM_SET_AUTORELOAD(&htim8, TIM8_arr);//10kHZ���������Ƶ��

}

void key_scan(void)
{

	
	if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) {
          DWT_Delay(0.05);
       if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) {
              if (!key0_press) {
                  key0 = !key0;

//                  if (key0) {

//                  } else {

//                  }
              }
              key0_press = 1;
          }
      } else {
          key0_press = 0;
      }
			
			
		if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
          DWT_Delay(0.05);
          if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
              if (!key1_press) {
                  key1 = !key1;

//                  if (key1) {
//          
//                  } else {

//                          }
              }

              key1_press = 1;
          }
      } else {
          key1_press = 0;
      }

			
			
      if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
          DWT_Delay(0.05);
          if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
              if (!key2_press) {
                  key2 = !key2;
								
//								    if (key2) {
//          
//                    } else {

//                          }

              }

              key2_press = 1;
          }
      } else {
          key2_press = 0;
      }

			
			
			
			
			
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//    if (hadc->Instance == ADC1) {
//        // 1. �ر� DMA ��������ж�?
//        __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);

//        // 2. �������� 1
//        task1_done = 0;
//        Task1_Start();
//    }
//}


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
