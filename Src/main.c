/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "fprint.h"
#include "math.h"
#include "svf10_util.h"

/* Defines -------------------------------------------------------------------*/

//#define AUTO_SAVE
//#define AUTO_SAVE_FRAME      3
#define IMAGE_PROCESS

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart4;

extern uint8_t uart_receive_frame[21];
extern uint8_t svf10_resister_frame[18];
extern float svf10_average, svf10_std_dev;
uint8_t svf10_chip_id[8];
uint8_t svf10_status_buffer[8];
extern uint16_t svf10_display_threshold;

uint8_t receive_end_flag = 0;
uint8_t receive_try_flag = 0;
uint8_t chip_id_buffer[8];
uint8_t first_do_flag = 1;
uint16_t svf10_capture_delay = 0;

/* variables for Histogram Function ------------------------------------------*/
uint8_t svf10_origine_buffer[SVF10_RECEIVE_COUNT];
uint8_t svf10_origine_offset[SVF10_RECEIVE_COUNT];
uint8_t svf10_image_data[SVF10_Y_PIXEL][SVF10_X_PIXEL]; 
float  svf10_result_data[SVF10_Y_PIXEL][SVF10_X_PIXEL];
float  svf10_result_data2[SVF10_Y_PIXEL][SVF10_X_PIXEL];

//double histogram1[256], histogram2[256];
uint8_t INDEX;

float coverage;

int16_t         N_Capture_Frame;
/* variable for uart communicatio  -------------------------------------------*/
uint8_t uart_read_chk=0, uart_read_try=0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);

int main(void)
{
//  #ifdef MATLAB_TEST
//    int C_INT =0;
//  #endif


  HAL_Init();

  SystemClock_Config();
  
  HAL_Delay(200);
  
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  SVF10_CS_DIS;
  
  SVF_DLY2X_OSC = OSC_10MHZ;     // 0
  SVF_S_GAIN = GAIN_2X;        // 03
  SVF_ADC_RSEL = RSEL_INTERAL;  // 0
  SVF_DYL2X = DYL2X_15NS;       // 1
  SVF_CLK_SEL = CK_SYS;         // 00
  SVF_PERIOD = PERIOD_001;      // 001
  SVF_ISOL = ISOL_10LINE;       // 00
  SVF_SL_TIME = SL_1000MS;      // 3
  SVF_NAVI = NAVI_OFF;
  SVF_SCK_UT = SCK_UT_MODE_00;
  SVF_SLV_STU = SLV_SUT_CAP_O;
  SVF_MULTIPLYING = 5;          // 
  SVF_ADC_REF = 5;              // 
  SVF_GPB_REG = 0xe0;
  
  svf10_display_threshold = 100;   // value/100
  
  SVF_SENSING_MODE = SENSING_NORMAL;
  SVF_IMAGE_FILTER = NO_FILTER;
  
  svf10_capture_delay = 25*(SVF_DLY2X_OSC+1)*(SVF_SCK_UT+1);
  
  
    
  HAL_UART_Receive_IT(&huart4, uart_receive_frame, 21);
  
  SVF10_Chip_ID_Read(svf10_chip_id);
  HAL_Delay(1);
  SVF10_Chip_ID_Read(svf10_chip_id);
  HAL_Delay(1);
  
  SVF10_Memory_Read_Mode5();
  HAL_Delay(1);
  
  SVF10_Sreg_Set();
  HAL_Delay(1);

  SVF10_Sreg_Read_Creg_Set(svf10_status_buffer);
  HAL_Delay(1);
  
  SVF10_Capture_Offset();
  HAL_Delay(svf10_capture_delay);
  
  SVF10_Memory_Read_Mode3(svf10_origine_offset);
  HAL_Delay(1);
 
  while (1)
  {  
//    #ifdef MATLAB_TEST
//    C_INT++;
//    #endif

    SVF10_Memory_Read_Mode5();
    HAL_Delay(1);    

    SVF10_Sreg_Set();
    HAL_Delay(1);
  
    SVF10_Sreg_Read_Creg_Set(svf10_status_buffer);
    HAL_Delay(1);
    
//    SVF10_Sleep_Sens();
//    HAL_Delay(200);

    
// TEST   
    SVF10_Capture_Offset();
    HAL_Delay(svf10_capture_delay);
// TEST  
    
//    SVF10_Memory_Read_Mode0();
  
    SVF10_Memory_Read_Mode3(svf10_origine_buffer);
    HAL_Delay(1);
    //SVF10_Memory_Read_Mode0();

    for(uint16_t i=0; i<SVF10_RECEIVE_COUNT; i++)
      if ( svf10_origine_buffer[i] <= svf10_origine_offset[i])
        //svf10_origine_buffer[i] = 0;
        svf10_origine_buffer[i] = svf10_origine_offset[i] - svf10_origine_buffer[i];
        //svf10_origine_buffer[i] = svf10_origine_offset[i];
    
      else
        //svf10_origine_buffer[i] = 0;
        svf10_origine_buffer[i] = svf10_origine_buffer[i] - svf10_origine_offset[i];
        //svf10_origine_buffer[i] = svf10_origine_offset[i] - svf10_origine_buffer[i];
        //svf10_origine_buffer[i] = svf10_origine_offset[i];

    uniform_image_ver(svf10_origine_buffer, svf10_image_data, svf10_result_data);
    uniform_image_hor(svf10_origine_buffer, svf10_image_data, svf10_result_data);
    image_quality(svf10_origine_buffer, svf10_image_data);
    
//  svf10_average  // svf10_std_sev    
//    moving_aver_by3(svf10_origine_buffer, svf10_image_data, svf10_result_data);
    
    coverage = finger_coverage(8, svf10_origine_buffer, svf10_image_data);    
    
//    if (temp2 >= ((float)(display_threshold)/100))
    if (coverage >= 0.9)
    {
      N_Capture_Frame++;
#ifdef IMAGE_PROCESS      
      //moving_aver_by4();
 //    wiener_filter(2)
//    hist_eq(svf10_origine_buffer, svf10_image_data);
 //     noise_filter(1, svf10_origine_buffer, svf10_image_data, svf10_result_data);
//      hist_ep(svf10_origine_buffer, svf10_image_data, 0, 255);
//      hist_ep(svf10_origine_buffer, svf10_image_data, 2, 8);
//      image_quality(svf10_origine_buffer, svf10_image_data);
      switch (SVF_SENSING_MODE) {
      case SENSING_NORMAL:
        hist_ep(svf10_origine_buffer, svf10_image_data, (uint8_t) (svf10_average-(svf10_std_dev/2.0)), (uint8_t) (svf10_average+(svf10_std_dev/2.0)), 70, 185, EP_OPTION_LINEAR);
        break;
      case SENSING_WET:
        hist_ep(svf10_origine_buffer, svf10_image_data, (uint8_t) (svf10_average-(svf10_std_dev/1.5)), (uint8_t) (svf10_average+(svf10_std_dev/1.5)), 50, 205, EP_OPTION_LINEAR);
        break;
      case SENSING_DRY:
        hist_ep(svf10_origine_buffer, svf10_image_data, (uint8_t) (svf10_average-(svf10_std_dev/2.5)), (uint8_t) (svf10_average+(svf10_std_dev/2.5)), 90, 165, EP_OPTION_LINEAR);
        break;
      default:
        break;        
      }

      noise_filter(1, svf10_origine_buffer, svf10_image_data, svf10_result_data);
      if (SVF_IMAGE_FILTER == WEINER_FILTER) 
        wiener_filter(1, svf10_origine_buffer, svf10_image_data, svf10_result_data, svf10_result_data2);
//      gaussian_filter_by3(svf10_origine_buffer, svf10_image_data, svf10_result_data );
//      hist_ep(svf10_origine_buffer, svf10_image_data, 0, 255);

#endif
      
#ifdef AUTO_SAVE
      if (N_Capture_Frame == AUTO_SAVE_FRAME)
      {
        SVF10_Send_Image(svf10_origine_buffer, svf10_bitmap_header_data);
      }
#else
       SVF10_Send_Image(svf10_origine_buffer, svf10_bitmap_header_data);
#endif      
    }
    else if (coverage < 0.70)
    {
      N_Capture_Frame = 0;
      HAL_Delay(500);
    }
    else
    {
      
    }

#ifndef AUTO_SAVE    
    #ifdef MATLAB_TEST
      HAL_UART_Transmit(&huart4, svf10_origine_buffer, SVF10_RECEIVE_COUNT, 2000);
    #else
       SVF10_Send_Image(svf10_origine_buffer, svf10_bitmap_header_data);
    #endif     
#endif      

      
    if(receive_end_flag)
    {
      receive_end_flag = 0;
      receive_try_flag = 1;
      
      if((uart_receive_frame[0] == 'R') && (uart_receive_frame[1] == 'E') && (uart_receive_frame[2] == 'S'))
      {
        for(uint8_t i=0; i<18 ; i++)
        {
           svf10_resister_frame [i] = uart_receive_frame[i+3];
        }
      
        svf10_display_threshold = SVF_DISPLAY_THRESHOLD;
        svf10_capture_delay = 25*(SVF_DLY2X_OSC+1)*(SVF_SCK_UT+1);
  
        SVF10_Chip_ID_Read(svf10_chip_id);
        HAL_Delay(1);
  
        SVF10_Memory_Read_Mode5();
        HAL_Delay(1);
  
        SVF10_Sreg_Set();
        HAL_Delay(1);

        SVF10_Sreg_Read_Creg_Set(svf10_status_buffer);
        HAL_Delay(1);
  
        SVF10_Capture_Offset();
        HAL_Delay(svf10_capture_delay);
  
        SVF10_Memory_Read_Mode3(svf10_origine_offset);
        HAL_Delay(1);
        }
      
      if((uart_receive_frame[0] == 'R') && (uart_receive_frame[1] == 'I') && (uart_receive_frame[2] == 'D'))
      {
         HAL_UART_Transmit(&huart4, svf10_chip_id, 6, 2000);
      }
          
      while(!(HAL_OK == (HAL_UART_Receive_IT(&huart4, uart_receive_frame, 21))));
//      HAL_UART_Receive_IT(&huart4, uart_receive_frame, 21);
    }
    
    HAL_Delay(5); 
    HAL_UART_Receive_IT(&huart4, uart_receive_frame, 21);
  }
}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;  //   SPI 모드 설정 Mode 0
//  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;  //   SPI 모드 설정 Mode 2
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
#ifdef SVF10P_R2
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
#else
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
#endif
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 921600;
//  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
