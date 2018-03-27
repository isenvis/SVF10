/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "fprint.h"
#include "math.h"

/* Defines -------------------------------------------------------------------*/


#define SVF10_CS_EN          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define SVF10_CS_DIS         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET) 

#define SVF_DLY2X_OSC        svf10_resister_frame[0]
#define SVF_S_GAIN           svf10_resister_frame[1]
#define SVF_ADC_RSEL         svf10_resister_frame[2]
#define SVF_DYL2X            svf10_resister_frame[3]
#define SVF_CLK_SEL          svf10_resister_frame[4]
#define SVF_PERIOD           svf10_resister_frame[5]
#define SVF_ISOL             svf10_resister_frame[6]   
#define SVF_SL_TIME          svf10_resister_frame[7]
#define SVF_NAVI             svf10_resister_frame[8]
#define SVF_SCK_UT           svf10_resister_frame[9]
#define SVF_SLV_STU          svf10_resister_frame[10]
#define SVF_MULTIPLYING      svf10_resister_frame[11]
#define SVF_ADC_REF          svf10_resister_frame[12]
#define SVF_DISPLAY_THRESHOLD   ((((uint16_t)(svf10_resister_frame[13]))<<8)|((uint16_t)(svf10_resister_frame[14])))
#define SVF_GPB_REG          svf10_resister_frame[15]

#define OSC_10MHZ            0  
#define OSC_5MHZ             1  

#define GAIN_1X              0   
#define GAIN_1X2             1
#define GAIN_1X5             2
#define GAIN_2X              3

#define RSEL_INTERAL         0
#define RSEL_EXTERNAL        1

#define DYL2X_10NS           0
#define DYL2X_15NS           1

#define CK_SYS               0 
#define CK_DIV_2             1 
#define CK_DIV_4             2 
#define CK_DIV_8             3

#define PERIOD_000           0 
#define PERIOD_001           1 
#define PERIOD_010           2 
#define PERIOD_011           3 
#define PERIOD_100           4 
#define PERIOD_101           5 
#define PERIOD_110           6 

#define ISOL_6LINE           0
#define ISOL_10LINE          1

#define SL_DEEP              0
#define SL_50MS              1
#define SL_100MS             2
#define SL_150MS             3
#define SL_300MS             4
#define SL_500MS             5
#define SL_700MS             6
#define SL_1000MS            7

#define NAVI_OFF             0
#define NAVI_ON              1

#define SCK_UT_8INT          0
#define SCK_UT_16INT         1

#define SLV_SUT_CAP_O        0
#define SLV_SUT_CAP_N        1
#define SLV_SUT_SLEEP        2
#define SLV_SUT_MEM_R        3

// #define AUTO_SAVE
#define AUTO_SAVE_FRAME      3

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart4;

#ifdef SVF10P
  
  uint16_t svf10_x_pixel = 96;
  uint16_t svf10_y_pixel = 96;
  uint16_t svf10_receive_count = 9412;
  uint16_t svf10_header_count = 10296;
  uint8_t svf10_origine_buffer[9412];
  
#endif
  
#ifdef SVF10G
  
  uint16_t svf10_x_pixel = 192;
  uint16_t svf10_y_pixel = 56;
  uint16_t svf10_receive_count = 10868;
  uint16_t svf10_header_count = 11832;
  uint8_t svf10_origine_buffer[10868];
 
#endif
  
  

/* Private variables ---------------------------------------------------------*/

uint8_t svf10_chip_id[8];
uint8_t svf10_status_buffer[8];
uint16_t origine_image_count = 4;
uint16_t recon_image_count = 1078;


uint8_t uart_receive_frame[19];
uint8_t svf10_resister_frame[16];
uint8_t receive_end_flag = 0;
uint8_t chip_id_buffer[6];

/* variables for Histogram Function ------------------------------------------*/

uint8_t image_data[96][96]; 
double result_data[96][96];
double histogram[256];
double histogram1[256], histogram2[256];
double temp, temp2, temp3, temp4;
uint8_t INDEX;
double average;
int16_t         N_Capture_Frame;

uint16_t display_threshold = 500;

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
void SVF10_Send_Image(void);
void SVF10_Chip_ID_Read(void);
void SVF10_Power_On(void);
void SVF10_Memory_reset(void);
void SVF10_Memory_write(void);
void SVF10_Status_read(void);
void SVF10_Status_write(void);
void SVF10_Sleep_Mode(void);
void TEST_Status_read(void);

/* 2017.08.18 Add Function Strat----------------------------------------------*/
void SVF10_Capture_Offset(void);
void SVF10_Chip_ID_Read(void);
void SVF10_GPB_Set(void);
void SVF10_Memory_Read_Mode0(void);
void SVF10_Memory_Read_Mode1(void);
void SVF10_Memory_Read_Mode2(void);
void SVF10_Memory_Read_Mode3(void);
void SVF10_Memory_Read_Mode4(void);
void SVF10_Memory_Read_Mode5(void);
void SVF10_Sleep_Sens(void);
void SVF10_Sreg_Read_Creg_Set(void);
void SVF10_Sreg_Set(void);
/* 2017.08.18 Add Function End------------------------------------------------*/

/* 2017.08.18 Senvis Add Function Start---------------------------------------*/
void image_quality(void);
void moving_aver_by2(void);
void moving_aver_by3(void);
void moving_aver_by4(void);
void moving_aver_by5(void);
void gaussian_filter_by3(void);
void gaussian_filter_by5(void);
void fir50_by3(void);
void fir50_by5(void);
void fir100_by3(void);
void fir100_by5(void);
void fir150_by3(void);
void fir150_by5(void);
void fir150_by7(void);
void hist_eq(void);
void histo(void);
void SVF10_Send_Image_mark(void);
void SVF10_Send_Image_BS(void);
/* 2017.08.18 Senvis Add Function End-----------------------------------------*/


uint8_t test_buffer[2];



int main(void)
{
  #ifdef MATLAB_TEST
    int C_INT =0;
  #endif

  HAL_Init();

  SystemClock_Config();
  
  HAL_Delay(2000);
  
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  SVF10_CS_DIS;

  
  chip_id_buffer[0] = 'S';
  chip_id_buffer[1] = 'V';
  chip_id_buffer[2] = 'F';
  chip_id_buffer[3] = '1';
  chip_id_buffer[4] = '0';
  chip_id_buffer[5] = 'P';
  
  SVF_DLY2X_OSC = OSC_10MHZ;
  SVF_S_GAIN = GAIN_1X2;
  SVF_ADC_RSEL = RSEL_INTERAL;
  SVF_DYL2X = DYL2X_15NS;
  SVF_CLK_SEL = CK_SYS;
  SVF_PERIOD = PERIOD_001;
  SVF_ISOL = ISOL_10LINE;
  SVF_SL_TIME = SL_500MS;
  SVF_NAVI = NAVI_OFF;
  SVF_SCK_UT = SCK_UT_16INT;
  SVF_SLV_STU = SLV_SUT_CAP_O;
  SVF_MULTIPLYING = 3;
  SVF_ADC_REF = 100;
  SVF_GPB_REG = 0xf0;
  
  display_threshold = 500;   // value/100
  
  HAL_UART_Receive_IT(&huart4, uart_receive_frame, 19);
   
  SVF10_Chip_ID_Read();
  HAL_Delay(1);
  
  //SVF10_Status_write();
  SVF10_Sreg_Set();
  HAL_Delay(1);
  
  //SVF10_Status_read();
  SVF10_Sreg_Read_Creg_Set();
  HAL_Delay(1);
 
  //SVF10_Memory_reset();
  SVF10_Memory_Read_Mode5();
  HAL_Delay(1);
  
  //SVF10_Status_write();
  SVF10_Sreg_Set();
  HAL_Delay(1);
   
  SVF10_Capture_Offset();
  HAL_Delay(1000);
  
  
  
#ifdef AUTO_DETEC  
    for(uint16_t i=0; i<10; i++)
    {
      SVF10_Send_Image_mark();
      HAL_Delay(500);      
    }

    ADC_REF = 0x10;
   
    for(uint16_t i=0; i<5; i++)
    {
      SVF10_Status_write();
      HAL_Delay(1);
      SVF10_Status_read();
      HAL_Delay(1);
  
      //SVF10_Memory_reset();
      // HAL_Delay(1);
      
      SVF10_Status_write();
      HAL_Delay(1);
      SVF10_Sleep_Mode();
      HAL_Delay(200);
    }
    
    SVF10_Memory_read_mem2(); 
    HAL_Delay(1);
    histo();
    for(uint16_t i=0; i<256; i++)
      histogram2[i] = histogram[i];
    
    SVF10_Memory_read_mem1(); 
    HAL_Delay(1);
    histo();
    for(uint16_t i=0; i<256; i++)
      histogram1[i] = histogram[i];
    
    temp4 = 0.0;
    
    for(uint16_t i=0; i<256; i++)
    {    
      temp = histogram1[i] - (1.0-50.0/384.0);
      temp2 = (1.0-50.0/384.0) - histogram2[i];
      if (temp>temp2)
        temp3 = temp2;
      else
        temp3 = temp;
      
      if ((temp>0) && (temp2>0) && (temp3>temp4))
      {
        temp4 = temp3;
        ADC_REF = (uint8_t) 230;
        INDEX = i;
      }   
    }
    
    SVF10_Memory_Read_Mode0();
    SVF10_Send_Image();
    HAL_Delay(1000);
     
#endif
    
    
  while (1)
  {
    
    #ifdef MATLAB_TEST
    C_INT++;
    #endif
    
    SVF10_Sreg_Set();
    HAL_Delay(1);
  
    SVF10_Sreg_Read_Creg_Set();
    HAL_Delay(1);

    SVF10_Sleep_Sens();
    HAL_Delay(200);
    
    SVF10_Memory_Read_Mode0();
    //SVF10_Memory_Read_Mode1();
    
    image_quality();
    
    moving_aver_by3();
    average = temp2;
    
    if (temp2 > ((float)(display_threshold)/100))
    {
      N_Capture_Frame++;
      //hist_eq();
      //moving_aver_by4();
      fir50_by5();
      //fir100_by5();
      //fir150_by7();
      //wiener_filter_by5();
      //moving_aver_by3();
      //gaussian_filter_by5();
      hist_eq();
      gaussian_filter_by3();
      //hist_eq();
      //moving_aver_by3();
      //fir100_by3();
   
#ifdef AUTO_SAVE
      if (N_Capture_Frame == AUTO_SAVE_FRAME)
      {
        SVF10_Send_Image();
      }
#else
      SVF10_Send_Image();
#endif      
    }
    else
    {
      N_Capture_Frame = 0;
    }
 
#ifndef AUTO_SAVE    
    #ifdef MATLAB_TEST
      HAL_UART_Transmit(&huart4, svf10_origine_buffer, 9412, 2000);
    #else
      SVF10_Send_Image();
    #endif     
#endif      
    
    if(receive_end_flag)
    {
      receive_end_flag = 0;
      if((uart_receive_frame[0] == 'R') && (uart_receive_frame[1] == 'E') && (uart_receive_frame[2] == 'S'))
      {
        for(uint8_t i=0; i<16 ; i++)
        {
           svf10_resister_frame [i] = uart_receive_frame[i+3];
        }
      }
      
      if((uart_receive_frame[0] == 'R') && (uart_receive_frame[1] == 'I') && (uart_receive_frame[2] == 'D'))
      {
         HAL_UART_Transmit(&huart4, chip_id_buffer, 6, 2000);
      }
      
      display_threshold = SVF_DISPLAY_THRESHOLD;
      
      SVF10_Sreg_Set();
      HAL_Delay(1);
  
      SVF10_Sreg_Read_Creg_Set();
      HAL_Delay(1);
 
//      SVF10_Memory_Read_Mode5();
//      HAL_Delay(1);
  
//      SVF10_Sreg_Set();
//      HAL_Delay(1);
   
      SVF10_Capture_Offset();
      HAL_Delay(200);
      
      
      while(!(HAL_OK == (HAL_UART_Receive_IT(&huart4, uart_receive_frame, 19))));
    }
      
  }
}

/* 2017.08.18 Add Function Strat----------------------------------------------*/

void SVF10_Capture_Offset(void)
{       
  uint8_t spi_tx_buffer[]={0x00,0x00,0x00};
  spi_tx_buffer[0] = (SVF_DYL2X << 7) | (SVF_CLK_SEL << 5) | (SVF_PERIOD << 2); 
  spi_tx_buffer[1] = (SVF_DLY2X_OSC << 3) | (SVF_S_GAIN << 1) | SVF_ADC_RSEL ;
  SVF10_CS_EN;
  HAL_Delay(1);
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

void SVF10_Chip_ID_Read(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x10,0x00};
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);  
  HAL_SPI_Receive(&hspi1,svf10_chip_id,8,10);
  SVF10_CS_DIS;
}

void SVF10_GPB_Set(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x00,0x00};
  
  spi_tx_buffer[0] = (SVF_GPB_REG << 2);
  spi_tx_buffer[1] = 0x20 | (SVF_GPB_REG >> 6);
    
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);  
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}


/* Memory Read Mode0 to MEM2-MEM1 read (compensated :default) */
void SVF10_Memory_Read_Mode0(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x30,0x00}; 
  
  spi_tx_buffer[0] = (SVF_MULTIPLYING << 2);
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode1 to To MISO after Digital amp */
void SVF10_Memory_Read_Mode1(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x34,0x00}; 
  
  spi_tx_buffer[0] = (SVF_MULTIPLYING << 2);
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode2 to no operation*/
void SVF10_Memory_Read_Mode2(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x36,0x00}; 
  
  spi_tx_buffer[0] = (SVF_MULTIPLYING << 2);
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode3 to MEM1 read (offset data) */
void SVF10_Memory_Read_Mode3(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x38,0x00}; 
  
  spi_tx_buffer[0] = (SVF_MULTIPLYING << 2);
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode4 to MEM2 read (not compensated) */
void SVF10_Memory_Read_Mode4(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x3c,0x00}; 
  
  spi_tx_buffer[0] = (SVF_MULTIPLYING << 2);
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode5 to MEM1, MEM2  reset */
void SVF10_Memory_Read_Mode5(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x3e,0x00}; 
  
  spi_tx_buffer[0] = (SVF_MULTIPLYING << 2);
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}
void SVF10_Sleep_Sens(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x00,0x00}; 
  
  spi_tx_buffer[0] = (SVF_ADC_REF << 2);
  spi_tx_buffer[1] = 0x40 | (SVF_PERIOD << 2) | (SVF_ADC_REF >> 6);
  
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
  
}

void SVF10_Sreg_Read_Creg_Set(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x00,0x00}; 
  
  spi_tx_buffer[0] = (SVF_DYL2X << 7) | (SVF_PERIOD << 2); 
  spi_tx_buffer[1] = 0x60 | (SVF_DLY2X_OSC << 3) | (SVF_S_GAIN << 1) | SVF_ADC_RSEL ;
    
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_status_buffer,5,10);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
  
}

void SVF10_Sreg_Set(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x00,0x00}; 
  
  spi_tx_buffer[0] = (SVF_SL_TIME << 7) | (SVF_NAVI << 6) | (SVF_SCK_UT << 5) | ((SVF_SLV_STU*2) <<2); 
  spi_tx_buffer[1] = 0x70 | (SVF_ISOL << 2) | (SVF_SL_TIME >> 1);
    
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;
  
  test_buffer[0] = spi_tx_buffer[0];
  test_buffer[1] = spi_tx_buffer[1];
}

/* 2017.08.18 Add Function End------------------------------------------------*/

/* 2017.08.18 Senvis Add Function Start---------------------------------------*/

void image_quality(void)
{
  origine_image_count = 4;
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      image_data[i][j] = svf10_origine_buffer[origine_image_count];
      origine_image_count++;
    }
    origine_image_count +=2;
  }
  
  temp=0.0;
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      temp=temp+image_data[i][j];
    }
  }
  
  temp = temp / 96.0 / 96.0;
  
  temp2 = 0.0;
   for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      temp2=temp2+(image_data[i][j]-temp)*(image_data[i][j]-temp);
    }
  } 
  temp2 = temp2 / 96.0 / 96.0;
}

void moving_aver_by2(void)
{

	int16_t i_pre, j_pre;
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			result_data[i][j] = 0;
		}
	}


	for (uint16_t i = 0; i<96; i++)
	{
		i_pre = i - 1;
		if (i_pre < 0)
			i_pre = 0;
		for (uint16_t j = 0; j<96; j++)
		{
			j_pre = j - 1;
			if (j_pre < 0)
				j_pre = 0;
			result_data[i][j] = 1.0 / 4.0 *(image_data[i_pre][j_pre] + image_data[i_pre][j] +
				image_data[i][j_pre] + image_data[i][j]);
		}
	}


	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

}


void moving_aver_by3(void)
{

  int16_t i_pre, j_pre, i_post, j_post;
  origine_image_count = 4;
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      image_data[i][j] = svf10_origine_buffer[origine_image_count];
      origine_image_count++;
    }
    origine_image_count +=2;
  }
  
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      result_data[i][j] = 0;
    }
  }
  
  
  for(uint16_t i=0; i<96; i++)
  {
	  i_pre = i - 1;
	  if (i_pre < 0)
		  i_pre = 0;
	  i_post = i + 1;
	  if (i_post > 95)
		  i_post = 95;
    for(uint16_t j=0; j<96; j++)
    {
		j_pre = j - 1;
		if (j_pre < 0)
			j_pre = 0;
		j_post = j + 1;
		if (j_post > 95)
			j_post = 95;
      result_data[i][j] = 1.0/9.0 *(image_data[i_pre][j_pre] + image_data[i_pre][j] + image_data[i_pre][j_post] +
                                image_data[i][j_pre] + image_data[i][j] + image_data[i][j_post] +
                                image_data[i_post][j_pre] + image_data[i_post][j] + image_data[i_post][j_post]);
    }
  }

   
  origine_image_count = 4;
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      svf10_origine_buffer[origine_image_count]= (uint8_t) result_data[i][j];
      origine_image_count++;
    }
    origine_image_count +=2;
  }
  
}

void moving_aver_by4(void)
{

	int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post;
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			result_data[i][j] = 0;
		}
	}


	for (uint16_t i = 0; i<96; i++)
	{
		i_pre2 = i - 2;
		if (i_pre2 < 0)
			i_pre2 = 0;
		i_pre = i - 1;
		if (i_pre < 0)
			i_pre = 0;
		i_post = i + 1;
		if (i_post > 95)
			i_post = 95;

		for (uint16_t j = 0; j<96; j++)
		{
			j_pre2 = j - 2;
			if (j_pre2 < 0)
				j_pre2 = 0;
			j_pre = j - 1;
			if (j_pre < 0)
				j_pre = 0;
			j_post = j + 1;
			if (j_post > 95)
				j_post = 95;

			result_data[i][j] = 1.0 / 16.0 *(image_data[i_pre2][j_pre2] + image_data[i_pre2][j_pre] + image_data[i_pre2][j] + image_data[i_pre2][j_post] + 
				image_data[i_pre][j_pre2] + image_data[i_pre][j_pre] + image_data[i_pre][j] + image_data[i_pre][j_post] +
				image_data[i][j_pre2] + image_data[i][j_pre] + image_data[i][j] + image_data[i][j_post] +
				image_data[i_post][j_pre2] + image_data[i_post][j_pre] + image_data[i_post][j] + image_data[i_post][j_post]);                      

		}
	}
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t) result_data[i][j];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

}

void moving_aver_by5(void)
{

	int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post, i_post2, j_post2;
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			result_data[i][j] = 0;
		}
	}


	for (uint16_t i = 0; i<96; i++)
	{
		i_pre2 = i - 2;
		if (i_pre2 < 0)
			i_pre2 = 0;
		i_pre = i - 1;
		if (i_pre < 0)
			i_pre = 0;
		i_post = i + 1;
		if (i_post > 95)
			i_post = 95;
		i_post2 = i + 2;
		if (i_post2 > 95)
			i_post2 = 95;

		for (uint16_t j = 0; j<96; j++)
		{
			j_pre2 = j - 2;
			if (j_pre2 < 0)
				j_pre2 = 0;
			j_pre = j - 1;
			if (j_pre < 0)
				j_pre = 0;
			j_post = j + 1;
			if (j_post > 95)
				j_post = 95;
			j_post2 = j + 2;
			if (j_post2 > 95)
				j_post2 = 95;

			result_data[i][j] = 1.0 / 25.0 *(image_data[i_pre2][j_pre2] + image_data[i_pre2][j_pre] + image_data[i_pre2][j] + image_data[i_pre2][j_post] + image_data[i_pre2][j_post2] +
				image_data[i_pre][j_pre2] + image_data[i_pre][j_pre] + image_data[i_pre][j] + image_data[i_pre][j_post] + image_data[i_pre][j_post2] +
				image_data[i][j_pre2] + image_data[i][j_pre] + image_data[i][j] + image_data[i][j_post] + image_data[i][j_post2] +
				image_data[i_post][j_pre2] + image_data[i_post][j_pre] + image_data[i_post][j] + image_data[i_post][j_post] + image_data[i_post][j_post2] +
				image_data[i_post2][j_pre2] + image_data[i_post2][j_pre] + image_data[i_post2][j] + image_data[i_post2][j_post] + image_data[i_post2][j_post2] );

		}
	}
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

}

void gaussian_filter_by3(void)
{
//	float factor[3][3], sum = 0;
	int16_t i_pre, j_pre, i_post, j_post;
	origine_image_count = 4;

//	sum = 0;
//	for (uint16_t i = 0; i < 3; i++)
//	{
//		for (uint16_t j = 1; j < 3; j++)
//		{
//			factor[i][j] = exp(-((i - 1.0)*(i - 1.0) + (j - 1.0)*(j - 1.0)) / (2.0*2.0));
//			sum = sum + factor[i][j];
//		}
//
//	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			result_data[i][j] = 0;
		}
	}


	for (uint16_t i = 0; i<96; i++)
	{
		i_pre = i - 1;
		if (i_pre < 0)
			i_pre = 0;
		i_post = i + 1;
		if (i_post > 95)
			i_post = 95;
		for (uint16_t j = 0; j<96; j++)
		{
			j_pre = j - 1;
			if (j_pre < 0)
				j_pre = 0;
			j_post = j + 1;
			if (j_post > 95)
				j_post = 95;
			//result_data[i][j] = 1.0 / sum *(factor[0][0]*image_data[i_pre][j_pre] + factor[0][1]*image_data[i_pre][j] + factor[0][2]*image_data[i_pre][j_post] +
			//	factor[1][0]*image_data[i][j_pre] + factor[1][1]*image_data[i][j] + factor[1][2]*image_data[i][j_post] +
			//	factor[2][0]*image_data[i_post][j_pre] + factor[2][1]*image_data[i_post][j] + factor[2][2]*image_data[i_post][j_post]);
                        result_data[i][j] = 1.0 / 16.0 *(1.0*image_data[i_pre][j_pre] + 2.0*image_data[i_pre][j] + 1.0*image_data[i_pre][j_post] +
				2.0*image_data[i][j_pre] + 4.0*image_data[i][j] + 2.0*image_data[i][j_post] +
				1.0*image_data[i_post][j_pre] + 2.0*image_data[i_post][j] + 1.0*image_data[i_post][j_post]);
		}
	}


	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

}

void gaussian_filter_by5(void)
{

	int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post, i_post2, j_post2;
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			result_data[i][j] = 0;
		}
	}


	for (uint16_t i = 0; i<96; i++)
	{
		i_pre2 = i - 2;
		if (i_pre2 < 0)
			i_pre2 = 0;
		i_pre = i - 1;
		if (i_pre < 0)
			i_pre = 0;
		i_post = i + 1;
		if (i_post > 95)
			i_post = 95;
		i_post2 = i + 2;
		if (i_post2 > 95)
			i_post2 = 95;

		for (uint16_t j = 0; j<96; j++)
		{
			j_pre2 = j - 2;
			if (j_pre2 < 0)
				j_pre2 = 0;
			j_pre = j - 1;
			if (j_pre < 0)
				j_pre = 0;
			j_post = j + 1;
			if (j_post > 95)
				j_post = 95;
			j_post2 = j + 2;
			if (j_post2 > 95)
				j_post2 = 95;

			result_data[i][j] = 1.0 / 273.0 *(1.0*image_data[i_pre2][j_pre2] + 4.0*image_data[i_pre2][j_pre] + 7.0*image_data[i_pre2][j] + 4.0*image_data[i_pre2][j_post] + 1.0*image_data[i_pre2][j_post2] +
				4.0*image_data[i_pre][j_pre2] + 16.0*image_data[i_pre][j_pre] + 26.0*image_data[i_pre][j] + 16.0*image_data[i_pre][j_post] + 4.0*image_data[i_pre][j_post2] +
				7.0*image_data[i][j_pre2] + 26.0*image_data[i][j_pre] + 41.0*image_data[i][j] + 26.0*image_data[i][j_post] + 7.0*image_data[i][j_post2] +
				4.0*image_data[i_post][j_pre2] + 16.0*image_data[i_post][j_pre] + 26.*image_data[i_post][j] + 16.0*image_data[i_post][j_post] + 4.0*image_data[i_post][j_post2] +
				1.0*image_data[i_post2][j_pre2] + 4.0*image_data[i_post2][j_pre] + 7.0*image_data[i_post2][j] + 4.0*image_data[i_post2][j_post] + 1.0*image_data[i_post2][j_post2] );

		}
	}
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

}

void fir50_by5(void)
{

	int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post, i_post2, j_post2;
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			result_data[i][j] = 0;
		}
	}


	for (uint16_t i = 0; i<96; i++)
	{
		i_pre2 = i - 2;
		if (i_pre2 < 0)
			i_pre2 = 0;
		i_pre = i - 1;
		if (i_pre < 0)
			i_pre = 0;
		i_post = i + 1;
		if (i_post > 95)
			i_post = 95;
		i_post2 = i + 2;
		if (i_post2 > 95)
			i_post2 = 95;

		for (uint16_t j = 0; j<96; j++)
		{
			j_pre2 = j - 2;
			if (j_pre2 < 0)
				j_pre2 = 0;
			j_pre = j - 1;
			if (j_pre < 0)
				j_pre = 0;
			j_post = j + 1;
			if (j_post > 95)
				j_post = 95;
			j_post2 = j + 2;
			if (j_post2 > 95)
				j_post2 = 95;

			result_data[i][j] = 1.0 / 248.0 *(7.0*image_data[i_pre2][j_pre2] + 8.0*image_data[i_pre2][j_pre] + 9.0*image_data[i_pre2][j] + 8.0*image_data[i_pre2][j_post] + 7.0*image_data[i_pre2][j_post2] +
				8.0*image_data[i_pre][j_pre2] + 11.0*image_data[i_pre][j_pre] + 14.0*image_data[i_pre][j] + 11.0*image_data[i_pre][j_post] + 8.0*image_data[i_pre][j_post2] +
				9.0*image_data[i][j_pre2] + 14.0*image_data[i][j_pre] + 20.0*image_data[i][j] + 14.0*image_data[i][j_post] + 9.0*image_data[i][j_post2] +
				8.0*image_data[i_post][j_pre2] + 11.0*image_data[i_post][j_pre] + 14.0*image_data[i_post][j] + 11.0*image_data[i_post][j_post] + 8.0*image_data[i_post][j_post2] +
				7.0*image_data[i_post2][j_pre2] + 8.0*image_data[i_post2][j_pre] + 9.0*image_data[i_post2][j] + 8.0*image_data[i_post2][j_post] + 7.0*image_data[i_post2][j_post2] );

		}
	}
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

}

void fir100_by3(void)
{

  int16_t i_pre, j_pre, i_post, j_post;
  origine_image_count = 4;
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      image_data[i][j] = svf10_origine_buffer[origine_image_count];
      origine_image_count++;
    }
    origine_image_count +=2;
  }
  
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      result_data[i][j] = 0;
    }
  }
  
  
  for(uint16_t i=0; i<96; i++)
  {
	  i_pre = i - 1;
	  if (i_pre < 0)
		  i_pre = 0;
	  i_post = i + 1;
	  if (i_post > 95)
		  i_post = 95;
    for(uint16_t j=0; j<96; j++)
    {
		j_pre = j - 1;
		if (j_pre < 0)
			j_pre = 0;
		j_post = j + 1;
		if (j_post > 95)
			j_post = 95;
      result_data[i][j] = 1.0 / 78.0 *(8.0*image_data[i_pre][j_pre] + 9.0*image_data[i_pre][j] + 8.0*image_data[i_pre][j_post] +
                                9.0*image_data[i][j_pre] + 10.0*image_data[i][j] + 9.0*image_data[i][j_post] +
                                8.0*image_data[i_post][j_pre] + 9.0*image_data[i_post][j] + 8.0*image_data[i_post][j_post]);
    }
  }
  
  origine_image_count = 4;
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      svf10_origine_buffer[origine_image_count]= (uint8_t) result_data[i][j];
      origine_image_count++;
    }
    origine_image_count +=2;
  }
  
}

void fir100_by5(void)
{

	int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post, i_post2, j_post2;
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			result_data[i][j] = 0;
		}
	}


	for (uint16_t i = 0; i<96; i++)
	{
		i_pre2 = i - 2;
		if (i_pre2 < 0)
			i_pre2 = 0;
		i_pre = i - 1;
		if (i_pre < 0)
			i_pre = 0;
		i_post = i + 1;
		if (i_post > 95)
			i_post = 95;
		i_post2 = i + 2;
		if (i_post2 > 95)
			i_post2 = 95;

		for (uint16_t j = 0; j<96; j++)
		{
			j_pre2 = j - 2;
			if (j_pre2 < 0)
				j_pre2 = 0;
			j_pre = j - 1;
			if (j_pre < 0)
				j_pre = 0;
			j_post = j + 1;
			if (j_post > 95)
				j_post = 95;
			j_post2 = j + 2;
			if (j_post2 > 95)
				j_post2 = 95;

			result_data[i][j] = 1.0 / 360.0 *(11.0*image_data[i_pre2][j_pre2] + 13.0*image_data[i_pre2][j_pre] + 14.0*image_data[i_pre2][j] + 13.0*image_data[i_pre2][j_post] + 11.0*image_data[i_pre2][j_post2] +
				13.0*image_data[i_pre][j_pre2] + 16.0*image_data[i_pre][j_pre] + 18.0*image_data[i_pre][j] + 16.0*image_data[i_pre][j_post] + 13.0*image_data[i_pre][j_post2] +
				14.0*image_data[i][j_pre2] + 18.0*image_data[i][j_pre] + 20.0*image_data[i][j] + 18.0*image_data[i][j_post] + 14.0*image_data[i][j_post2] +
				13.0*image_data[i_post][j_pre2] + 16.0*image_data[i_post][j_pre] + 18.0*image_data[i_post][j] + 16.0*image_data[i_post][j_post] + 13.0*image_data[i_post][j_post2] +
				11.0*image_data[i_post2][j_pre2] + 13.0*image_data[i_post2][j_pre] + 14.0*image_data[i_post2][j] + 13.0*image_data[i_post2][j_post] + 11.0*image_data[i_post2][j_post2] );

		}
	}
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

}

void fir150_by3(void)
{

  int16_t i_pre, j_pre, i_post, j_post;
  origine_image_count = 4;
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      image_data[i][j] = svf10_origine_buffer[origine_image_count];
      origine_image_count++;
    }
    origine_image_count +=2;
  }
  
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      result_data[i][j] = 0;
    }
  }
  
  
  for(uint16_t i=0; i<96; i++)
  {
	  i_pre = i - 1;
	  if (i_pre < 0)
		  i_pre = 0;
	  i_post = i + 1;
	  if (i_post > 95)
		  i_post = 95;
    for(uint16_t j=0; j<96; j++)
    {
		j_pre = j - 1;
		if (j_pre < 0)
			j_pre = 0;
		j_post = j + 1;
		if (j_post > 95)
			j_post = 95;
      result_data[i][j] = 1.0 / 168.0 *(18.0*image_data[i_pre][j_pre] + 19.0*image_data[i_pre][j] + 18.0*image_data[i_pre][j_post] +
                                19.0*image_data[i][j_pre] + 20.0*image_data[i][j] + 19.0*image_data[i][j_post] +
                                18.0*image_data[i_post][j_pre] + 19.0*image_data[i_post][j] + 18.0*image_data[i_post][j_post]);
    }
  }
  
  origine_image_count = 4;
  for(uint16_t i=0; i<96; i++)
  {
    for(uint16_t j=0; j<96; j++)
    {
      svf10_origine_buffer[origine_image_count]= (uint8_t) result_data[i][j];
      origine_image_count++;
    }
    origine_image_count +=2;
  }
  
}

void fir150_by5(void)
{

	int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post, i_post2, j_post2;
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			result_data[i][j] = 0;
		}
	}


	for (uint16_t i = 0; i<96; i++)
	{
		i_pre2 = i - 2;
		if (i_pre2 < 0)
			i_pre2 = 0;
		i_pre = i - 1;
		if (i_pre < 0)
			i_pre = 0;
		i_post = i + 1;
		if (i_post > 95)
			i_post = 95;
		i_post2 = i + 2;
		if (i_post2 > 95)
			i_post2 = 95;

		for (uint16_t j = 0; j<96; j++)
		{
			j_pre2 = j - 2;
			if (j_pre2 < 0)
				j_pre2 = 0;
			j_pre = j - 1;
			if (j_pre < 0)
				j_pre = 0;
			j_post = j + 1;
			if (j_post > 95)
				j_post = 95;
			j_post2 = j + 2;
			if (j_post2 > 95)
				j_post2 = 95;

			result_data[i][j] = 1.0 / 424.0 *(15.0*image_data[i_pre2][j_pre2] + 16.0*image_data[i_pre2][j_pre] + 17.0*image_data[i_pre2][j] + 16.0*image_data[i_pre2][j_post] + 15.0*image_data[i_pre2][j_post2] +
				16.0*image_data[i_pre][j_pre2] + 18.0*image_data[i_pre][j_pre] + 19.0*image_data[i_pre][j] + 18.0*image_data[i_pre][j_post] + 16.0*image_data[i_pre][j_post2] +
				17.0*image_data[i][j_pre2] + 19.0*image_data[i][j_pre] + 20.0*image_data[i][j] + 19.0*image_data[i][j_post] + 17.0*image_data[i][j_post2] +
				16.0*image_data[i_post][j_pre2] + 18.0*image_data[i_post][j_pre] + 19.0*image_data[i_post][j] + 18.0*image_data[i_post][j_post] + 16.0*image_data[i_post][j_post2] +
				15.0*image_data[i_post2][j_pre2] + 16.0*image_data[i_post2][j_pre] + 17.0*image_data[i_post2][j] + 16.0*image_data[i_post2][j_post] + 15.0*image_data[i_post2][j_post2] );

		}
	}
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

}

void fir150_by7(void)
{

	int16_t i_pre3, i_pre2, i_pre, j_pre3, j_pre2, j_pre, i_post, j_post, i_post2, i_post3, j_post2, j_post3;
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			result_data[i][j] = 0;
		}
	}


	for (uint16_t i = 0; i<96; i++)
	{
		i_pre3 = i - 3;
		if (i_pre3 < 0)
			i_pre3 = 0;
                i_pre2 = i - 2;
		if (i_pre2 < 0)
			i_pre2 = 0;
		i_pre = i - 1;
		if (i_pre < 0)
			i_pre = 0;
		i_post = i + 1;
		if (i_post > 95)
			i_post = 95;
		i_post2 = i + 2;
		if (i_post2 > 95)
			i_post2 = 95;
                i_post3 = i + 3;
		if (i_post3 > 95)
			i_post3 = 95;

		for (uint16_t j = 0; j<96; j++)
		{
			j_pre2 = j - 2;
			if (j_pre2 < 0)
				j_pre2 = 0;
			j_pre = j - 1;
			if (j_pre < 0)
				j_pre = 0;
			j_post = j + 1;
			if (j_post > 95)
				j_post = 95;
			j_post2 = j + 2;
			if (j_post2 > 95)
				j_post2 = 95;

			result_data[i][j] = 1.0 / 724.0 *(11.0*image_data[i_pre3][j_pre3] + 15.0*image_data[i_pre3][j_pre2] + 16.0*image_data[i_pre3][j_pre] + 17.0*image_data[i_pre3][j] + 16.0*image_data[i_pre3][j_post] + 15.0*image_data[i_pre3][j_post2] + 11.0*image_data[i_pre3][j_post3] +
                                12.0*image_data[i_pre2][j_pre3] + 15.0*image_data[i_pre2][j_pre2] + 16.0*image_data[i_pre2][j_pre] + 17.0*image_data[i_pre2][j] + 16.0*image_data[i_pre2][j_post] + 15.0*image_data[i_pre2][j_post2] + 12.0*image_data[i_pre2][j_post3] +
				13.0*image_data[i_pre][j_pre3] + 16.0*image_data[i_pre][j_pre2] + 18.0*image_data[i_pre][j_pre] + 19.0*image_data[i_pre][j] + 18.0*image_data[i_pre][j_post] + 16.0*image_data[i_pre][j_post2] + 13.0*image_data[i_pre][j_post3] +
				14.0*image_data[i][j_pre3] + 17.0*image_data[i][j_pre2] + 19.0*image_data[i][j_pre] + 20.0*image_data[i][j] + 19.0*image_data[i][j_post] + 17.0*image_data[i][j_post2] + 14.0*image_data[i][j_post3] +
				13.0*image_data[i_post][j_pre3] + 16.0*image_data[i_post][j_pre2] + 18.0*image_data[i_post][j_pre] + 19.0*image_data[i_post][j] + 18.0*image_data[i_post][j_post] + 16.0*image_data[i_post][j_post2] + 13.0*image_data[i_post][j_post3] +
				12.0*image_data[i_post2][j_pre3] + 15.0*image_data[i_post2][j_pre2] + 16.0*image_data[i_post2][j_pre] + 17.0*image_data[i_post2][j] + 16.0*image_data[i_post2][j_post] + 15.0*image_data[i_post2][j_post2] + 12.0*image_data[i_post2][j_post3] +
                                11.0*image_data[i_post3][j_pre3] + 15.0*image_data[i_post3][j_pre2] + 16.0*image_data[i_post3][j_pre] + 17.0*image_data[i_post3][j] + 16.0*image_data[i_post3][j_post] + 15.0*image_data[i_post3][j_post2] + 11.0*image_data[i_post3][j_post3] );

		}
	}
	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
			origine_image_count++;
		}
		origine_image_count += 2;
	}

}

void hist_eq(void)
{
	uint8_t gray;

	for (uint16_t i = 0; i < 256; i++)
		histogram[i] = 0;

	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			gray = image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
			histogram[gray] = histogram[gray] + 1;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i < 256; i++)
		histogram[i] = histogram[i] / 96.0 / 96.0 * 255.0;

	for (uint16_t i = 1; i < 256; i++)
		histogram[i] = histogram[i - 1] + histogram[i];

	for (uint16_t i = 1; i < 256; i++)
		histogram[i] = (uint8_t) histogram[i];

	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			svf10_origine_buffer[origine_image_count] = (uint8_t) histogram[(uint8_t) image_data[i][j]];
			origine_image_count++;
		}
		origine_image_count += 2;
	}
}

void histo(void)
{
	uint8_t gray;

	for (uint16_t i = 0; i < 256; i++)
		histogram[i] = 0;

	origine_image_count = 4;
	for (uint16_t i = 0; i<96; i++)
	{
		for (uint16_t j = 0; j<96; j++)
		{
			gray = image_data[i][j] = svf10_origine_buffer[origine_image_count];
			origine_image_count++;
			histogram[gray] = histogram[gray] + 1;
		}
		origine_image_count += 2;
	}

	for (uint16_t i = 0; i < 256; i++)
		histogram[i] = histogram[i] / 96.0 / 96.0;

	for (uint16_t i = 1; i < 256; i++)
		histogram[i] = histogram[i - 1] + histogram[i];
}

void SVF10_Send_Image_mark(void)
{
  origine_image_count = 4;
  recon_image_count = 1078;
      
  for(uint16_t i=0; i< svf10_y_pixel; i++)
  {
    for(uint16_t j = 0; j< svf10_x_pixel ; j++)
    {
      bitmap_header_data[recon_image_count] = 255 - bitmap_header_data[recon_image_count] ;
      origine_image_count++;
      recon_image_count++;
    }
    origine_image_count +=2;
  }
  
  HAL_UART_Transmit(&huart4,bitmap_header_data,svf10_header_count,2000);
}


void SVF10_Send_Image_BS(void)
{
  uint8_t tmpByte;
  uint8_t decMSB;
  
  origine_image_count = 4;
  recon_image_count = 1078;
  
    
  for(uint16_t i=0; i< svf10_y_pixel; i++)
  {
    for(uint16_t j = 0; j< svf10_x_pixel ; j++)
    {
      tmpByte = svf10_origine_buffer[origine_image_count];
      decMSB = tmpByte & 0x01;
      if (decMSB == 1) 
        bitmap_header_data[recon_image_count-1] = svf10_origine_buffer[origine_image_count-1] | 0x80;
      else
        bitmap_header_data[recon_image_count-1] = svf10_origine_buffer[origine_image_count-1];
      bitmap_header_data[recon_image_count] = tmpByte >> 1;
      origine_image_count++;
      recon_image_count++;
    }
    origine_image_count +=2;
  }
  
  HAL_UART_Transmit(&huart4,bitmap_header_data,svf10_header_count,2000);

}

/* 2017.08.18 Senvis Add Function End-----------------------------------------*/

void SVF10_Send_Image(void)
{
  origine_image_count = 4;
  recon_image_count = 1078;
  
   
  for(uint16_t i=0; i< svf10_y_pixel; i++)
  {
    for(uint16_t j = 0; j< svf10_x_pixel ; j++)
    {
      bitmap_header_data[recon_image_count] = 255 - svf10_origine_buffer[origine_image_count];
      origine_image_count++;
      recon_image_count++;
    }
    origine_image_count +=2;
  }
  
  HAL_UART_Transmit(&huart4,bitmap_header_data,svf10_header_count,2000);
}


 void SVF10_Power_On(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x1c,0x00};
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;
}


void SVF10_Memory_reset(void)
{
  uint8_t spi_tx_buffer[]={0x14,0x3e,0x00};

  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
}


void SVF10_Memory_write(void)
{
  uint8_t spi_tx_buffer[]={0x14,0x3e,0x00};

  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
}


void SVF10_Status_read(void)
{
  uint8_t spi_tx_buffer[]={0x10,0x6a,0x00};
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Receive(&hspi1,svf10_status_buffer,5,10);
  SVF10_CS_DIS;
}

void SVF10_Status_write(void)
{
  uint8_t spi_tx_buffer[]={0x20,0x76,0x00};

  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;

}

void SVF10_Sleep_Mode(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x51,0x00};

  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;

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
  //hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;  //   SPI 모드 설정 Mode 2
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
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
  //huart4.Init.BaudRate = 460800;
  //huart4.Init.BaudRate = 115200;
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
