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

#define ISOL_10LINE          0
#define ISOL_15LINE          1
#define ISOL_20LINE          2
#define ISOL_25LINE          3

#define SL_DEEP              0
#define SL_50MS              1
#define SL_400MS             2
#define SL_1000MS            3

#define NAVI_OFF             0
#define NAVI_ON              1

#define SCK_UT_MODE_00       0
#define SCK_UT_MODE_01       1
#define SCK_UT_MODE_10       2
#define SCK_UT_MODE_11       3

#define SLV_SUT_CAP_O        0
#define SLV_SUT_CAP_N        1
#define SLV_SUT_SLEEP        2
#define SLV_SUT_MEM_R        3

//#define AUTO_SAVE
//#define AUTO_SAVE_FRAME      3
#define IMAGE_PROCESS

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart4;

#ifdef SVF10P
  
  uint16_t svf10_x_pixel = 96;
  uint16_t svf10_y_pixel = 96;
  uint16_t svf10_receive_count = 9413;  // 5 + (96+2) * 96= 9412
  uint16_t svf10_header_count = 10296;
  uint8_t svf10_origine_buffer[9413];
  uint8_t svf10_origine_offset[9413];
  
#endif
  
#ifdef SVF10P_R2
  
  uint16_t svf10_x_pixel = 96;
  uint16_t svf10_y_pixel = 96;
  uint16_t svf10_receive_count = 9511; // 5 + (96+2) * 97 = 9510
  uint16_t svf10_header_count = 10296;
  uint8_t svf10_origine_buffer[9511];
  uint8_t svf10_origine_offset[9511];
  
#endif  
  
#ifdef SVF10G
  
  uint16_t svf10_x_pixel = 192;
  uint16_t svf10_y_pixel = 56;
  uint16_t svf10_receive_count = 10869;  // 5 + (192+2) * 56 =10868
  uint16_t svf10_header_count = 11832;
  uint8_t svf10_origine_buffer[10869];
  uint8_t svf10_origine_offset[10869];
  
#endif
  

/* Private variables ---------------------------------------------------------*/

uint8_t svf10_chip_id[8];
uint8_t svf10_status_buffer[8];
uint16_t origine_image_count = 5;
uint16_t recon_image_count = 1078;


uint8_t uart_receive_frame[19];
uint8_t svf10_resister_frame[16];
uint8_t receive_end_flag = 0;
uint8_t receive_try_flag = 0;
uint8_t chip_id_buffer[8];
uint8_t first_do_flag = 1;

/* variables for Histogram Function ------------------------------------------*/

uint8_t image_data[96][96]; 
float  result_data[96][96];
float  result_data2[96][96];
double histogram[256];
double histogram1[256], histogram2[256];
double temp, temp2, temp3, temp4, temp5;
uint8_t INDEX;
double average,std_dev;
double l_aver[96];
int16_t         N_Capture_Frame;

uint16_t display_threshold = 100;

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
void SVF10_Sreg_Set_Read(void);
/* 2017.08.18 Add Function End------------------------------------------------*/

/* 2017.08.18 Senvis Add Function Start---------------------------------------*/
void image_quality(void);
void uniform_image_hor(void);
void uniform_image_ver(void);
void moving_aver_by2(void);
void moving_aver_by3(void);
void moving_aver_by4(void);
void moving_aver_by5(void);
void gaussian_filter_by3(void);
void wiener_filter(int16_t istep);
void noise_filter(int16_t istep);
void hist_eq(void);
void histo(void);
void SVF10_Send_Image_mark(void);
void SVF10_Send_Image_BS(void);
/* 2017.08.18 Senvis Add Function End-----------------------------------------*/


uint8_t test_buffer[2];

void image_convert()
{
//  origine_image_count = 4;
  origine_image_count = 5;
  
#ifdef SVF10P_R2
  for(uint16_t j=0; j<svf10_y_pixel; j++)
  {
    origine_image_count++;
  }
  origine_image_count +=2;
#endif
  for(uint16_t i=0; i<svf10_y_pixel; i++)
  {
    for(uint16_t j=0; j<svf10_x_pixel; j++)
    {
      image_data[i][j] = svf10_origine_buffer[origine_image_count];
      origine_image_count++;
    }
    origine_image_count +=2;
  }
}

void buffer_convert()
{
//  origine_image_count = 4;
  origine_image_count = 5;
  
#ifdef SVF10P_R2
 for(uint16_t j=0; j<svf10_y_pixel; j++)
  {
    origine_image_count++;
  }
  origine_image_count +=2;
#endif
 
  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {
      svf10_origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
      origine_image_count++;
    }
    origine_image_count += 2;
  }
}

void clean_result()
{
  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {
      result_data[i][j] = 0;
     }
  }  
}

void clean_result2()
{
  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {
      result_data2[i][j] = 0;
     }
  }  
}

int main(void)
{
//  #ifdef MATLAB_TEST
//    int C_INT =0;
//  #endif

  HAL_Init();

  SystemClock_Config();
  
  HAL_Delay(2000);
  
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  SVF10_CS_DIS;

  
//  chip_id_buffer[0] = 'S';
//  chip_id_buffer[1] = 'V';
//  chip_id_buffer[2] = 'F';
//  chip_id_buffer[3] = '2';
//  chip_id_buffer[4] = '0';
//  chip_id_buffer[5] = 'P';
//  chip_id_buffer[6] = 0xff;
//  chip_id_buffer[7] = 0xff;
  
  SVF_DLY2X_OSC = OSC_5MHZ;     // 1
  SVF_S_GAIN = GAIN_1X2;        // 01
  SVF_ADC_RSEL = RSEL_INTERAL;  // 0
  SVF_DYL2X = DYL2X_15NS;       // 1
  SVF_CLK_SEL = CK_SYS;         // 00
  SVF_PERIOD = PERIOD_100;      // 100
  SVF_ISOL = ISOL_10LINE;       // 00
  SVF_SL_TIME = SL_1000MS;      // 3
  SVF_NAVI = NAVI_OFF;
  SVF_SCK_UT = SCK_UT_MODE_00;
  SVF_SLV_STU = SLV_SUT_CAP_O;
  SVF_MULTIPLYING = 5;          // 
  SVF_ADC_REF = 5;              // 
  SVF_GPB_REG = 0xe0;
  
  display_threshold = 100;   // value/100
  
  HAL_UART_Receive_IT(&huart4, uart_receive_frame, 19);
   
  SVF10_Chip_ID_Read();
  HAL_Delay(1);
//  HAL_UART_Transmit(&huart4, svf10_chip_id, 8, 2000);
  
  //SVF10_Status_write();
//  SVF10_Sreg_Set();
//  HAL_Delay(1);
  
  //SVF10_Status_read();
//  SVF10_Sreg_Read_Creg_Set();
//  HAL_Delay(1);
 
  //SVF10_Memory_reset();
  SVF10_Memory_Read_Mode5();
  HAL_Delay(1);
  
  //SVF10_Status_write();
  SVF10_Sreg_Set();
  HAL_Delay(1);
  
//  SVF10_Sreg_Read_Creg_Set();
//  HAL_UART_Transmit(&huart4, svf10_status_buffer, 5, 2000);
  
//  HAL_Delay(200); 
//  SVF10_Capture_Offset();
//  HAL_Delay(200);
  
//  SVF10_Memory_Read_Mode3();
//  for(uint16_t i=0; i<svf10_receive_count; i++)
//    svf10_origine_offset[i] = svf10_origine_buffer[i];
  
//  SVF10_Memory_Read_Mode1();
//  HAL_UART_Transmit(&huart4, svf10_origine_buffer, svf10_receive_count, 2000);

//  SVF10_Sreg_Read_Creg_Set();
//  HAL_UART_Transmit(&huart4, svf10_status_buffer, 5, 2000);

/*  
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
*/
 
  while (1)
  {  
//    #ifdef MATLAB_TEST
//    C_INT++;
//    #endif
    

    SVF10_Sreg_Set();
    HAL_Delay(1);
  
 //   SVF10_Sreg_Read_Creg_Set();
 //   HAL_Delay(1);
    
//    SVF10_Sleep_Sens();
//    HAL_Delay(200);

    
// TEST   
    SVF10_Capture_Offset();
    HAL_Delay(200);
// TEST  
//    SVF10_Memory_Read_Mode0();
    
    if (first_do_flag)
    {
      first_do_flag = 0;
      
      SVF10_Chip_ID_Read();
      HAL_Delay(1);
//      HAL_UART_Transmit(&huart4, svf10_chip_id, 8, 2000);
      
//      SVF10_Sreg_Set();
//      HAL_Delay(1);
      
//      SVF10_Sreg_Read_Creg_Set();
//      HAL_Delay(1);  
      
//      SVF10_Memory_Read_Mode5();
//      HAL_Delay(1); 
      SVF10_Sreg_Set_Read();
      HAL_Delay(1);
      
      SVF10_Capture_Offset();
      HAL_Delay(200);

      SVF10_Memory_Read_Mode3();
      //SVF10_Memory_Read_Mode4();
      HAL_Delay(1);
      
      for(uint16_t i=0; i<svf10_receive_count; i++)
        svf10_origine_offset[i] = svf10_origine_buffer[i];
    }
    else
    {
//      SVF10_Chip_ID_Read();
//      HAL_Delay(1);

      
      SVF10_Sreg_Set_Read();
      HAL_Delay(1);
      
      SVF10_Memory_Read_Mode3();
      //SVF10_Memory_Read_Mode0();

      for(uint16_t i=0; i<svf10_receive_count; i++)
        if ( svf10_origine_buffer[i] <= svf10_origine_offset[i])
          svf10_origine_buffer[i] = svf10_origine_offset[i] - svf10_origine_buffer[i];
          //svf10_origine_buffer[i] = svf10_origine_offset[i];
      
        else
          svf10_origine_buffer[i] = svf10_origine_buffer[i] - svf10_origine_offset[i];
          //svf10_origine_buffer[i] = svf10_origine_offset[i] - svf10_origine_buffer[i];
          //svf10_origine_buffer[i] = svf10_origine_offset[i];
    }

    uniform_image_ver();
    uniform_image_hor();
    image_quality();
    
//    moving_aver_by3();
    average = temp;
    temp2 = sqrt(temp2);
    std_dev = temp2;
    
    if (temp2 >= ((float)(display_threshold)/100))
    {
      N_Capture_Frame++;
#ifdef IMAGE_PROCESS      
      //moving_aver_by4();
 //    wiener_filter(2)
      hist_eq();
      noise_filter(1);
//      wiener_filter(1);
//      gaussian_filter_by3();

#endif
      
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
      HAL_UART_Transmit(&huart4, svf10_origine_buffer, svf10_receive_count, 2000);
    #else
       SVF10_Send_Image();
    #endif     
#endif      

      
    if(receive_end_flag)
    {
      receive_end_flag = 0;
      receive_try_flag = 1;
      if((uart_receive_frame[0] == 'R') && (uart_receive_frame[1] == 'E') && (uart_receive_frame[2] == 'S'))
      {
        for(uint8_t i=0; i<16 ; i++)
        {
           svf10_resister_frame [i] = uart_receive_frame[i+3];
        }
      }
      
      if((uart_receive_frame[0] == 'R') && (uart_receive_frame[1] == 'I') && (uart_receive_frame[2] == 'D'))
      {
         HAL_UART_Transmit(&huart4, svf10_chip_id, 6, 2000);
      }
     
      display_threshold = SVF_DISPLAY_THRESHOLD;
     
//      SVF10_Sreg_Set();
//      HAL_Delay(1);
  
//      SVF10_Sreg_Read_Creg_Set();
//      HAL_Delay(1);
 
//      SVF10_Memory_Read_Mode5();
//      HAL_Delay(1);
  
//      SVF10_Sreg_Set();
//      HAL_Delay(1);
   
//      SVF10_Capture_Offset();
//      HAL_Delay(200);
      
      
      while(!(HAL_OK == (HAL_UART_Receive_IT(&huart4, uart_receive_frame, 19))));
    }
    HAL_Delay(100); 

  }
}

/* 2017.08.18 Add Function Strat----------------------------------------------*/
uint8_t Byte_Order_Convert(uint8_t input)
{
#ifdef SVF10P_R2
  uint8_t result=0;

  for(uint16_t i=0; i<8; i++)
  {
    if ( ( input & (0x01<<i) ) != 0)
    {
      result = result | ((0x80)>>i);
    }
  }
  return result;
#else
  return input;
#endif 
}

void SVF10_Capture_Offset(void)
{       
//  uint8_t spi_tx_buffer[]={0x90,0x0c,0x00};
  uint8_t spi_tx_buffer[]={0x90,0x0a,0x00};  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_DYL2X << 7) | (SVF_CLK_SEL << 5) | (SVF_PERIOD << 2) ); 
//  spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert( (SVF_DLY2X_OSC << 3) | (SVF_S_GAIN << 1) | SVF_ADC_RSEL );
//  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
//  HAL_Delay(1);
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

uint8_t Check_Bit(uint8_t posi)
{
  uint8_t int_posi, bit_posi;
  uint8_t result;
  
  int_posi = posi/8;
  bit_posi = posi%8;
  
  result = svf10_chip_id[int_posi] & ((0x01)<<bit_posi);
  if (result != 0) 
  {
    return 1;
  }
  else 
  {
    return 0;
  }
}

void Set_Bit(uint8_t posi, uint8_t value)
{
  uint8_t int_posi, bit_posi;
//  uint8_t result;
  
  int_posi = posi/8;
  bit_posi = posi%8;
  
  if (value!=0)
    chip_id_buffer[int_posi] = chip_id_buffer[int_posi] | ((0x01)<<bit_posi);
  else
    chip_id_buffer[int_posi] = chip_id_buffer[int_posi] & ~((0x01)<<bit_posi);
}

void Chip_ID_Convert()
{
  uint8_t tmpByte;
  uint8_t shift_bit;
  
  shift_bit = 3;
  
  for(uint16_t i=0; i< 6; i++)
  {
    tmpByte = svf10_chip_id[i];    
    svf10_chip_id[i] = (svf10_chip_id[i+1] >> shift_bit) | (tmpByte << (8 - shift_bit));
    
  }  
}

uint8_t Bit_Shift(uint8_t preByte, uint8_t nextByte, uint8_t n_shift)
{
  uint8_t tmpByte;
  
  tmpByte = preByte;    
  preByte = ((tmpByte << (8 - n_shift)) | (nextByte >> n_shift) );
  
  return preByte;
}


void SVF10_Chip_ID_Read(void)
{
  //uint8_t spi_tx_buffer[]={0x00,0x10,0x00};
  uint8_t spi_tx_buffer[]={0x00,0x10};
  spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
//  svf10_chip_id[0] = 0;
//  svf10_chip_id[1] = 'S';
//  svf10_chip_id[2] = 'V';
//  svf10_chip_id[3] = 'F';
//  svf10_chip_id[4] = '1';
//  svf10_chip_id[5] = '0';
//  svf10_chip_id[6] = 'P';
//  svf10_chip_id[7] = 0;
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,2,10);  
  HAL_SPI_Receive(&hspi1,svf10_chip_id,8,10);
  SVF10_CS_DIS;
  
  //Chip_ID_Convert();
  for(uint16_t i=0; i< 6; i++)  
    svf10_chip_id[i] = Bit_Shift(svf10_chip_id[i], svf10_chip_id[i+1], 3);
}

void SVF10_GPB_Set(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x00,0x00}; 
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_GPB_REG << 2) );
  spi_tx_buffer[1] = Byte_Order_Convert( 0x20 | (SVF_GPB_REG >> 6) );
    
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);  
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}


/* Memory Read Mode0 to MEM2-MEM1 read (compensated :default) */
void SVF10_Memory_Read_Mode0(void)
{
  //uint8_t spi_tx_buffer[]={0x00,0x30,0x00}; 
  uint8_t spi_tx_buffer[]={0x14,0x30};
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
  //spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  //spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
 // HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,2,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode1 to To MISO after Digital amp */
void SVF10_Memory_Read_Mode1(void)
{
//  uint8_t spi_tx_buffer[]={0x00,0x34,0x00}; 
  uint8_t spi_tx_buffer[]={0x14,0x34};
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,2,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode2 to no operation*/
void SVF10_Memory_Read_Mode2(void)
{
//  uint8_t spi_tx_buffer[]={0x00,0x36,0x00};
  uint8_t spi_tx_buffer[]={0x14,0x36}; 
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,2,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode3 to MEM1 read (offset data) */
void SVF10_Memory_Read_Mode3(void)
{
//  uint8_t spi_tx_buffer[]={0x00,0x38,0x00};
  uint8_t spi_tx_buffer[]={0x14,0x38};
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
//  spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert(  spi_tx_buffer[1] );
//  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,2,10); 
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
//  HAL_SPI_Receive(&hspi1,svf10_origine_offset,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode4 to MEM2 read (not compensated) */
void SVF10_Memory_Read_Mode4(void)
{
//  uint8_t spi_tx_buffer[]={0x00,0x3c,0x00};
  uint8_t spi_tx_buffer[]={0x14,0x3c};
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
  //spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
//  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,2,10);
  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode5 to MEM1, MEM2  reset */
void SVF10_Memory_Read_Mode5(void)
{
  uint8_t spi_tx_buffer[]={0x14,0x3e,0x00}; 
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
  //spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert(  spi_tx_buffer[1] );
  //spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
//  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,svf10_receive_count,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

void SVF10_Sleep_Sens(void)
{
//  uint8_t spi_tx_buffer[]={0x14,0x48,0x00}; 
  uint8_t spi_tx_buffer[]={0x14,0x50,0x00};
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_ADC_REF << 2) );
  //spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] =  Byte_Order_Convert( 0x40 | (SVF_PERIOD << 2) | (SVF_ADC_REF >> 6) );
  //spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
  
}

void SVF10_Sreg_Read_Creg_Set(void)
{
  //uint8_t spi_tx_buffer[]={0x00,0x00,0x00}; 
//  uint8_t spi_tx_buffer[]={0x90,0x6C};
  uint8_t spi_tx_buffer[]={0x90,0x6a};
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_DYL2X << 7) | (SVF_PERIOD << 2) );
  //spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert( 0x60 | (SVF_DLY2X_OSC << 3) | (SVF_S_GAIN << 1) | SVF_ADC_RSEL );
  //spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
    
  SVF10_CS_EN;
  //HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,2,10);
  HAL_SPI_Receive(&hspi1,svf10_status_buffer,5,10);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
  
}

void SVF10_Sreg_Set(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x70,0x00}; 
 
#ifdef SVF10P_R2  
    spi_tx_buffer[0] = Byte_Order_Convert( (SVF_SL_TIME << 7) | (SVF_NAVI << 6) | ((SVF_SCK_UT << 5) & 0x20) | ((SVF_SLV_STU*2) <<2) ); 
   //spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] ); 
  //spi_tx_buffer[1] = Byte_Order_Convert( 0x70 | ((SVF_ISOL) << 2) | (SVF_SCK_UT & 0x02) | (SVF_SL_TIME >> 1) );
   spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
#else 
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_SL_TIME << 7) | (SVF_NAVI << 6) | (SVF_SCK_UT << 5) | ((SVF_SLV_STU*2) <<2) ); 
  spi_tx_buffer[1] = Byte_Order_Convert( 0x70 | (SVF_ISOL << 2) | (SVF_SL_TIME >> 1) );
#endif
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;
  
  test_buffer[0] = spi_tx_buffer[0];
  test_buffer[1] = spi_tx_buffer[1];
}

void SVF10_Sreg_Set_Read(void)
{
  uint8_t spi_tx_buffer[]={0x00,0x70,0x00}; 
 
#ifdef SVF10P_R2  
//  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_SL_TIME << 7) | (SVF_NAVI << 6) | ((SVF_SCK_UT << 5) & 0x20) | ((SVF_SLV_STU*2) <<2) ); 
  spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] ); 
  //  spi_tx_buffer[1] = Byte_Order_Convert( 0x70 | ((SVF_ISOL) << 2) | (SVF_SCK_UT & 0x02) | (SVF_SL_TIME >> 1) );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
#else 
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_SL_TIME << 7) | (SVF_NAVI << 6) | (SVF_SCK_UT << 5) | ((SVF_SLV_STU*2) <<2) ); 
  spi_tx_buffer[1] = Byte_Order_Convert( 0x70 | (SVF_ISOL << 2) | (SVF_SL_TIME >> 1) );
#endif
  
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
  
  image_convert();
  
  temp=0.0;
  for(uint16_t i=0; i<svf10_y_pixel; i++)
  {
    for(uint16_t j=0; j<svf10_x_pixel; j++)
    {
      temp=temp+image_data[i][j];
    }
  }
  
  temp = temp / (double) svf10_x_pixel / (double) svf10_y_pixel;
  
  temp2 = 0.0;
   for(uint16_t i=0; i<svf10_y_pixel; i++)
  {
    for(uint16_t j=0; j<svf10_x_pixel; j++)
    {
      temp2=temp2+(image_data[i][j]-temp)*(image_data[i][j]-temp);
    }
  } 
  temp2 = temp2 / (double) svf10_x_pixel / (double) svf10_y_pixel;
}

void uniform_image_ver(void)
{
  int16_t i_pre, i_post;

  image_convert();

  clean_result();
  
  temp = 0.0;
  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {

    l_aver[i] = 0.0;
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {

      l_aver[i] = l_aver[i]+(double) image_data[i][j];
    }
    temp = temp + l_aver[i];
    l_aver[i] = l_aver[i] / (double) svf10_x_pixel; 
  }
  
  temp = temp / (double) svf10_x_pixel / (double) svf10_y_pixel;
  
  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {
    i_pre = i - 1;
    if (i_pre<0) i_pre = 0;
    i_post = i + 1;
    if (i_post>svf10_y_pixel-1) 
      i_post = svf10_y_pixel -1;
    
    temp2 = temp/( (l_aver[i_pre] + l_aver[i] + l_aver[i_post])/3.0);
    
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {

      result_data[i][j] = temp2*((double) image_data[i][j]);
    }
  }  

  buffer_convert();

}

void uniform_image_hor(void)
{
  int16_t j_pre, j_post;

  image_convert();

  clean_result();
  
  temp = 0.0;
  for (uint16_t j = 0; j<svf10_x_pixel; j++)
  {

    l_aver[j] = 0.0;
    for (uint16_t i = 0; i<svf10_y_pixel; i++)
    {

      l_aver[j] = l_aver[j]+(double) image_data[i][j];
    }
    temp = temp + l_aver[j];
    l_aver[j] = l_aver[j] / (double) svf10_y_pixel; 
  }
  
  temp = temp / (double) svf10_x_pixel / (double) svf10_y_pixel;
  
  for (uint16_t j = 0; j<svf10_x_pixel; j++)
  {
    j_pre = j - 1;
    if (j_pre<0) j_pre = 0;
    j_post = j + 1;
    if (j_post > svf10_x_pixel-1) 
      j_post = svf10_x_pixel -1;
    
    temp2 = temp/( (l_aver[j_pre] + l_aver[j] + l_aver[j_post])/3.0);
    
    for (uint16_t i = 0; i<svf10_y_pixel; i++)
    {

      result_data[i][j] = temp2*((double) image_data[i][j]);
    }
  }  

  buffer_convert();

}

void moving_aver_by2(void)
{

  int16_t i_pre, j_pre;
  image_convert();

  clean_result();

  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {
      j_pre = j - 1;
      if (j_pre < 0)
        j_pre = 0;
      result_data[i][j] = 1.0 / 4.0 *(image_data[i_pre][j_pre] + image_data[i_pre][j] +
                                      image_data[i][j_pre] + image_data[i][j]);
    }
  }

  buffer_convert();

}


void moving_aver_by3(void)
{

  int16_t i_pre, j_pre, i_post, j_post;
  image_convert();
  
  clean_result();
  
  for(uint16_t i=0; i<svf10_y_pixel; i++)
  {
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    i_post = i + 1;
    if (i_post > svf10_y_pixel-1)
      i_post = svf10_y_pixel-1;
    for(uint16_t j=0; j<svf10_x_pixel; j++)
    {
      j_pre = j - 1;
      if (j_pre < 0)
        j_pre = 0;
      j_post = j + 1;
      if (j_post > svf10_x_pixel-1)
        j_post = svf10_x_pixel-1;
      result_data[i][j] = 1.0/9.0 *(image_data[i_pre][j_pre] + image_data[i_pre][j] + image_data[i_pre][j_post] +
                                image_data[i][j_pre] + image_data[i][j] + image_data[i][j_post] +
                                image_data[i_post][j_pre] + image_data[i_post][j] + image_data[i_post][j_post]);
    }
  }

  buffer_convert();
  
}

void moving_aver_by4(void)
{

  int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post;
  image_convert();

  clean_result();

  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {
    i_pre2 = i - 2;
    if (i_pre2 < 0)
      i_pre2 = 0;
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    i_post = i + 1;
    if (i_post > svf10_y_pixel-1)
      i_post = svf10_y_pixel-1;

    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {
      j_pre2 = j - 2;
      if (j_pre2 < 0)
        j_pre2 = 0;
      j_pre = j - 1;
      if (j_pre < 0)
        j_pre = 0;
      j_post = j + 1;
      if (j_post > svf10_x_pixel-1)
        j_post = svf10_x_pixel-1;

      result_data[i][j] = 1.0 / 16.0 *(image_data[i_pre2][j_pre2] + image_data[i_pre2][j_pre] + image_data[i_pre2][j] + image_data[i_pre2][j_post] + 
				image_data[i_pre][j_pre2] + image_data[i_pre][j_pre] + image_data[i_pre][j] + image_data[i_pre][j_post] +
				image_data[i][j_pre2] + image_data[i][j_pre] + image_data[i][j] + image_data[i][j_post] +
				image_data[i_post][j_pre2] + image_data[i_post][j_pre] + image_data[i_post][j] + image_data[i_post][j_post]);

    }
  }

  buffer_convert();
  
}

void moving_aver_by5(void)
{

  int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post, i_post2, j_post2;

  image_convert();
  
  clean_result();
  
  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {
    i_pre2 = i - 2;
    if (i_pre2 < 0)
      i_pre2 = 0;
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    i_post = i + 1;
    if (i_post > svf10_y_pixel-1)
      i_post = svf10_y_pixel-1;
    i_post2 = i + 2;
    if (i_post2 > svf10_y_pixel-1)
      i_post2 = svf10_y_pixel-1;

    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {
      j_pre2 = j - 2;
      if (j_pre2 < 0)
        j_pre2 = 0;
      j_pre = j - 1;
      if (j_pre < 0)
	j_pre = 0;
      j_post = j + 1;
      if (j_post > svf10_x_pixel-1)
	j_post = svf10_x_pixel-1;
      j_post2 = j + 2;
      if (j_post2 > svf10_x_pixel-1)
        j_post2 = svf10_x_pixel-1;

      result_data[i][j] = 1.0 / 25.0 *(image_data[i_pre2][j_pre2] + image_data[i_pre2][j_pre] + image_data[i_pre2][j] + image_data[i_pre2][j_post] + image_data[i_pre2][j_post2] +
                                       image_data[i_pre][j_pre2] + image_data[i_pre][j_pre] + image_data[i_pre][j] + image_data[i_pre][j_post] + image_data[i_pre][j_post2] +
                                         image_data[i][j_pre2] + image_data[i][j_pre] + image_data[i][j] + image_data[i][j_post] + image_data[i][j_post2] +
                                           image_data[i_post][j_pre2] + image_data[i_post][j_pre] + image_data[i_post][j] + image_data[i_post][j_post] + image_data[i_post][j_post2] +
                                             image_data[i_post2][j_pre2] + image_data[i_post2][j_pre] + image_data[i_post2][j] + image_data[i_post2][j_post] + image_data[i_post2][j_post2] );
      
    }
  }

  buffer_convert();
}

void gaussian_filter_by3(void)
{
  float factor[3][3], sum = 0;
  int16_t i_pre, j_pre, i_post, j_post;
  sum = 0;
  for (uint16_t i = 0; i < 3; i++)
  {
    for (uint16_t j = 1; j < 3; j++)
    {
      factor[i][j] = exp(-((i - 1.0)*(i - 1.0) + (j - 1.0)*(j - 1.0)) / (2.0*2.0));
      sum = sum + factor[i][j];
    }
  }

  image_convert();
  
  clean_result();
  
  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    i_post = i + 1;
    if (i_post > svf10_y_pixel-1)
      i_post = svf10_y_pixel-1;
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {
      j_pre = j - 1;
      if (j_pre < 0)
        j_pre = 0;
      j_post = j + 1;
      if (j_post > svf10_x_pixel-1)
        j_post = svf10_x_pixel-1;
      result_data[i][j] = 1.0 / sum *(factor[0][0]*image_data[i_pre][j_pre] + factor[0][1]*image_data[i_pre][j] + factor[0][2]*image_data[i_pre][j_post] +
				factor[1][0]*image_data[i][j_pre] + factor[1][1]*image_data[i][j] + factor[1][2]*image_data[i][j_post] +
				factor[2][0]*image_data[i_post][j_pre] + factor[2][1]*image_data[i_post][j] + factor[2][2]*image_data[i_post][j_post]);
    }
  }


  buffer_convert();

}

void wiener_filter(int16_t istep)
{
  float sum, sum2, noise;
  int16_t i_pre, j_pre, i_post, j_post, i_index, j_index;

  image_convert();

  clean_result();
    
  noise = 0.0;
  
  for(uint16_t i=0; i<svf10_y_pixel; i++)
  {
    i_pre = i - istep;
    i_post = i + istep;
    for(uint16_t j=0; j<svf10_x_pixel; j++)
    {
      j_pre = j - istep;
      j_post = j + istep;
      sum = 0;
      sum2 = 0;
      for(int16_t ii=i_pre; ii<=i_post; ii++)
      {
        if (ii<0)
          i_index = 0;
        else if (ii>svf10_y_pixel-1)
          i_index = svf10_y_pixel-1;
        else
          i_index = ii;
        for(int16_t jj=j_pre; jj<=j_post; jj++)
        {
          if (jj<0)
            j_index = 0;
          else if (jj>svf10_x_pixel-1)
            j_index = svf10_x_pixel-1;
          else
            j_index = jj;
            
          sum = sum + image_data[i_index][j_index];
          sum2 = sum2 + image_data[i_index][j_index]*image_data[i_index][j_index];
        }
      }
      temp = sum/((double) (1+2*istep)*(1+2*istep));
      result_data[i][j] = temp;
      temp2 = sum2/((double) (1+2*istep)*(1+2*istep)) - temp*temp;
      noise = noise + temp;
    }
  }

   noise = noise / (96.0 * 96.0);
  
  for(uint16_t i=0; i<svf10_y_pixel; i++)
  {
    for(uint16_t j=0; j<svf10_x_pixel; j++)
    {
      temp = image_data[i][j] - result_data[i][j];
      temp2 = result_data2[i][j] - noise;
      if (temp2<0)
        temp2 = 0;
      if (result_data2[i][j]>noise)
        temp3 = result_data2[i][j];
      else
        temp3 = noise;
      
      temp = temp / temp3;
      temp = temp * temp2;
      temp = temp + result_data[i][j];
      
      result_data[i][j] = temp;
    }
  }  

  buffer_convert();
  
}

void noise_filter(int16_t istep)
{
  float sum, sum2, sum_e, sum2_e;
  int16_t i_pre, j_pre, i_post, j_post, i_index, j_index;

  image_convert();

  clean_result();
      
  for(uint16_t i=0; i<svf10_y_pixel; i++)
  {
    i_pre = i - istep;
    i_post = i + istep;
    for(uint16_t j=0; j<svf10_x_pixel; j++)
    {
      j_pre = j - istep;
      j_post = j + istep;
      sum = 0.0;
      sum2 = 0.0;
      sum_e = 0.0;
      sum2_e = 0.0;

      for(int16_t ii=i_pre; ii<=i_post; ii++)
      {
        if (ii<0)
          i_index = 0;
        else if (ii>svf10_y_pixel-1)
          i_index = svf10_y_pixel-1;
        else
          i_index = ii;
        for(int16_t jj=j_pre; jj<=j_post; jj++)
        {
          if (jj<0)
            j_index = 0;
          else if (jj>svf10_x_pixel-1)
            j_index = svf10_x_pixel-1;
          else
            j_index = jj;
          
          if (ii==i && jj==j) {
          }
          else{
            sum_e = sum_e + image_data[i_index][j_index];
            sum2_e = sum2_e + image_data[i_index][j_index]*image_data[i_index][j_index];          
          }
          sum = sum + image_data[i_index][j_index];
          sum2 = sum2 + image_data[i_index][j_index]*image_data[i_index][j_index];  
        }
      }
      temp = sum/((double) (1+2*istep)*(1+2*istep));
      temp2 = sum2/((double) (1+2*istep)*(1+2*istep)) - temp*temp;
      
      temp3 = sum_e/((double) ((1+2*istep)*(1+2*istep)-1));
      temp4 = sum2_e/((double) ((1+2*istep)*(1+2*istep)-1)) - temp3*temp3;
      
      temp5 = temp4/temp2;    //
      
      result_data[i][j] = image_data[i][j]*temp5 + temp3*(1-temp5);
    }
  }
  
  buffer_convert();
  
}

void hist_eq(void)
{
  uint8_t gray;

  for (uint16_t i = 0; i < 256; i++)
    histogram[i] = 0;

  image_convert();
  
  for(uint16_t i=0; i<svf10_y_pixel;i++)
  {
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {
      gray = image_data[i][j];
      histogram[gray] = histogram[gray] + 1;
    }
  }
    
  for (uint16_t i = 0; i < 256; i++)
    histogram[i] = histogram[i] / (double) svf10_x_pixel / (double) svf10_y_pixel * 255.0;

  for (uint16_t i = 1; i < 256; i++)
    histogram[i] = histogram[i - 1] + histogram[i];

  for (uint16_t i = 1; i < 256; i++)
    histogram[i] = (uint8_t) histogram[i];

  origine_image_count = 5;
  
#ifdef SVF10P_R2
 for(uint16_t j=0; j<svf10_y_pixel; j++)
  {
    origine_image_count++;
  }
  origine_image_count +=2;
#endif  
 
  for (uint16_t i = 0; i<svf10_y_pixel; i++)
  {
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
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

  image_convert();
  
  for(uint16_t i=0; i<svf10_y_pixel;i++)
  {
    for (uint16_t j = 0; j<svf10_x_pixel; j++)
    {
      gray = image_data[i][j];
      histogram[gray] = histogram[gray] + 1;
    }
  }
  
  for (uint16_t i = 0; i < 256; i++)
    histogram[i] = histogram[i] / 96.0 / 96.0;

  for (uint16_t i = 1; i < 256; i++)
    histogram[i] = histogram[i - 1] + histogram[i];
}

void SVF10_Send_Image_mark(void)
{
  //origine_image_count = 4;
  origine_image_count = 5;
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
//  uint8_t tmpByte;
//  uint8_t decMSB;
  
  //origine_image_count = 4;
  origine_image_count = 5;
  recon_image_count = 1078;
  
    
  for(uint16_t i=0; i< svf10_y_pixel; i++)
  {
    for(uint16_t j = 0; j< svf10_x_pixel ; j++)
    {
     /* tmpByte = svf10_origine_buffer[origine_image_count];
      decMSB = tmpByte & 0x01;
      if (decMSB == 1) 
        bitmap_header_data[recon_image_count-1] = svf10_origine_buffer[origine_image_count-1] | 0x80;
      else
        bitmap_header_data[recon_image_count-1] = svf10_origine_buffer[origine_image_count-1];
      bitmap_header_data[recon_image_count] = tmpByte >> 1;
      origine_image_count++;
      recon_image_count++;
      */
     bitmap_header_data[recon_image_count] = Bit_Shift( svf10_origine_buffer[recon_image_count], svf10_origine_buffer[recon_image_count+1], 1);
    }
    origine_image_count +=2;
  }
  
  HAL_UART_Transmit(&huart4,bitmap_header_data,svf10_header_count,2000);

}

/* 2017.08.18 Senvis Add Function End-----------------------------------------*/

void SVF10_Send_Image(void)
{
  //origine_image_count = 4;
  origine_image_count = 5;
  recon_image_count = 1078;

  #ifdef SVF10P_R2
    for(uint16_t j=0; j<svf10_y_pixel; j++)
    {
      origine_image_count++;
    }
    origine_image_count +=2;
  #endif  
   
  for(uint16_t i=0; i< svf10_y_pixel; i++)
  {
    for(uint16_t j = 0; j< svf10_x_pixel ; j++)
    {
      bitmap_header_data[recon_image_count] = 255 - svf10_origine_buffer[origine_image_count];
//      bitmap_header_data[recon_image_count] = 255 - 30* svf10_origine_buffer[origine_image_count];
      origine_image_count++;
      recon_image_count++;
    }
    origine_image_count +=2;
  }
  
  HAL_UART_Transmit(&huart4,bitmap_header_data,svf10_header_count,2000);
//  HAL_UART_Transmit(&huart4,svf10_origine_buffer,svf10_receive_count,2000);
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
