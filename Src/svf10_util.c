#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "math.h"
#include "svf10_util.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart4;

uint8_t uart_receive_frame[19];
uint8_t svf10_resister_frame[16];
uint16_t svf10_display_threshold = 100;

/* Private variables ---------------------------------------------------------*/

//uint16_t origine_image_count = 5;
//uint16_t recon_image_count = 1078;
float svf10_average, svf10_std_dev;
float svf10_histogram[256];
float svf10_histogram2[256];
float svf10_l_aver[SVF10_X_PIXEL];
//uint8_t test_buffer[2];

/* UTILITY FUNCTION  ---------------------------------------------------------*/
// FUNC: SVF10_CHIP_ID_READ
// INPUT: 
// OUTPUT: svf10_chip_id
void SVF10_Chip_ID_Read(uint8_t chip_id[])
{
  //uint8_t spi_tx_buffer[]={0x00,0x10,0x00};
  uint8_t spi_tx_buffer[]={0x00,0x10};
  spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  
  HAL_SPI_Transmit(&hspi1, spi_tx_buffer, 2, 10);  
  HAL_SPI_Receive(&hspi1, chip_id, 8, 10);
  SVF10_CS_DIS;
  
  //Chip_ID_Convert();
  for(uint16_t i=0; i< 6; i++)  
    chip_id[i] = Bit_Shift( chip_id[i], chip_id[i+1], 3);
}

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

uint8_t Bit_Shift(uint8_t preByte, uint8_t nextByte, uint8_t n_shift)
{
  uint8_t tmpByte;
  
  tmpByte = preByte;    
  preByte = ((tmpByte << (8 - n_shift)) | (nextByte >> n_shift) );
  
  return preByte;
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
void SVF10_Memory_Read_Mode0(uint8_t origine_buffer[])
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
  HAL_SPI_Receive(&hspi1, origine_buffer, SVF10_RECEIVE_COUNT, 500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode1 to To MISO after Digital amp */
void SVF10_Memory_Read_Mode1(uint8_t origine_buffer[])
{
//  uint8_t spi_tx_buffer[]={0x00,0x34,0x00}; 
  uint8_t spi_tx_buffer[]={0x14,0x34};
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1, spi_tx_buffer, 2, 10);
  HAL_SPI_Receive(&hspi1, origine_buffer, SVF10_RECEIVE_COUNT, 500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode2 to no operation*/
void SVF10_Memory_Read_Mode2(uint8_t origine_buffer[])
{
//  uint8_t spi_tx_buffer[]={0x00,0x36,0x00};
  uint8_t spi_tx_buffer[]={0x14,0x36}; 
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1, spi_tx_buffer, 2, 10);
  HAL_SPI_Receive(&hspi1, origine_buffer, SVF10_RECEIVE_COUNT, 500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode3 to MEM1 read (offset data) */
void SVF10_Memory_Read_Mode3(uint8_t origine_buffer[])
{
//  uint8_t spi_tx_buffer[]={0x00,0x38,0x00};
  uint8_t spi_tx_buffer[]={0x14,0x38};
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
//  spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert(  spi_tx_buffer[1] );
//  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1, spi_tx_buffer, 2, 10); 
  HAL_SPI_Receive(&hspi1, origine_buffer, SVF10_RECEIVE_COUNT, 500);
//  HAL_SPI_Receive(&hspi1,svf10_origine_offset,SVF10_RECEIVE_COUNT,500);
  SVF10_CS_DIS;
  
  //test_buffer[0] = spi_tx_buffer[0];
  //test_buffer[1] = spi_tx_buffer[1];
}

/* Memory Read Mode4 to MEM2 read (not compensated) */
void SVF10_Memory_Read_Mode4(uint8_t origine_buffer[])
{
//  uint8_t spi_tx_buffer[]={0x00,0x3c,0x00};
  uint8_t spi_tx_buffer[]={0x14,0x3c};
  
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_MULTIPLYING << 2) );
  //spi_tx_buffer[0] = Byte_Order_Convert( spi_tx_buffer[0] );
  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
//  spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
  
  SVF10_CS_EN;
//  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  HAL_SPI_Transmit(&hspi1, spi_tx_buffer, 2, 10);
  HAL_SPI_Receive(&hspi1, origine_buffer, SVF10_RECEIVE_COUNT, 500);
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
//  HAL_SPI_Receive(&hspi1,svf10_origine_buffer,SVF10_RECEIVE_COUNT,500);
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

void SVF10_Sreg_Read_Creg_Set(uint8_t status_buffer[])
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
  HAL_SPI_Transmit(&hspi1, spi_tx_buffer, 2, 10);
  HAL_SPI_Receive(&hspi1, status_buffer, 5, 10);
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
//  spi_tx_buffer[1] = Byte_Order_Convert( 0x70 | ((SVF_ISOL) << 2) | (SVF_SCK_UT & 0x02) | (SVF_SL_TIME >> 1) );
   spi_tx_buffer[1] = Byte_Order_Convert( spi_tx_buffer[1] );
#else 
  spi_tx_buffer[0] = Byte_Order_Convert( (SVF_SL_TIME << 7) | (SVF_NAVI << 6) | (SVF_SCK_UT << 5) | ((SVF_SLV_STU*2) <<2) ); 
  spi_tx_buffer[1] = Byte_Order_Convert( 0x70 | (SVF_ISOL << 2) | (SVF_SL_TIME >> 1) );
#endif
  
  SVF10_CS_EN;
  HAL_SPI_Transmit(&hspi1,spi_tx_buffer,3,10);
  SVF10_CS_DIS;
  
//  test_buffer[0] = spi_tx_buffer[0];
//  test_buffer[1] = spi_tx_buffer[1];
}

void Chip_ID_Convert(uint8_t chip_id[])
{
  uint8_t tmpByte;
  uint8_t shift_bit;
  
  shift_bit = 3;
  
  for(uint16_t i=0; i< 6; i++)
  {
    tmpByte = chip_id[i];    
    chip_id[i] = (chip_id[i+1] >> shift_bit) | (tmpByte << (8 - shift_bit));
  }  
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
  
//  test_buffer[0] = spi_tx_buffer[0];
//  test_buffer[1] = spi_tx_buffer[1];
}


// FUNC: HIST_EQ
// INPUT: svf10_origine_buffer
// OUTPUT: svf10_origine buffer
void hist_eq(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL])
{
  uint8_t gray;

  for (uint16_t i = 0; i < 256; i++)
    svf10_histogram[i] = 0;

  image_convert(origine_buffer, image_data); 
  
  for(uint16_t i=0; i<SVF10_Y_PIXEL;i++)
  {
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      gray = image_data[i][j];
      svf10_histogram[gray] = svf10_histogram[gray] + 1;
    }
  }
    
  for (uint16_t i = 0; i < 256; i++)
    svf10_histogram[i] = svf10_histogram[i] / (double) SVF10_X_PIXEL / (double) SVF10_Y_PIXEL * 255.0;

  for (uint16_t i = 1; i < 256; i++)
    svf10_histogram[i] = svf10_histogram[i - 1] + svf10_histogram[i];

  for (uint16_t i = 1; i < 256; i++)
    svf10_histogram[i] = (uint8_t) svf10_histogram[i];

  uint16_t origine_image_count = SVF10_DUMMY_COUNT;
  
#ifdef SVF10P_R2
 for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
  {
    origine_image_count++;
  }
  origine_image_count +=2;
#endif  
 
  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      origine_buffer[origine_image_count] = (uint8_t) svf10_histogram[(uint8_t) image_data[i][j]];
      origine_image_count++;
    }
    origine_image_count += 2;
  }

}

// FUNC: HIST_EP
// INPUT: svf10_origine_buffer, level_1, level_2
// OUTPUT: svf10_origine buffer
void hist_ep(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], uint8_t origin_l1, uint8_t origin_l2, uint8_t target_l1, uint8_t target_l2, uint8_t option)
{
  uint8_t gray;

  for (uint16_t i = 0; i < 256; i++)
    svf10_histogram[i] = 0;

  image_convert(origine_buffer, image_data); 
  
  for(uint16_t i=0; i<SVF10_Y_PIXEL;i++)
  {
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      gray = image_data[i][j];
      svf10_histogram[gray] = svf10_histogram[gray] + 1;
    }
  }
    
  for (uint16_t i = 0; i < 256; i++)
    svf10_histogram[i] = svf10_histogram[i] / (double) SVF10_X_PIXEL / (double) SVF10_Y_PIXEL * 255.0;

  for (uint16_t i = 1; i < 256; i++)
    svf10_histogram[i] = svf10_histogram[i - 1] + svf10_histogram[i];

  for (uint16_t i = 0; i < 256; i++)
  {
    svf10_histogram2[i] = ((target_l2 - target_l1)/(svf10_histogram[origin_l2] - svf10_histogram[origin_l1])*(svf10_histogram[i] - svf10_histogram[origin_l1]) + target_l1);
  }
   
  for (uint16_t i = 0; i < 256; i++)
  {
    if (svf10_histogram2[i] <= 0)
    {
      svf10_histogram2[i] = 0;
    }
    else if (svf10_histogram2[i] >= 255)
    {
      svf10_histogram2[i] = 255;
    }
  }

  if (option == EP_OPTION_FLAT) 
  {
    for (uint16_t i = 0; i < 256; i++)
    {
      if (i <= origin_l1)
      {
        svf10_histogram2[i] = (uint8_t) target_l1;
      }
      else if (i >= origin_l2)
      {
        svf10_histogram2[i] = (uint8_t) target_l2;
      }
    }
  }
  
  for (uint16_t i = 0; i < 256; i++)
  {
    svf10_histogram2[i] = (uint8_t) svf10_histogram2[i];
  }
  
  uint16_t origine_image_count = SVF10_DUMMY_COUNT;
  
#ifdef SVF10P_R2
 for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
  {
    origine_image_count++;
  }
  origine_image_count +=2;
#endif  
 
  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      origine_buffer[origine_image_count] = (uint8_t) svf10_histogram2[(uint8_t) image_data[i][j]];
      origine_image_count++;
    }
    origine_image_count += 2;
  }

}

void histo(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL])
{
  uint8_t gray;

  for (uint16_t i = 0; i < 256; i++)
    svf10_histogram[i] = 0;

  image_convert(origine_buffer, image_data); 
  
  for(uint16_t i=0; i<SVF10_Y_PIXEL;i++)
  {
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      gray = image_data[i][j];
      svf10_histogram[gray] = svf10_histogram[gray] + 1;
    }
  }
  
  for (uint16_t i = 0; i < 256; i++)
    svf10_histogram[i] = svf10_histogram[i] / (double) SVF10_X_PIXEL / (double) SVF10_Y_PIXEL;

  for (uint16_t i = 1; i < 256; i++)
    svf10_histogram[i] = svf10_histogram[i - 1] + svf10_histogram[i];
}

void SVF10_Send_Image_mark(uint8_t origine_buffer[], uint8_t bitmap_header_data[])
{
  //origine_image_count = 4;
  uint16_t origine_image_count = SVF10_DUMMY_COUNT;
  uint16_t recon_image_count = 1078;
      
  for(uint16_t i=0; i< SVF10_Y_PIXEL; i++)
  {
    for(uint16_t j = 0; j< SVF10_X_PIXEL ; j++)
    {
      bitmap_header_data[recon_image_count] = 255 - bitmap_header_data[recon_image_count] ;
      origine_image_count++;
      recon_image_count++;
    }
    origine_image_count +=2;
  }
  
  HAL_UART_Transmit(&huart4,bitmap_header_data,SVF10_HEADER_COUNT,2000);
}


void SVF10_Send_Image_BS(uint8_t origine_buffer[], uint8_t bitmap_header_data[])
{
//  uint8_t tmpByte;
//  uint8_t decMSB;
  
  //origine_image_count = 4;
  uint16_t origine_image_count = SVF10_DUMMY_COUNT;
  uint16_t recon_image_count = 1078;
  
    
  for(uint16_t i=0; i< SVF10_Y_PIXEL; i++)
  {
    for(uint16_t j = 0; j< SVF10_X_PIXEL ; j++)
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
     bitmap_header_data[recon_image_count] = Bit_Shift( origine_buffer[recon_image_count], origine_buffer[recon_image_count+1], 1);
    }
    origine_image_count +=2;
  }
  
  HAL_UART_Transmit(&huart4,bitmap_header_data,SVF10_HEADER_COUNT,2000);

}

/* 2017.08.18 Senvis Add Function End-----------------------------------------*/

void SVF10_Send_Image(uint8_t origine_buffer[], uint8_t bitmap_header_data[])
{
  //origine_image_count = 4;
  uint16_t origine_image_count = SVF10_DUMMY_COUNT;
  uint16_t recon_image_count = 1078;

  #ifdef SVF10P_R2
    for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
    {
      origine_image_count++;
    }
    origine_image_count +=2;
  #endif  
   
  for(uint16_t i=0; i< SVF10_Y_PIXEL; i++)
  {
    for(uint16_t j = 0; j< SVF10_X_PIXEL ; j++)
    {
      bitmap_header_data[recon_image_count] = 255 - origine_buffer[origine_image_count];
//      bitmap_header_data[recon_image_count] = 255 - 30* svf10_origine_buffer[origine_image_count];
      origine_image_count++;
      recon_image_count++;
    }
    origine_image_count +=2;
  }
  
  HAL_UART_Transmit(&huart4,bitmap_header_data,SVF10_HEADER_COUNT,2000);
//  HAL_UART_Transmit(&huart4,svf10_origine_buffer,SVF10_RECEIVE_COUNT,2000);
}

void image_convert(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL])
{
//  origine_image_count = 4;
  uint16_t  origine_image_count = SVF10_DUMMY_COUNT;
  
#ifdef SVF10P_R2
  for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
  {
    origine_image_count++;
  }
  origine_image_count +=2;
#endif
  for(uint16_t i=0; i<SVF10_Y_PIXEL; i++)
  {
    for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
    {
      image_data[i][j] = origine_buffer[origine_image_count];
      origine_image_count++;
    }
    origine_image_count +=2;
  }
}

void buffer_convert(float result_data[][SVF10_X_PIXEL], uint8_t origine_buffer[])
{
//  origine_image_count = 4;
  uint16_t  origine_image_count = SVF10_DUMMY_COUNT;
  
#ifdef SVF10P_R2
 for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
  {
    origine_image_count++;
  }
  origine_image_count +=2;
#endif
 
  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      origine_buffer[origine_image_count] = (uint8_t)result_data[i][j];
      origine_image_count++;
    }
    origine_image_count += 2;
  }
}

void clean_result(float result_data[][SVF10_X_PIXEL])
{
  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      result_data[i][j] = 0.0;
     }
  }  
}

void clean_result2(float result_data2[][SVF10_X_PIXEL])
{
  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      result_data2[i][j] = 0;
     }
  }  
}

uint8_t Check_Bit(int chip_id[], uint8_t posi)
{
  uint8_t int_posi, bit_posi;
  uint8_t result;
  
  int_posi = posi/8;
  bit_posi = posi%8;
  
  result = chip_id[int_posi] & ((0x01)<<bit_posi);
  if (result != 0) 
  {
    return 1;
  }
  else 
  {
    return 0;
  }
}

void Set_Bit(int chip_id[], uint8_t posi, uint8_t value)
{
  uint8_t int_posi, bit_posi;
//  uint8_t result;
  
  int_posi = posi/8;
  bit_posi = posi%8;
  
  if (value!=0)
    chip_id[int_posi] = chip_id[int_posi] | ((0x01)<<bit_posi);
  else
    chip_id[int_posi] = chip_id[int_posi] & ~((0x01)<<bit_posi);
}

void image_quality(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL])
{
  float temp, temp2;
  
  image_convert(origine_buffer, image_data);
  
  temp=0.0;
  for(uint16_t i=0; i<SVF10_Y_PIXEL; i++)
  {
    for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
    {
      temp=temp + image_data[i][j];
    }
  }
  
  temp = temp / (double) SVF10_X_PIXEL / (double) SVF10_Y_PIXEL;
  
  temp2 = 0.0;
   for(uint16_t i=0; i<SVF10_Y_PIXEL; i++)
  {
    for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
    {
      temp2 = temp2 + (image_data[i][j]-temp)*(image_data[i][j]-temp);
    }
  } 
  temp2 = temp2 / (double) SVF10_X_PIXEL / (double) SVF10_Y_PIXEL;
  svf10_average = temp;
  svf10_std_dev = sqrt(temp2);
}

void uniform_image_ver(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float  result_data[][SVF10_X_PIXEL])
{
  int16_t i_pre, i_post;
  float temp, temp2;

  image_convert(origine_buffer, image_data);

  clean_result(result_data);
  
  temp = 0.0;
  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    svf10_l_aver[i] = 0.0;
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {

      svf10_l_aver[i] = svf10_l_aver[i]+(double) image_data[i][j];
    }
    temp = temp + svf10_l_aver[i];
    svf10_l_aver[i] = svf10_l_aver[i] / (double) SVF10_X_PIXEL; 
  }
  
  temp = temp / (double) SVF10_X_PIXEL / (double) SVF10_X_PIXEL;
  
  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    i_pre = i - 1;
    if (i_pre<0) i_pre = 0;
    i_post = i + 1;
    if (i_post>SVF10_Y_PIXEL-1) 
      i_post = SVF10_Y_PIXEL -1;
    
    temp2 = temp/( (svf10_l_aver[i_pre] + svf10_l_aver[i] + svf10_l_aver[i_post])/3.0);
    
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      result_data[i][j] = temp2*((double) image_data[i][j]);
    }
  }  

  buffer_convert(result_data, origine_buffer);
}

void uniform_image_hor(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float  result_data[][SVF10_X_PIXEL])
{
  float temp, temp2;
  int16_t j_pre, j_post;

  image_convert(origine_buffer, image_data);

  clean_result(result_data);
  
  temp = 0.0;
  for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
  {
    svf10_l_aver[j] = 0.0;
    for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
    {
      svf10_l_aver[j] = svf10_l_aver[j]+(double) image_data[i][j];
    }
    temp = temp + svf10_l_aver[j];
    svf10_l_aver[j] = svf10_l_aver[j] / (double) SVF10_Y_PIXEL; 
  }
  
  temp = temp / (double) SVF10_X_PIXEL / (double) SVF10_Y_PIXEL;
  
  for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
  {
    j_pre = j - 1;
    if (j_pre<0) j_pre = 0;
    j_post = j + 1;
    if (j_post > SVF10_X_PIXEL-1) 
      j_post = SVF10_X_PIXEL -1;
    
    temp2 = temp/( (svf10_l_aver[j_pre] + svf10_l_aver[j] + svf10_l_aver[j_post])/3.0);
    
    for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
    {

      result_data[i][j] = temp2*((double) image_data[i][j]);
    }
  }  

  buffer_convert(result_data, origine_buffer);

}

void moving_aver_by2(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL])
{

  int16_t i_pre, j_pre;
  
  image_convert(origine_buffer, image_data);
  
  clean_result(result_data);

  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      j_pre = j - 1;
      if (j_pre < 0)
        j_pre = 0;
      result_data[i][j] = 1.0 / 4.0 *(image_data[i_pre][j_pre] + image_data[i_pre][j] +
                                      image_data[i][j_pre] + image_data[i][j]);
    }
  }

  buffer_convert(result_data, origine_buffer);

}


void moving_aver_by3(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL])
{

  int16_t i_pre, j_pre, i_post, j_post;
  
  image_convert(origine_buffer, image_data);
  
  clean_result(result_data);
  
  for(uint16_t i=0; i<SVF10_Y_PIXEL; i++)
  {
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    i_post = i + 1;
    if (i_post > SVF10_Y_PIXEL-1)
      i_post = SVF10_Y_PIXEL-1;
    for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
    {
      j_pre = j - 1;
      if (j_pre < 0)
        j_pre = 0;
      j_post = j + 1;
      if (j_post > SVF10_X_PIXEL-1)
        j_post = SVF10_X_PIXEL-1;
      result_data[i][j] = 1.0/9.0 *(image_data[i_pre][j_pre] + image_data[i_pre][j] + image_data[i_pre][j_post] +
                                    image_data[i][j_pre] + image_data[i][j] + image_data[i][j_post] +
                                    image_data[i_post][j_pre] + image_data[i_post][j] + image_data[i_post][j_post]);
    }
  }

  buffer_convert(result_data, origine_buffer);
  
}

void moving_aver_by4(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL])
{

  int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post;
  
  image_convert(origine_buffer, image_data);

  clean_result(result_data);

  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    i_pre2 = i - 2;
    if (i_pre2 < 0)
      i_pre2 = 0;
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    i_post = i + 1;
    if (i_post > SVF10_Y_PIXEL-1)
      i_post = SVF10_Y_PIXEL-1;

    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      j_pre2 = j - 2;
      if (j_pre2 < 0)
        j_pre2 = 0;
      j_pre = j - 1;
      if (j_pre < 0)
        j_pre = 0;
      j_post = j + 1;
      if (j_post > SVF10_X_PIXEL-1)
        j_post = SVF10_X_PIXEL-1;

      result_data[i][j] = 1.0 / 16.0 *(image_data[i_pre2][j_pre2] + image_data[i_pre2][j_pre] + image_data[i_pre2][j] + image_data[i_pre2][j_post] + 
				       image_data[i_pre][j_pre2] + image_data[i_pre][j_pre] + image_data[i_pre][j] + image_data[i_pre][j_post] +
				       image_data[i][j_pre2] + image_data[i][j_pre] + image_data[i][j] + image_data[i][j_post] +
				       image_data[i_post][j_pre2] + image_data[i_post][j_pre] + image_data[i_post][j] + image_data[i_post][j_post]);

    }
  }

   buffer_convert(result_data, origine_buffer);
  
}

void moving_aver_by5(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL])
{

  int16_t i_pre2, i_pre, j_pre2, j_pre, i_post, j_post, i_post2, j_post2;

  image_convert(origine_buffer, image_data);
  
  clean_result(result_data);
  
  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    i_pre2 = i - 2;
    if (i_pre2 < 0)
      i_pre2 = 0;
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    i_post = i + 1;
    if (i_post > SVF10_Y_PIXEL-1)
      i_post = SVF10_Y_PIXEL-1;
    i_post2 = i + 2;
    if (i_post2 > SVF10_Y_PIXEL-1)
      i_post2 = SVF10_Y_PIXEL-1;

    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      j_pre2 = j - 2;
      if (j_pre2 < 0)
        j_pre2 = 0;
      j_pre = j - 1;
      if (j_pre < 0)
	j_pre = 0;
      j_post = j + 1;
      if (j_post > SVF10_X_PIXEL-1)
	j_post = SVF10_X_PIXEL-1;
      j_post2 = j + 2;
      if (j_post2 > SVF10_X_PIXEL-1)
        j_post2 = SVF10_X_PIXEL-1;

      result_data[i][j] = 1.0 / 25.0 *(image_data[i_pre2][j_pre2] + image_data[i_pre2][j_pre] + image_data[i_pre2][j] + image_data[i_pre2][j_post] + image_data[i_pre2][j_post2] +
                                       image_data[i_pre][j_pre2] + image_data[i_pre][j_pre] + image_data[i_pre][j] + image_data[i_pre][j_post] + image_data[i_pre][j_post2] +
                                       image_data[i][j_pre2] + image_data[i][j_pre] + image_data[i][j] + image_data[i][j_post] + image_data[i][j_post2] +
                                       image_data[i_post][j_pre2] + image_data[i_post][j_pre] + image_data[i_post][j] + image_data[i_post][j_post] + image_data[i_post][j_post2] +
                                       image_data[i_post2][j_pre2] + image_data[i_post2][j_pre] + image_data[i_post2][j] + image_data[i_post2][j_post] + image_data[i_post2][j_post2] );
      
    }
  }

  buffer_convert(result_data, origine_buffer);
}

// FUNC: GAUSSIAN_FILTER_BY3
// INTPUT: svf10_origine_buffer
// OUTPUT: svf10_origine_buffer
void gaussian_filter_by3(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL])
{
  float factor[3][3], sum = 0;
  int16_t i_pre, j_pre, i_post, j_post;
  
  sum = 0;
  for (uint16_t i = 0; i < 3; i++)
  {
    for (uint16_t j = 0; j < 3; j++)
    {
      factor[i][j] = exp(-((i - 1.0)*(i - 1.0) + (j - 1.0)*(j - 1.0)) / (2.0*2.0));
      sum = sum + factor[i][j];
    }
  }

  image_convert(origine_buffer, image_data);
  
  clean_result(result_data);
  
  for (uint16_t i = 0; i<SVF10_Y_PIXEL; i++)
  {
    i_pre = i - 1;
    if (i_pre < 0)
      i_pre = 0;
    i_post = i + 1;
    if (i_post > SVF10_Y_PIXEL-1)
      i_post = SVF10_Y_PIXEL-1;
    for (uint16_t j = 0; j<SVF10_X_PIXEL; j++)
    {
      j_pre = j - 1;
      if (j_pre < 0)
        j_pre = 0;
      j_post = j + 1;
      if (j_post > SVF10_X_PIXEL-1)
        j_post = SVF10_X_PIXEL-1;
      result_data[i][j] = 1.0 / sum *(factor[0][0]*image_data[i_pre][j_pre] + factor[0][1]*image_data[i_pre][j] + factor[0][2]*image_data[i_pre][j_post] +
				      factor[1][0]*image_data[i][j_pre] + factor[1][1]*image_data[i][j] + factor[1][2]*image_data[i][j_post] +
				      factor[2][0]*image_data[i_post][j_pre] + factor[2][1]*image_data[i_post][j] + factor[2][2]*image_data[i_post][j_post]);
    }
  }


  buffer_convert(result_data, origine_buffer);

}

// FUNC: WIENER_FILTER
// INPUT: istep, svf10_origine_buffer
// OUTPUT: svf10_origine_buffer
void wiener_filter(int16_t istep, uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL], float result_data2[][SVF10_X_PIXEL])
{
  float temp, temp2, temp3; 
  float sum, sum2, noise;
  int16_t i_pre, j_pre, i_post, j_post, i_index, j_index;

  image_convert(origine_buffer, image_data);

  clean_result(result_data);
    
  noise = 0.0;
  
  for(uint16_t i=0; i<SVF10_Y_PIXEL; i++)
  {
    i_pre = i - istep;
    i_post = i + istep;
    for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
    {
      j_pre = j - istep;
      j_post = j + istep;
      sum = 0;
      sum2 = 0;
      for(int16_t ii=i_pre; ii<=i_post; ii++)
      {
        if (ii<0)
          i_index = 0;
        else if (ii>SVF10_Y_PIXEL-1)
          i_index = SVF10_Y_PIXEL-1;
        else
          i_index = ii;
        for(int16_t jj=j_pre; jj<=j_post; jj++)
        {
          if (jj<0)
            j_index = 0;
          else if (jj>SVF10_X_PIXEL-1)
            j_index = SVF10_X_PIXEL-1;
          else
            j_index = jj;
            
          sum = sum + image_data[i_index][j_index];
          sum2 = sum2 + image_data[i_index][j_index]*image_data[i_index][j_index];
        }
      }
      temp = sum/((double) (1 + 2*istep)*(1 + 2*istep));
      result_data[i][j] = temp;
      temp2 = sum2/((double) (1 + 2*istep)*(1 + 2*istep)) - temp*temp;
      noise = noise + temp;
    }
  }

   noise = noise / (double) SVF10_X_PIXEL / (double) SVF10_Y_PIXEL;
  
  for(uint16_t i=0; i<SVF10_Y_PIXEL; i++)
  {
    for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
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

  buffer_convert(result_data, origine_buffer);
  
}

// FUNC: NOISE_FILTER
// INPUT: istep, svf10_origine_buffer
// OUTPUT: svf10_origine_buffer
void noise_filter(int16_t istep, uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL])
{
  float temp, temp2, temp3, temp4, temp5, temp6;
  float sum, sum2, sum_e, sum2_e;
  int16_t i_pre, j_pre, i_post, j_post, i_index, j_index;
  uint8_t decision_option = 0; // 0 : equation 1: discrete

  image_convert(origine_buffer, image_data); 

  clean_result(result_data);
      
  for(uint16_t i=0; i<SVF10_Y_PIXEL; i++)
  {
    i_pre = i - istep;
    i_post = i + istep;
    for(uint16_t j=0; j<SVF10_X_PIXEL; j++)
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
        else if (ii>SVF10_Y_PIXEL-1)
          i_index = SVF10_Y_PIXEL-1;
        else
          i_index = ii;
        for(int16_t jj=j_pre; jj<=j_post; jj++)
        {
          if (jj<0)
            j_index = 0;
          else if (jj>SVF10_X_PIXEL-1)
            j_index = SVF10_X_PIXEL-1;
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
      
      temp5 = (0.00000001+temp4)/(0.00000001+temp2);    //
      
      
      temp6 = image_data[i][j]*temp5 + temp3*(1-temp5);
      
      if (decision_option == 0)
      {
        if (temp6<=0) 
        {
          temp6 = 0.0;
        }
        else if (temp6 >= 255)
        {
          temp6 = 255.0;
        }
      
        result_data[i][j] = (uint8_t) temp6;
      }
      else 
      {
         if ( temp2 <= (1.2*temp4))
          result_data[i][j] = image_data[i][j];
         else if (temp2 <= 1.4*temp4)
          result_data[i][j] = (uint8_t) (image_data[i][j] + temp3)/2;
         else
          result_data[i][j] = (uint8_t) temp3;
      } 
    }
  }
  
  buffer_convert(result_data, origine_buffer);
}


// FUNC: FINGER_COVERAGE
// INPUT: istep, svf10_origine_buffer
// OUTPUT: return 
float finger_coverage(int16_t istep, uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL] )
{
  float temp, temp2, sum, sum2;
  int16_t i_pre, j_pre, i_post, j_post, i_index, j_index, count;

  image_convert(origine_buffer, image_data); 
  
  count = 0;
        
  for(uint16_t i=0; i<SVF10_Y_PIXEL; i=i+istep)
  {
    i_pre = i;
    i_post = i + istep;
    for(uint16_t j=0; j<SVF10_X_PIXEL; j=j+istep)
    {
      j_pre = j;
      j_post = j + istep;
      sum = 0.0;
      sum2 = 0.0;

      for(int16_t ii=i_pre; ii<=i_post; ii++)
      {
        if (ii<0)
          i_index = 0;
        else if (ii>SVF10_Y_PIXEL-1)
          i_index = SVF10_Y_PIXEL-1;
        else
          i_index = ii;
        for(int16_t jj=j_pre; jj<=j_post; jj++)
        {
          if (jj<0)
            j_index = 0;
          else if (jj>SVF10_X_PIXEL-1)
            j_index = SVF10_X_PIXEL-1;
          else
            j_index = jj;
          
          sum = sum + image_data[i_index][j_index];
          sum2 = sum2 + image_data[i_index][j_index]*image_data[i_index][j_index];  
        }
      }
      temp = sum/((double) (1+2*istep)*(1+2*istep));
      temp2 = sum2/((double) (1+2*istep)*(1+2*istep)) - temp*temp;
      
      temp2 = sqrt(temp2);
      if (temp2 > ((float)(svf10_display_threshold)/100)) 
        count = count + 1;
    }
  }
  
  return( (float) count / (SVF10_X_PIXEL/istep) / (SVF10_X_PIXEL/istep));
  
}

