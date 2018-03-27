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
#define SVF_SENSING_MODE     svf10_resister_frame[16]
#define SVF_IMAGE_FILTER     svf10_resister_frame[17]

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

#define SENSING_NORMAL       0
#define SENSING_WET          1
#define SENSING_DRY          2

#define NO_FILTER            0
#define WEINER_FILTER        1

#ifdef SVF10P
  
#define SVF10_X_PIXEL        96
#define SVF10_Y_PIXEL        96
#define SVF10_RECEIVE_COUNT  9413        // 5 + (96+2) * 96= 9413
#define SVF10_HEADER_COUNT   10296 
  
#endif
  
#ifdef SVF10P_R2
  
#define SVF10_X_PIXEL        96
#define SVF10_Y_PIXEL        96
#define SVF10_RECEIVE_COUNT  9511        // 5 + (96+2) * 97 = 9510
#define SVF10_HEADER_COUNT   10296 
  
#endif  
  
#ifdef SVF10G
  
#define SVF10_X_PIXEL        192
#define SVF10_Y_PIXEL        56
#define SVF10_RECEIVE_COUNT  10869       // 5 + (192+2) * 56 =10868
#define SVF10_HEADER_COUNT   11832
  
#endif

#define SVF10_DUMMY_COUNT      5

#define EP_OPTION_FLAT         0
#define EP_OPTION_LINEAR       1


/* UTILITY FUNCTION  ---------------------------------------------------------*/
void SVF10_Chip_ID_Read(uint8_t chip_id[]);
uint8_t Byte_Order_Convert(uint8_t input);
uint8_t Bit_Shift(uint8_t preByte, uint8_t nextByte, uint8_t n_shift);

void SVF10_Capture_Offset(void);
void SVF10_GPB_Set(void);
void SVF10_Memory_Read_Mode0(uint8_t origine_buffer[]);
void SVF10_Memory_Read_Mode1(uint8_t origine_buffer[]);
void SVF10_Memory_Read_Mode2(uint8_t origine_buffer[]);
void SVF10_Memory_Read_Mode3(uint8_t origine_buffer[]);
void SVF10_Memory_Read_Mode4(uint8_t origine_buffer[]);
void SVF10_Memory_Read_Mode5(void);
void SVF10_Sleep_Sens(void);
void SVF10_Sreg_Read_Creg_Set(uint8_t status_buffer[]);
void SVF10_Sreg_Set(void);
void SVF10_Sreg_Set_For_Memread(void);

void Chip_ID_Convert(uint8_t chip_id[]);
// void SVF10_Sreg_Set_Read(void);

void hist_eq(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL]);
void hist_ep(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], uint8_t origin_l1, uint8_t origin_l2, uint8_t target_l1, uint8_t target_l2, uint8_t option);
void histo(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL]);
void SVF10_Send_Image_mark(uint8_t origine_buffer[], uint8_t bitmap_header_data[]);
void SVF10_Send_Image_BS(uint8_t origine_buffer[], uint8_t bitmap_header_data[]);
void SVF10_Send_Image(uint8_t origine_buffer[], uint8_t bitmap_header_data[]);

void buffer_convert(float result_data[][SVF10_X_PIXEL], uint8_t origine_buffer[]);
void clean_result(float result_data[][SVF10_X_PIXEL]);
void clean_result2(float result_data2[][SVF10_X_PIXEL]);
void image_convert(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL]);

uint8_t Check_Bit(int chip_id[], uint8_t posi);
void Set_Bit(int chip_id[], uint8_t posi, uint8_t value);

void image_quality(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL]);
void uniform_image_ver(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float  result_data[][SVF10_X_PIXEL]);
void uniform_image_hor(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float  result_data[][SVF10_X_PIXEL]);

void moving_aver_by2(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL]);
void moving_aver_by3(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL]);
void moving_aver_by4(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL]);
void moving_aver_by5(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL]);
void gaussian_filter_by3(uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL]);
void wiener_filter(int16_t istep, uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL], float result_data2[][SVF10_X_PIXEL]);
void noise_filter(int16_t istep, uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL], float result_data[][SVF10_X_PIXEL]);
float finger_coverage(int16_t istep, uint8_t origine_buffer[], uint8_t image_data[][SVF10_X_PIXEL] );
