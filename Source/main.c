

//-- Includes -----------------------------------------------------------------
#include "typedefs.h"              // type definitions 
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"   			//define type_ GPIO_InitTypeDef
#include "stm32f10x_usart.h" 				// define type_ USART_InitStruct
#include "stm32f10x_conf.h"
#include "stm32f10x_rcc.h" 					// define function_ RCC_APB2PeriphClockCmd
#include "typedefs.h"               // type definitions 
#include <stdio.h> 
#include <math.h>



//-- Typedefs -----------------------------------------------------------------
 
typedef enum{
  ACK  = 0,
  NACK = 1,
}etI2cAck;

typedef enum{
  NO_ERROR       = 0x00, // no error
  ACK_ERROR      = 0x01, // no acknowledgment error
  CHECKSUM_ERROR = 0x02, // checksum mismatch error
  TIMEOUT_ERROR  = 0x04, // timeout error
  PARM_ERROR     = 0x80, // parameter out of range error
}etError;


//-- Defines ------------------------------------------------------------------
// Generator polynomial for CRC
#define POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

//=============================================================================
// IO-Pins                            /* -- adapt the defines for your uC -- */



// SDA on port B, bit 14
#define SDA_LOW()  (GPIOB->BSRR = 0x40000000) // set SDA to low
#define SDA_OPEN() (GPIOB->BSRR = 0x00004000) // set SDA to open-drain
#define SDA_READ   (GPIOB->IDR  & 0x4000)     // read SDA

// SCL on port B, bit 13              /* -- adapt the defines for your uC -- */
#define SCL_LOW()  (GPIOB->BSRR = 0x20000000) // set SCL to low
#define SCL_OPEN() (GPIOB->BSRR = 0x00002000) // set SCL to open-drain
#define SCL_READ   (GPIOB->IDR  & 0x2000)     // read SCL



//-- Global variables ---------------------------------------------------------
static u8t i2cAddress = 0x69;// I2C Address
																// Address: 0x44 = Sensor on EvalBoard connector
																//          0x45 = Sensor on EvalBoard


//----------------------- System initialization, enable the ports, the pins,set the voltage-----------------------
static void EvalBoardPower_Init(void);
void SystemInit(void);
void DelayMicroSeconds(u32t nbrOfUs);
static void sps30_Init(void);
 
static etError sps30_read_measurement(ft* mass_PM1, ft* mass_PM2_5, ft* mass_PM4, ft* mass_PM10, ft *N_PM0_5, ft *N_PM1_0, ft *N_PM2_5, ft *N_PM4, ft *N_PM10);
static etError sps30_StartMeasurement(void);
static etError Data_Ready_Flag(ft *flag)  ;
static etError sps30_StartWriteAccess(void);
  
static etError sps30_Read2BytesAndCrc(u16t* data, etI2cAck finaleAckNack, u8t timeout);  
static u8t sps30_CalcCrc(u8t data[], u8t nbrOfBytes);
static etError sps30_CheckCrc(u8t data[], u8t nbrOfBytes, u8t checksum);


//----------------------- For I2C stuffs -----------------------
void I2c_Init(void);
void I2c_StartCondition(void);
void I2c_StopCondition(void);
etError I2c_WriteByte(u8t txByte);
etError I2c_ReadByte(u8t *rxByte, etI2cAck ack, u8t timeout);
etError I2c_GeneralCallReset(void);
static etError I2c_WaitWhileClockStreching(u8t timeout);


float GetSinglePrecision(u32t number);
double ConvertNumberToFloat(unsigned long number, int isDoublePrecision);
 
//-----------------------------------------------------------------------------
int main(void)
{  
  etError   error;       // error code 
	
	
	//button
  RCC->APB2ENR |= 0x00000004;  // I/O port A clock enabled
  GPIOA->CRL   &= 0xFFFFFFF0;  // set general purpose input mode for User Button
  GPIOA->CRL   |= 0x00000004;  // set input floating mode - mode0=0100

	RCC->APB2ENR |= 0x00000010;  // I/O port C clock enabled
  GPIOC->CRH   &= 0xFFFFFF00;  // set general purpose output mode for LEDs
  GPIOC->CRH   |= 0x00000011;  // 

  SystemInit();
  EvalBoardPower_Init();

 
	DelayMicroSeconds(1000000);
	
	
	sps30_Init();
	sps30_StartMeasurement();
	
	
	
  while(1)
			{
							ft        flag;  
				
							ft 				mass_PM1 = 0;
							ft 				mass_PM2_5 = 0;
							ft 				mass_PM4 = 0;
							ft 				mass_PM10 = 0;
				
				
							ft 				N_PM0_5 = 0;
							ft 				N_PM1_0 = 0;
							ft 				N_PM4 = 0;
							ft 				N_PM2_5 = 0;
							ft 				N_PM10 = 0; 
				
							char str[1];
				
				 
							error = Data_Ready_Flag(&flag);
				
													
							if(error != NO_ERROR)
								{}

							if(flag==1)
									{
											error = sps30_read_measurement(&mass_PM1, &mass_PM2_5,&mass_PM4, &mass_PM10, &N_PM0_5, &N_PM1_0, &N_PM2_5, &N_PM4, &N_PM10);
											if(error != NO_ERROR){}  
												
											sprintf(str, "PM2.5:  %2.2f", mass_PM2_5);  					
											
											
							}
														
						 DelayMicroSeconds(800000);
			}
}







//-----------------------------------------------------------------------------
static void EvalBoardPower_Init(void)    /* -- adapt this code for your platform -- */
{
//---------------------------SHT30 B GND-11, VDD-15---------------------------
  RCC->APB2ENR |= 0x00000008;  // I/O port B clock enabled

  GPIOB->CRH   &= 0x0FFF0FFF;  // set push-pull output for Vdd & GND pins
  GPIOB->CRH   |= 0x10001000;  //

  GPIOB->BSRR = 0x08008000;    // set Vdd to High, set GND to Low 0x08008000- GND PB11, VDD PB15
	//GPIOB->BSRR = 0x04000000;    // chang ALART to Low

}

static void sps30_Init(void)
{
	
  RCC->APB2ENR |= 0x00000008;  // I/O port B clock enabled
  
  SDA_OPEN();                  // I2C-bus idle mode SDA released
  SCL_OPEN();                  // I2C-bus idle mode SCL released
  
  // SDA on port B, bit 14
  // SCL on port B, bit 13
  GPIOB->CRH   &= 0xF00FFFFF;  // set open-drain output for SDA and SCL
  GPIOB->CRH   |= 0x05500000;  // 
 
 
}

void SystemInit(void)
{
  // no initialization required
}

//-----------------------------------------------------------------------------
void DelayMicroSeconds(u32t nbrOfUs)   /* -- adapt this delay for your uC -- */
{
  u32t i;
  for(i = 0; i < nbrOfUs; i++)
  {
    __nop();  // nop's may be added or removed for timing adjustment
    __nop();
    __nop();
    __nop();
  }
}


//-----------------------------------------------------------------------------
static etError sps30_read_measurement(ft* mass_PM1, ft* mass_PM2_5, ft* mass_PM4, ft* mass_PM10, ft *N_PM0_5, ft *N_PM1_0, ft *N_PM2_5, ft *N_PM4, ft *N_PM10)
{
	unsigned int command_readvalue = 0x0300 ;
	
  etError error;        
  u32t    PM1_0_raw;  
  u32t    PM2_5_raw; 
  u32t    PM4_raw; 
  u32t    PM10_raw; 
	
  u32t    N_PM0_5_raw;  
  u32t    N_PM1_0_raw;  
  u32t    N_PM2_5_raw; 
  u32t    N_PM4_raw; 
  u32t    N_PM10_raw; 
	
	u16t 		byte[2];
	
	int i;

  error = sps30_StartWriteAccess();

  // if no error ...
  if(error == NO_ERROR)
  { 
       
			 error  = I2c_WriteByte(command_readvalue >> 8);

			// write the lower 8 bits of the command to the sensor
			 error |= I2c_WriteByte(command_readvalue & 0xFF);
    }
  
	I2c_StopCondition();
		
  // if no error, start read access
				if(error == NO_ERROR)  
								{ 
									I2c_StartCondition(); 
									I2c_WriteByte(i2cAddress << 1 | 0x01); 
								}
 
	for(i=0;i<9;i++){ 
			if(error == NO_ERROR) error = sps30_Read2BytesAndCrc(&byte[0], ACK,0); 
			if(error == NO_ERROR) error = sps30_Read2BytesAndCrc(&byte[1], ACK,0);  
			
			if(i==0) PM1_0_raw = (byte[0] << 16) | byte[1];
			if(i==1) PM2_5_raw = (byte[0] << 16) | byte[1];
			if(i==2) PM4_raw = (byte[0] << 16) | byte[1];
			if(i==3) PM10_raw = (byte[0] << 16) | byte[1];
		
			if(i==4) N_PM0_5_raw = (byte[0] << 16) | byte[1];
			if(i==5) N_PM1_0_raw = (byte[0] << 16) | byte[1];
			if(i==6) N_PM2_5_raw = (byte[0] << 16) | byte[1];
			if(i==7) N_PM4_raw = (byte[0] << 16) | byte[1];
			if(i==8) N_PM10_raw = (byte[0] << 16) | byte[1];
		 
		}
	
		
	if(error == NO_ERROR) error = sps30_Read2BytesAndCrc(&byte[0], ACK,0); 
	if(error == NO_ERROR) error = sps30_Read2BytesAndCrc(&byte[1], NACK,0); 
	

  I2c_StopCondition();

  // if no error, calculate temperature in �C and humidity in %RH
  if(error == NO_ERROR)
			{
				*mass_PM1 =  GetSinglePrecision(PM1_0_raw);
				*mass_PM2_5 = GetSinglePrecision(PM2_5_raw);
				*mass_PM4 = GetSinglePrecision(PM4_raw);
				*mass_PM10 =  GetSinglePrecision(PM10_raw);
				
				*N_PM0_5  =  GetSinglePrecision(N_PM0_5_raw);
				*N_PM1_0 =  GetSinglePrecision(N_PM1_0_raw);
				*N_PM2_5 =  GetSinglePrecision(N_PM2_5_raw);
				*N_PM4	 =  GetSinglePrecision(N_PM4_raw);
				*N_PM10	 =  GetSinglePrecision(N_PM10_raw);
								 
			}

  return error;
}


double ConvertNumberToFloat(unsigned long number, int isDoublePrecision)
{
    int mantissaShift = isDoublePrecision ? 52 : 23;
    unsigned long long exponentMask = isDoublePrecision ? 0x7FF0000000000000 : 0x7f800000;
    int bias = isDoublePrecision ? 1023 : 127;
    int signShift = isDoublePrecision ? 63 : 31;

    int sign = !((number >> signShift) & 0x01);
    int exponent = ((number & exponentMask) >> mantissaShift) - bias;

    int power = -1;
    double total = 0.0;
		int i;
	
    for ( i = 0; i < mantissaShift; i++ )
				{
						int calc = (number >> (mantissaShift-i-1)) & 0x01;
						total += calc * pow(2.0, power);
						power--;
				}
		
		
    total += 1.0;

    return sign * pow(2.0, exponent) * total;
 
}





float GetSinglePrecision(u32t number)
{
    return (float)ConvertNumberToFloat(number, 0);
}
 

// s16 sensirion_i2c_write_cmd_with_args(u8 address, u16 command, const u16 *data_words, u16 num_words) 

//s16 sps30_start_measurement() {
//    const u16 arg = SPS_CMD_START_MEASUREMENT_ARG;

//    return sensirion_i2c_write_cmd_with_args(SPS_I2C_ADDRESS,
//                                             SPS_CMD_START_MEASUREMENT,
//                                             &arg,
//                                             SENSIRION_NUM_WORDS(arg));
//}

//SPS_I2C_ADDRESS = 0x69
//SPS_CMD_START_MEASUREMENT = 0x0010
//SPS_CMD_START_MEASUREMENT_ARG = 0x0300
//


//-----------------------------------------------------------------------------
static etError sps30_StartMeasurement(void)
{
  etError error; // error code
	unsigned int Start_address = 0x0010 ;
	
	
	unsigned int Start_measurement = 0x0300AC ;

  error = sps30_StartWriteAccess();

  // if no error ...
  if(error == NO_ERROR)
  {
    // start measurement in clock stretching mode
    // use depending on the required repeatability, the corresponding command
 
       
			 error  = I2c_WriteByte(Start_address >> 8);

			// write the lower 8 bits of the command to the sensor
			 error |= I2c_WriteByte(Start_address & 0xFF);
		
			 error  |=  I2c_WriteByte(Start_measurement >> 16);
		
			 error  |=  I2c_WriteByte(Start_measurement & 0xFF00);

			// write the lower 8 bits of the command to the sensor
			 error |= I2c_WriteByte(Start_measurement & 0xFF);
		
		
    }
	
	I2c_StopCondition();	
		
  return error;
}
 


//0x01: new measurements ready to read
//0x00: no new measurements available
static etError Data_Ready_Flag(ft *flag)   
{
  etError error; // error code
	unsigned int command_readvalue = 0x0202 ;  
	u16t flag_raw;
	

  error = sps30_StartWriteAccess();

  // if no error ...
  if(error == NO_ERROR)
  {
    // start measurement in clock stretching mode
    // use depending on the required repeatability, the corresponding command
 
       
			 error  = I2c_WriteByte(command_readvalue >> 8);

			// write the lower 8 bits of the command to the sensor
			 error |= I2c_WriteByte(command_readvalue & 0xFF);
    }
	
	I2c_StopCondition();	
		
  DelayMicroSeconds(10000); 
		
  // if no error, start read access
				if(error == NO_ERROR)  
								{ 
									I2c_StartCondition(); 
									I2c_WriteByte(i2cAddress << 1 | 0x01); 
								}

  // read 6 bytes data. first 3 bytes are temperature and second 3 bytes are humidity
  // if no error, read temperature raw values 
  if(error == NO_ERROR) error = sps30_Read2BytesAndCrc(&flag_raw, NACK, 20);

  I2c_StopCondition();

  // if no error, calculate temperature in �C and humidity in %RH
  if(error == NO_ERROR)
  {
				*flag = (float) flag_raw;
  }

  return error;

}




//-----------------------------------------------------------------------------
static etError sps30_StartWriteAccess(void)
{
  etError error; // error code

  // write a start condition
  I2c_StartCondition();

  // write the sensor I2C address with the write flag
  error = I2c_WriteByte(i2cAddress << 1);

  return error;
}

//-----------------------------------------------------------------------------
static etError sps30_Read2BytesAndCrc(u16t* data, etI2cAck finaleAckNack,
                                      u8t timeout)
{
  etError error;    // error code
  u8t     bytes[2]; // read data array
  u8t     checksum; // checksum byte

  // read two data bytes and one checksum byte
                        error = I2c_ReadByte(&bytes[0], ACK, timeout);
  if(error == NO_ERROR) error = I2c_ReadByte(&bytes[1], ACK, 0);
  if(error == NO_ERROR) error = I2c_ReadByte(&checksum, finaleAckNack, 0);

  // verify checksum
  if(error == NO_ERROR) error = sps30_CheckCrc(bytes, 2, checksum);

  // combine the two bytes to a 16-bit value
  *data = (bytes[0] << 8) | bytes[1];

  return error;
}

//-----------------------------------------------------------------------------
static u8t sps30_CalcCrc(u8t data[], u8t nbrOfBytes)
{
  u8t bit;        // bit mask
  u8t crc = 0xFF; // calculated checksum
  u8t byteCtr;    // byte counter

  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
  {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit)
    {
      if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else           crc = (crc << 1);
    }
  } 
  return crc;
}

//-----------------------------------------------------------------------------
static etError sps30_CheckCrc(u8t data[], u8t nbrOfBytes, u8t checksum)
{
  u8t crc;     // calculated checksum

  // calculates 8-Bit checksum
  crc = sps30_CalcCrc(data, nbrOfBytes);

  // verify checksum
  if(crc != checksum) return CHECKSUM_ERROR;
  else                return NO_ERROR;
}


//-----------------------------------------------------------------------------
//=============================================================================
//the below is for I2C part..
//this should be same for all sensors
//=============================================================================
//-----------------------------------------------------------------------------
void I2c_Init(void)                      /* -- adapt the init for your uC -- */
{
  RCC->APB2ENR |= 0x00000008;  // I/O port B clock enabled



  // SDA on port B, bit 14
  // SCL on port B, bit 13
  GPIOB->CRH   &= 0xF00FFFFF;  // set open-drain output for SDA and SCL
  GPIOB->CRH   |= 0x05500000;  //
	  
  SDA_OPEN();                  // I2C-bus idle mode SDA released
  SCL_OPEN();                  // I2C-bus idle mode SCL released
}

//-----------------------------------------------------------------------------
void I2c_StartCondition(void)
{
  SDA_OPEN();
  DelayMicroSeconds(1);
  SCL_OPEN();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(10);  // hold time start condition (t_HD;STA)
  SCL_LOW();
  DelayMicroSeconds(10);
}

//-----------------------------------------------------------------------------
void I2c_StopCondition(void)
{
  SCL_LOW();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(1);
  SCL_OPEN();
  DelayMicroSeconds(10);  // set-up time stop condition (t_SU;STO)
  SDA_OPEN();
  DelayMicroSeconds(10);
}

//-----------------------------------------------------------------------------
etError I2c_WriteByte(u8t txByte)
{
  etError error = NO_ERROR;
  u8t     mask;
  for(mask = 0x80; mask > 0; mask >>= 1)// shift bit for masking (8 times)
  {
    if((mask & txByte) == 0) SDA_LOW(); // masking txByte, write bit to SDA-Line
    else                     SDA_OPEN();
    DelayMicroSeconds(1);               // data set-up time (t_SU;DAT)
    SCL_OPEN();                         // generate clock pulse on SCL
    DelayMicroSeconds(5);               // SCL high time (t_HIGH)
    SCL_LOW();
    DelayMicroSeconds(1);               // data hold time(t_HD;DAT)
  }
  SDA_OPEN();                           // release SDA-line
  SCL_OPEN();                           // clk #9 for ack
  DelayMicroSeconds(1);                 // data set-up time (t_SU;DAT)
  if(SDA_READ) error = ACK_ERROR;       // check ack from i2c slave
  SCL_LOW();
  DelayMicroSeconds(20);                // wait to see byte package on scope
  return error;                         // return error code
}

//-----------------------------------------------------------------------------
etError I2c_ReadByte(u8t *rxByte, etI2cAck ack, u8t timeout)
{
  etError error = NO_ERROR;
  u8t mask;
  *rxByte = 0x00;
  SDA_OPEN();                            // release SDA-line
  for(mask = 0x80; mask > 0; mask >>= 1) // shift bit for masking (8 times)
  {
    SCL_OPEN();                          // start clock on SCL-line
    DelayMicroSeconds(1);                // clock set-up time (t_SU;CLK)
    error = I2c_WaitWhileClockStreching(timeout);// wait while clock streching
    DelayMicroSeconds(3);                // SCL high time (t_HIGH)
    if(SDA_READ) *rxByte |= mask;        // read bit
    SCL_LOW();
    DelayMicroSeconds(1);                // data hold time(t_HD;DAT)
  }
  if(ack == ACK) SDA_LOW();              // send acknowledge if necessary
  else           SDA_OPEN();
  DelayMicroSeconds(1);                  // data set-up time (t_SU;DAT)
  SCL_OPEN();                            // clk #9 for ack
  DelayMicroSeconds(5);                  // SCL high time (t_HIGH)
  SCL_LOW();
  SDA_OPEN();                            // release SDA-line
  DelayMicroSeconds(20);                 // wait to see byte package on scope

  return error;                          // return with no error
}

//-----------------------------------------------------------------------------
etError I2c_GeneralCallReset(void)
{
  etError error;

  I2c_StartCondition();
                        error = I2c_WriteByte(0x00);
  if(error == NO_ERROR) error = I2c_WriteByte(0x06);

  return error;
}


//-----------------------------------------------------------------------------
static etError I2c_WaitWhileClockStreching(u8t timeout)
{
  etError error = NO_ERROR;

  while(SCL_READ == 0)
  {
    if(timeout-- == 0) return TIMEOUT_ERROR;
    DelayMicroSeconds(1000);
  }

  return error;
}

