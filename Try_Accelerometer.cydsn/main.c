/*******************************************************************************
* File Name: main.c
*
* Version: alpha taco
*
* Description:
*  This is code from the https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library
*  re-writen to use teh PSoC SPI Master component
*  (Master, ADXL345) components.
*  Parameters used:
*   Mode                (CPHA ==1, CPOL ==1)                
*   Data lines          MOSI+MISO
*   Shift direction     MSB First
*   DataBits:           16 currently 
*   Bit Rate            To be determined
*   Clock source        External 
*
*  SPI communication test.
*
********************************************************************************
* Copyright 2012, Cypress Semiconductor Corporation. All rights reserved.
* This software is owned by Cypress Semiconductor Corporation and is protected
* by and subject to worldwide patent and copyright laws and treaties.
* Therefore, you may use this software only as provided in the license agreement
* accompanying the software package from which you obtained this software.
* CYPRESS AND ITS SUPPLIERS MAKE NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* WITH REGARD TO THIS SOFTWARE, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

#include <project.h>
#include <stdio.h>

/*************************** REGISTER MAP ***************************/
#define ADXL345_DEVID			0x00		// Device ID
#define ADXL345_RESERVED1		0x01		// Reserved. Do Not Access. 
#define ADXL345_THRESH_TAP		0x1D		// Tap Threshold. 
#define ADXL345_OFSX			0x1E		// X-Axis Offset. 
#define ADXL345_OFSY			0x1F		// Y-Axis Offset.
#define ADXL345_OFSZ			0x20		// Z- Axis Offset.
#define ADXL345_DUR				0x21		// Tap Duration.
#define ADXL345_LATENT			0x22		// Tap Latency.
#define ADXL345_WINDOW			0x23		// Tap Window.
#define ADXL345_THRESH_ACT		0x24		// Activity Threshold
#define ADXL345_THRESH_INACT	0x25		// Inactivity Threshold
#define ADXL345_TIME_INACT		0x26		// Inactivity Time
#define ADXL345_ACT_INACT_CTL	0x27		// Axis Enable Control for Activity and Inactivity Detection
#define ADXL345_THRESH_FF		0x28		// Free-Fall Threshold.
#define ADXL345_TIME_FF			0x29		// Free-Fall Time.
#define ADXL345_TAP_AXES		0x2A		// Axis Control for Tap/Double Tap.
#define ADXL345_ACT_TAP_STATUS	0x2B		// Source of Tap/Double Tap
#define ADXL345_BW_RATE			0x2C		// Data Rate and Power mode Control
#define ADXL345_POWER_CTL		0x2D		// Power-Saving Features Control
#define ADXL345_INT_ENABLE		0x2E		// Interrupt Enable Control
#define ADXL345_INT_MAP			0x2F		// Interrupt Mapping Control
#define ADXL345_INT_SOURCE		0x30		// Source of Interrupts
#define ADXL345_DATA_FORMAT		0x31		// Data Format Control
#define ADXL345_DATAX0			0x32		// X-Axis Data 0
#define ADXL345_DATAX1			0x33		// X-Axis Data 1
#define ADXL345_DATAY0			0x34		// Y-Axis Data 0
#define ADXL345_DATAY1			0x35		// Y-Axis Data 1
#define ADXL345_DATAZ0			0x36		// Z-Axis Data 0
#define ADXL345_DATAZ1			0x37		// Z-Axis Data 1
#define ADXL345_FIFO_CTL		0x38		// FIFO Control
#define ADXL345_FIFO_STATUS		0x39		// FIFO Status

#define ADXL345_BW_1600			0xF			// 1111		IDD = 40uA
#define ADXL345_BW_800			0xE			// 1110		IDD = 90uA
#define ADXL345_BW_400			0xD			// 1101		IDD = 140uA
#define ADXL345_BW_200			0xC			// 1100		IDD = 140uA
#define ADXL345_BW_100			0xB			// 1011		IDD = 140uA 
#define ADXL345_BW_50			0xA			// 1010		IDD = 140uA
#define ADXL345_BW_25			0x9			// 1001		IDD = 90uA
#define ADXL345_BW_12_5		    0x8			// 1000		IDD = 60uA 
#define ADXL345_BW_6_25			0x7			// 0111		IDD = 50uA
#define ADXL345_BW_3_13			0x6			// 0110		IDD = 45uA
#define ADXL345_BW_1_56			0x5			// 0101		IDD = 40uA
#define ADXL345_BW_0_78			0x4			// 0100		IDD = 34uA
#define ADXL345_BW_0_39			0x3			// 0011		IDD = 23uA
#define ADXL345_BW_0_20			0x2			// 0010		IDD = 23uA
#define ADXL345_BW_0_10			0x1			// 0001		IDD = 23uA
#define ADXL345_BW_0_05			0x0			// 0000		IDD = 23uA


 /************************** INTERRUPT PINS **************************/
#define ADXL345_INT1_PIN		0x00		//INT1: 0
#define ADXL345_INT2_PIN		0x01		//INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define ADXL345_INT_DATA_READY_BIT		0x07
#define ADXL345_INT_SINGLE_TAP_BIT		0x06
#define ADXL345_INT_DOUBLE_TAP_BIT		0x05
#define ADXL345_INT_ACTIVITY_BIT		0x04
#define ADXL345_INT_INACTIVITY_BIT		0x03
#define ADXL345_INT_FREE_FALL_BIT		0x02
#define ADXL345_INT_WATERMARK_BIT		0x01
#define ADXL345_INT_OVERRUNY_BIT		0x00

#define ADXL345_DATA_READY				0x07
#define ADXL345_SINGLE_TAP				0x06
#define ADXL345_DOUBLE_TAP				0x05
#define ADXL345_ACTIVITY				0x04
#define ADXL345_INACTIVITY				0x03
#define ADXL345_FREE_FALL				0x02
#define ADXL345_WATERMARK				0x01
#define ADXL345_OVERRUNY				0x00


 /****************************** ERRORS ******************************/
#define ADXL345_OK			1		// No Error
#define ADXL345_ERROR		0		// Error Exists

#define ADXL345_NO_ERROR	0		// Initial State
#define ADXL345_READ_ERROR	1		// Accelerometer Reading Error
#define ADXL345_BAD_ARG		2		// Bad Argument

#define ADXL345_TO_READ (6)      // Number of Bytes Read - Two Bytes Per Axis

double gains[3];				// Counts to Gs
uint8 _buff[6] ;
uint16 _bigbuff[3];
uint8 msg_count = 0;
int _CS = 10;
uint8 I2C = 0;

uint8 status;					// Set When Error Exists 

uint8 error_code;				// Initial State
uint8 prev_byte = 0;

char msg1[20];
char msg2[20];
char msg3[20];

/*******************************************************************************
* Woopty Woo
********************************************************************************
*******************************************************************************/
// function prototoypes
void ADXL345(int CS);
void setup();
void powerOn();
void setRangeSetting(int val);
void setSpiBit(uint8 spiBit);
uint8 getRegisterBit(uint8 regAdress, int bitPos);
void setRegisterBit(uint8 regAdress, int bitPos, uint8 state);
void setActivityXYZ(uint8 stateX, uint8 stateY, uint8 stateZ);
void setActivityX(uint8 state);
void setActivityY(uint8 state);
void setActivityZ(uint8 state);
void setInactivityXYZ(uint8 stateX, uint8 stateY, uint8 stateZ);
void setInactivityX(uint8 state);
void setInactivityY(uint8 state);
void setInactivityZ(uint8 state);
void setInactivityThreshold(int inactivityThreshold);
void setActivityThreshold(int activityThreshold);
int getActivityThreshold();
void setTimeInactivity(int timeInactivity);
void setTapDetectionOnXYZ(uint8 stateX, uint8 stateY, uint8 stateZ);
uint8 isTapDetectionOnX();
void setTapDetectionOnX(uint8 state);
uint8 isTapDetectionOnY();
void setTapDetectionOnY(uint8 state);
uint8 isTapDetectionOnZ();
void setTapDetectionOnZ(uint8 state);
void readFrom(uint8 address, int num, uint8 _buff[]);
void readFromSPI(uint8 __reg_address, int num, uint8 _buff[]);
void readFromSPI_16(uint8 __reg_address, int num, uint16* _bigbuff);
void writeTo(uint8 address, uint8 val);
void writeToSPI(uint8 __reg_address, uint8 __val);
void setTapThreshold(int tapThreshold);
void setTapDuration(int tapDuration);
void setDoubleTapLatency(int doubleTapLatency);
void setDoubleTapWindow(int doubleTapWindow);
int getDoubleTapWindow();
void setFreeFallThreshold(int freeFallThreshold);
int getFreeFallThreshold();
void setFreeFallDuration(int freeFallDuration);
int getFreeFallDuration();
void InactivityINT(uint8 status);
void setInterrupt(uint8 interruptBit, uint8 state);
void ActivityINT(uint8 status);
void FreeFallINT(uint8 status);
void singleTapINT(uint8 status);
void doubleTapINT(uint8 status);
void readAccel(int *xyz);
void readAccelSingles(int16 *x, int16 *y, int16 *z);
void get_Gxyz(double *xyz);
uint8 getInterruptSource();
uint8 triggered(uint8 interrupts, int mask);
void loop();
void set_bw(uint8 bw_code);
uint8 get_bw_code();
uint8 getDeviceId();

int constrain(int inval, int lower, int upper);


int main()
{
    uint8 i = 0u;
    
    /* Software buffers use internal interrupt functionality to interact with
    * hardware buffers. Thus global interrupt enable command should be called 
    */
    CyGlobalIntEnable;
    
    /* We need to start Character LCD, SPI Master and Slave components */
    LCD_Start();
    SPIM_Start();
    
    //SPIS_Start();
    //CyDelay(500u);
    SPIM_ClearTxBuffer();
    ADXL345(10);
    
    LCD_Position(2u,0u);
    LCD_PrintString("Ere We Go!!!");
    
    LCD_Position(3u, 0u);
    LCD_PrintString("Shem Shala Bem");
    //SPIM_ClearTxBuffer();
    //SPIM_ClearRxBuffer();
    //SPIM_ClearFIFO();
    CyDelay(4000u);
    
    LCD_Position(3u,0u);
    LCD_PrintString("Peaky Blinders!!!");
    setup(); // Modified setup from Arduino Example Code.
    
    while(1){
        loop(); // slightly modified arduino loop function.
    }
      
}   

void powerOn() {
	//ADXL345 TURN ON
	writeTo(ADXL345_POWER_CTL, 0);	// Wakeup
	writeTo(ADXL345_POWER_CTL, 16);	// Auto_Sleep
	writeTo(ADXL345_POWER_CTL, 8);	// Measure
}

void setup(){
  SPIM_ClearTxBuffer();

  //uint8 this = 0;
  //while(this != 0x28){
  //  writeTo(ADXL345_POWER_CTL, 0x28);
  //  readFrom();
  //}

  powerOn();                     // Power on the ADXL345
  CyDelay(100u);

  setRangeSetting(16);           // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity
  set_bw(ADXL345_BW_1600);
  //set_bw(ADXL345_BW_400);
  CyDelay(10u);

  setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
                                      // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
  CyDelay(10u);
  setActivityXYZ(1, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)

  SPIM_ClearRxBuffer();
  volatile uint8 actTh = 0;
  actTh = getActivityThreshold();
 
  setInactivityXYZ(1, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  setTimeInactivity(10);         // How many seconds of no activity is inactive?

  setTapDetectionOnXYZ(0, 0, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
 
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  setTapThreshold(50);           // 62.5 mg per increment

  setTapDuration(15);            // 625 μs per increment
  setDoubleTapLatency(80);       // 1.25 ms per increment
  setDoubleTapWindow(200);       // 1.25 ms per increment
 
  // Set values for what is considered FREE FALL (0-255)
  setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
 
  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);" 
                                                        // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
                                                        // This library may have a problem using INT2 pin. Default to INT1 pin.
  
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  /*InactivityINT(1);
  ActivityINT(1);
  FreeFallINT(1);
  doubleTapINT(1);
  singleTapINT(1);*/


  SPIM_ClearRxBuffer();
 // while(1){
  // Verify Device ID and threshold I set.
  uint8 devId = getDeviceId();
  
  char dev_msg[20];
  sprintf(dev_msg, "devID: 0x%x, %u", devId, actTh);

  LCD_Position(0u,0u);
  LCD_PrintString(dev_msg);
  CyDelay(1000);
//}
  
//attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt

}

void ADXL345(int CS) {
	status = ADXL345_OK;
	error_code = ADXL345_NO_ERROR;

	gains[0] = 0.00376390;
	gains[1] = 0.00376009;
	gains[2] = 0.00349265;
	_CS = CS;
	I2C = 0;
	//SPI.begin();
	//SPI.setDataMode(SPI_MODE3);
	//pinMode(_CS, OUTPUT);
	//digitalWrite(_CS, HIGH);
}

/*********************** THRESH_ACT REGISTER ************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Activity.
// Data Format is Unsigned, so the Magnitude of the Activity Event is Compared
//  with the Value is Compared with the Value in the THRESH_ACT Register.
// The Scale Factor is 62.5mg/LSB.
// Value of 0 may Result in Undesirable Behavior if the Activity Interrupt Enabled.
// It Accepts a Maximum Value of 255.
void setActivityThreshold(int activityThreshold) {
	activityThreshold = constrain(activityThreshold,0,255);
	uint8 _b = (uint8) (activityThreshold);
	writeTo(ADXL345_THRESH_ACT, _b);
}

// Gets the THRESH_ACT byte
int getActivityThreshold() {
	uint8 _b;
	readFrom(ADXL345_THRESH_ACT, 1, &_b);
	return (int) (_b);
}

void setRangeSetting(int val) {
	uint8 _s;
	uint8 _b;

	switch (val) {
		case 2:
			_s = 0x0;
			break;
		case 4:
			_s = 0x1;
			break;
		case 8:
			_s = 0x2;
			break;
		case 16:
			_s = 0x3;
			break;
		default:
			_s = 0x0;
	}
	readFrom(ADXL345_DATA_FORMAT, 1, &_b);
	_s |= (_b & 0xed); //B11101100
	writeTo(ADXL345_DATA_FORMAT, _s);
}

void setSpiBit(uint8 spiBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}

void setRegisterBit(uint8 regAdress, int bitPos, uint8 state) {
	uint8 _b;
	readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.
	}
	else {
		_b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
	}
	writeTo(regAdress, _b);
}

void setActivityXYZ(uint8 stateX, uint8 stateY, uint8 stateZ) {
	setActivityX(stateX);
	setActivityY(stateY);
	setActivityZ(stateZ);
}

void setActivityX(uint8 state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state);
}
void setActivityY(uint8 state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state);
}
void setActivityZ(uint8 state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state);
}

/*********************** READING ACCELERATION ***********************/
/*    Reads Acceleration into Three Variables:  x, y and z          */

void readAccel(int *xyz){
	readAccelSingles(xyz, xyz + 1, xyz + 2);
}

void readAccelSingles(int16 *x, int16 *y, int16 *z) {
	readFrom(ADXL345_DATAX0, 1, &_buff[0]);	// Read Accel Data from ADXL345
    readFrom(ADXL345_DATAX1, 1, &_buff[1]);	// Read Accel Data from ADXL345
    readFrom(ADXL345_DATAY0, 1, &_buff[2]);	// Read Accel Data from ADXL345
    readFrom(ADXL345_DATAY1, 1, &_buff[3]);	// Read Accel Data from ADXL345
    readFrom(ADXL345_DATAZ0, 1, &_buff[4]);	// Read Accel Data from ADXL345
    readFrom(ADXL345_DATAZ1, 1, &_buff[5]);	// Read Accel Data from ADXL345
	//readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff);
    // Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
    //readFromSPI_16(ADXL345_DATAX0, 6, _bigbuff);
	*x = (int16_t)(((_buff[1]) << 8) | _buff[0]);
	*y = (int16_t)(((_buff[3]) << 8) | _buff[2]);
	*z = (int16_t)(((_buff[5]) << 8) | _buff[4]);
    //*x = _bigbuff[0];
	//*y = _bigbuff[1];
	//*z = _bigbuff[2];
}

void get_Gxyz(double *xyz){
	int i;
	int xyz_int[3];
	readAccel(xyz_int);
	for(i=0; i<3; i++){
		xyz[i] = xyz_int[i] * gains[i];
	}
}

void setInactivityXYZ(uint8 stateX, uint8 stateY, uint8 stateZ) {
	setInactivityX(stateX);
	setInactivityY(stateY);
	setInactivityZ(stateZ);
}

void setInactivityX(uint8 state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state);
}
void setInactivityY(uint8 state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state);
}
void setInactivityZ(uint8 state) {
	setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state);
}

void setInactivityThreshold(int inactivityThreshold) {
	inactivityThreshold = constrain(inactivityThreshold,0,255);
	uint8 _b = (uint8)inactivityThreshold;
	writeTo(ADXL345_THRESH_INACT, _b);
}

void setTimeInactivity(int timeInactivity) {
	timeInactivity = constrain(timeInactivity,0,255);
	uint8 _b = (uint8)timeInactivity;
	writeTo(ADXL345_TIME_INACT, _b);
}

int constrain(int inval, int lower, int upper){
    if(inval < lower) return lower;
    if(inval > upper) return upper;
    return inval;
}

void setTapDetectionOnXYZ(uint8 stateX, uint8 stateY, uint8 stateZ) {
	setTapDetectionOnX(stateX);
	setTapDetectionOnY(stateY);
	setTapDetectionOnZ(stateZ);
}

uint8 isTapDetectionOnX(){
	return getRegisterBit(ADXL345_TAP_AXES, 2);
}
void setTapDetectionOnX(uint8 state) {
	setRegisterBit(ADXL345_TAP_AXES, 2, state);
}
uint8 isTapDetectionOnY(){
	return getRegisterBit(ADXL345_TAP_AXES, 1);
}
void setTapDetectionOnY(uint8 state) {
	setRegisterBit(ADXL345_TAP_AXES, 1, state);
}
uint8 isTapDetectionOnZ(){
	return getRegisterBit(ADXL345_TAP_AXES, 0);
}
void setTapDetectionOnZ(uint8 state) {
	setRegisterBit(ADXL345_TAP_AXES, 0, state);
}

uint8 getRegisterBit(uint8 regAdress, int bitPos) {
	uint8 _b;
	readFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}

uint8 getDeviceId(){
    uint8 _b = 0;
    readFrom(ADXL345_DEVID, 1, &_b);
    return _b;
}

/************************ READING NUM BYTES *************************/
/*    Reads Num Bytes. Starts from Address Reg to _buff Array        */
void readFrom(uint8 address, int num, uint8 _buff[]) {
	readFromSPI(address, num, _buff);	// If SPI Communication
}

/*************************** READ FROM SPI **************************/
/*
*/

void readFromSPI_16(uint8 __reg_address, int num, uint16* bigbuff){
    // Attempt to read entire x,y,z register set in one shot
    
    uint16 _address = (0x80 | __reg_address) << 8;
    // If Multi-Byte Read: Bit 6 Set
    if(num > 1) {
    _address = _address | 0x4000;
    }
    SPIM_ClearRxBuffer();
    SPIM_WriteTxData(_address); // Register byte with first recieve byte 0x00
    SPIM_WriteTxData(0x0000); // recieve bytes 2 and 3
    while(!(SPIM_ReadTxStatus() & SPIM_STS_SPI_DONE)){}
    volatile uint16 tmp1 = SPIM_ReadRxData(); // read bytes garbage and 1
    uint16 tmp2 = SPIM_ReadRxData(); // read byte 2 and 3
    while((SPIM_ReadRxStatus() & SPIM_STS_RX_FIFO_NOT_EMPTY)){}
    SPIM_WriteTxData(0x0000); // recieve bytes 4 and 5
    SPIM_WriteTxData(0x0000); // recieve byte 6 and garbage?
    while(!(SPIM_ReadTxStatus() & SPIM_STS_SPI_DONE)){}
    //SPIM_WriteTxData(0x00);
    //ChannelSel_Write(0u);
    uint16 tmp3 = SPIM_ReadRxData();//read bytes 4 and 5
    uint16 tmp4 = SPIM_ReadRxData();// read bytes 6 and garbage
    while((SPIM_ReadRxStatus() & SPIM_STS_RX_FIFO_NOT_EMPTY)){}
    bigbuff[0] = (tmp1 & 0x00FF)|(tmp2 & 0xFF00); // little endian shifting
    bigbuff[1] = (tmp2 & 0x00FF)|(tmp3 & 0xFF00);
    bigbuff[2] = (tmp3 & 0x00FF)|(tmp4 & 0xFF00);
    //while((SPIM_ReadRxStatus() & SPIM_STS_RX_FIFO_NOT_EMPTY)){}
    
    
}

void readFromSPI(uint8 __reg_address, int num, uint8* buff) {
  // Read: Most Sig Bit of Reg Address Set
  /**** This Current version only works on 1 request and register read nut multi byte read *****/
  uint16 _address = (0x80 | __reg_address) << 8;
  //uint8 tmp;
  // If Multi-Byte Read: Bit 6 Set
  if(num > 1) {
  	_address = _address | 0x4000;
  }
  SPIM_ClearRxBuffer();
  //digitalWrite(_CS, LOW);
  //SPI.transfer(_address);		// Transfer Starting Reg Address To Be Read
  
  SPIM_WriteTxData(_address);
  while(!(SPIM_ReadTxStatus() & SPIM_STS_SPI_DONE)){}
  *buff = SPIM_ReadRxData(); // low byte
  while((SPIM_ReadRxStatus() & SPIM_STS_RX_FIFO_NOT_EMPTY)){}
  //for(int i = 1; i < num / 2 + 1; ++i){
  //    SPIM_WriteTxData(0x00);
  //    while(!(SPIM_ReadTxStatus() & SPIM_STS_SPI_DONE)){}
  //    buff[i] = SPIM_ReadRxData();
  //    while((SPIM_ReadRxStatus() & SPIM_STS_RX_FIFO_NOT_EMPTY)){}
  //}
}

/***************** WRITES VALUE TO ADDRESS REGISTER *****************/
void writeTo(uint8 address, uint8 val) {
	writeToSPI(address, val);
}

/************************** WRITE FROM SPI **************************/
/*         Point to Destination; Write Value; Turn Off              */
void writeToSPI(uint8 __reg_address, uint8 __val) {
    SPIM_ClearTxBuffer();
    uint16 _packet = (__reg_address << 8)| __val;
    SPIM_WriteTxData(_packet);
    //SPIM_WriteTxData(__val);
    while(!(SPIM_ReadTxStatus() & SPIM_STS_SPI_DONE)){}
}

/*********************** THRESH_TAP BYTE VALUE **********************/
/*                          ~ SET & GET                             */
// Should Set Between 0 and 255
// Scale Factor is 62.5 mg/LSB
// A Value of 0 May Result in Undesirable Behavior
void setTapThreshold(int tapThreshold) {
	tapThreshold = constrain(tapThreshold,0,255);
	uint8 _b = (uint8)tapThreshold;
	writeTo(ADXL345_THRESH_TAP, _b);
}

/****************************** DUR BYTE ****************************/
/*                           ~ SET & GET                            */
// DUR Byte: Contains an Unsigned Time Value Representing the Max Time
//  that an Event must be Above the THRESH_TAP Threshold to qualify
//  as a Tap Event
// The scale factor is 625µs/LSB
// Value of 0 Disables the Tap/Double Tap Funcitons. Max value is 255.
void setTapDuration(int tapDuration) {
	tapDuration = constrain(tapDuration,0,255);
	uint8 _b = (uint8)tapDuration;
	writeTo(ADXL345_DUR, _b);
}

/************************** LATENT REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains Unsigned Time Value Representing the Wait Time from the Detection
//  of a Tap Event to the Start of the Time Window (defined by the Window
//  Register) during which a possible Second Tap Even can be Detected.
// Scale Factor is 1.25ms/LSB.
// A Value of 0 Disables the Double Tap Function.
// It Accepts a Maximum Value of 255.
void setDoubleTapLatency(int doubleTapLatency) {
	uint8 _b = (uint8) (doubleTapLatency);
	writeTo(ADXL345_LATENT, _b);
}

/************************** WINDOW REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains an Unsigned Time Value Representing the Amount of Time
//  After the Expiration of the Latency Time (determined by Latent register)
//  During which a Second Valid Tape can Begin.
// Scale Factor is 1.25ms/LSB.
// Value of 0 Disables the Double Tap Function.
// It Accepts a Maximum Value of 255.
void setDoubleTapWindow(int doubleTapWindow) {
	doubleTapWindow = constrain(doubleTapWindow,0,255);
	uint8 _b = (uint8) (doubleTapWindow);
	writeTo(ADXL345_WINDOW, _b);
}

int getDoubleTapWindow() {
	uint8 _b;
	readFrom(ADXL345_WINDOW, 1, &_b);
	return (int) (_b);
}

/*********************** THRESH_FF Register *************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value, in Unsigned Format, for Free-Fall Detection
// The Acceleration on all Axes is Compared with the Value in THRES_FF to
//  Determine if a Free-Fall Event Occurred.
// Scale Factor is 62.5mg/LSB.
// Value of 0 May Result in Undesirable Behavior if the Free-Fall interrupt Enabled.
// Accepts a Maximum Value of 255.
void setFreeFallThreshold(int freeFallThreshold) {
	freeFallThreshold = constrain(freeFallThreshold,0,255);
	uint8 _b = (uint8) (freeFallThreshold);
	writeTo(ADXL345_THRESH_FF, _b);
}

int getFreeFallThreshold() {
	uint8 _b;
	readFrom(ADXL345_THRESH_FF, 1, &_b);
	return (int) (_b);
}

/************************ TIME_FF Register **************************/
/*                          ~ SET & GET                             */
// Stores an Unsigned Time Value Representing the Minimum Time that the Value
//  of all Axes must be Less Than THRES_FF to Generate a Free-Fall Interrupt.
// Scale Factor is 5ms/LSB.
// Value of 0 May Result in Undesirable Behavior if the Free-Fall Interrupt Enabled.
// Accepts a Maximum Value of 255.
void setFreeFallDuration(int freeFallDuration) {
	freeFallDuration = constrain(freeFallDuration,0,255);
	uint8 _b = (uint8) (freeFallDuration);
	writeTo(ADXL345_TIME_FF, _b);
}

int getFreeFallDuration() {
	uint8 _b;
	readFrom(ADXL345_TIME_FF, 1, &_b);
	return (int) (_b);
}

void InactivityINT(uint8 status) {
	if(status) {
		setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
	}
	else {
		setInterrupt( ADXL345_INT_INACTIVITY_BIT, 0);
	}
}

void setInterrupt(uint8 interruptBit, uint8 state) {
	setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

void ActivityINT(uint8 status) {
	if(status) {
		setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
	}
	else {
		setInterrupt( ADXL345_INT_ACTIVITY_BIT,   0);
	}
}

void FreeFallINT(uint8 status) {
	if(status) {
		setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
	}
	else {
		setInterrupt( ADXL345_INT_FREE_FALL_BIT,  0);
	}
}

void singleTapINT(uint8 status) {
	if(status) {
		setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
	}
	else {
		setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 0);
	}
}

void doubleTapINT(uint8 status) {
	if(status) {
		setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
	}
	else {
		setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 0);
	}
}

uint8 getInterruptSource() {
	uint8 _b;
	readFrom(ADXL345_INT_SOURCE, 1, &_b);
	return _b;
}

/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                             */
void set_bw(uint8 bw_code){
	if((bw_code < ADXL345_BW_0_05) || (bw_code > ADXL345_BW_1600)){
		status = 0;
		error_code = ADXL345_BAD_ARG;
	}
	else{
		writeTo(ADXL345_BW_RATE, bw_code);
	}
}

uint8 get_bw_code(){
	uint8 bw_code;
	readFrom(ADXL345_BW_RATE, 1, &bw_code);
	return bw_code;
}

/************************* TRIGGER CHECK  ***************************/
/*                                                                  */
// Check if Action was Triggered in Interrupts
// Example triggered(interrupts, ADXL345_SINGLE_TAP);
uint8 triggered(uint8 interrupts, int mask){
	return ((interrupts >> mask) & 1);
}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {
  
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  uint8 interrupts = 0;
  char check[20];
  interrupts = getInterruptSource();
  //sprintf(check, "%x, %x, %x", interrupts, (interrupts >> ADXL345_FREE_FALL), ADXL345_FREE_FALL);
  //LCD_Position(1u,0u);
  //LCD_PrintString(check);
  // Free Fall Detection
  if(triggered(interrupts, ADXL345_FREE_FALL)){
    
    LCD_Position(0u,0u);
    LCD_PrintString("*** FREE FALL ***");
    //Serial.println("*** FREE FALL ***");
    //add code here to do when free fall is sensed
  } 
  
  // Inactivity
  if(triggered(interrupts, ADXL345_INACTIVITY)){
    LCD_Position(0u,0u);
    LCD_PrintString("*** INACTIVITY ***");
     //add code here to do when inactivity is sensed
  }
  
  // Activity
  if(triggered(interrupts, ADXL345_ACTIVITY)){
    LCD_Position(0u,0u);
    LCD_PrintString("*** ACTIVITY ***"); 
     //add code here to do when activity is sensed
  }
  
  // Double Tap Detection
  if(triggered(interrupts, ADXL345_DOUBLE_TAP)){
    LCD_Position(0u,0u);
    LCD_PrintString("*** DOUBLE TAP ***");
     //add code here to do when a 2X tap is sensed
  }
  
  // Tap Detection
  if(triggered(interrupts, ADXL345_SINGLE_TAP)){
    LCD_Position(0u,0u);
    LCD_PrintString("*** TAP ***");
     //add code here to do when a tap is sensed
  } 
}

/****************** MAIN CODE ******************/
/*     Accelerometer Readings and Interrupt    */
void loop(){
  
  // Accelerometer Readings
  int16 x,y,z = 0;   
  readAccelSingles(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z
  //uint8 curBW = 0;
  //curBW = get_bw_code();

  // only print values every 1000 data points
  if(msg_count%1000 == 0){
  //if(1){
    sprintf(msg2, "x: %6.d          ", x);
    sprintf(msg1, "y: %6.d          ", y);
    sprintf(msg3, "z: %6.d          ", z);
    LCD_Position(1u,0u);
    LCD_PrintString(msg1);
    LCD_Position(2u,0u);
    LCD_PrintString(msg2);
    LCD_Position(3u,0u);
    LCD_PrintString(msg3);
    msg_count = 1;
  }

  // Output Results to Serial
  /* UNCOMMENT TO VIEW X Y Z ACCELEROMETER VALUES */  
  //Serial.print(x);
  //Serial.print(", ");
  //Serial.print(y);
  //Serial.print(", ");
  //Serial.println(z); 
  
  ADXL_ISR();

  msg_count++;
  // You may also choose to avoid using interrupts and simply run the functions within ADXL_ISR(); 
  //  and place it within the loop instead.  
  // This may come in handy when it doesn't matter when the action occurs. 

}
/* [] END OF FILE */
