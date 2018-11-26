/*================================================================================================================
 Program4-Controller.c; (Note: Program for the Controller PIC24H microcontroller)
 
 A variable resistor connected to AN0 will be used to vary a voltage from 0Vdc to
 3.3Vdc.  The ADC results are stored into a variable (ONTime) which will then be 
 added to an array (SendDataArray).  This array will utilize the UART1 hardware
 module to send this data through the TX pin to a HC-O5 Bluetooth Module.  Another
 HC-05 Bluetooth Module which is paired will be connected to the Robot PIC24H
 microcontroller. The Controller PIC24H microcontroller will also utilize the UART1
 module to receive data through the RX pin connected to the HC-05 Bluetooth Module
 and store this data into an array (ReceiveDataArray). A value from this array is stored
 into a variable (Photocell) and is used to adjust Timer3's Interrupt which will
 control the pulse rate of an LED connected to RB5.

 Note: Communications is setup for 115200 Baud, 8 Data Bits, 1 Stop Bit and No Parity Bit

 For information on PIC24H Compiler Peripheral Libraries refer to link below:
 file:///C:/Program%20Files%20(x86)/Microchip/xc16/v1.32/docs/periph_libs/16-bit%20Peripheral%20Libraries.htm
=================================================================================================================*/
#include "p24HJ128GP502.h"  //Include device library file
#include "timer.h"          //Include TIMER library file
#include "uart.h"           //Include UART library file
#include "adc.h"            //Include ADC library file

#define BUFF_SIZE 32

//Configuration Bits
_FOSCSEL(FNOSC_FRC & IESO_OFF);
_FOSC	(POSCMD_NONE & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSDCMD);
_FWDT	(FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR32 & WDTPOST_PS1);
_FPOR	(FPWRT_PWR1 & ALTI2C_ON);
_FICD	(ICS_PGD1 & JTAGEN_OFF);

//Function prototypes
void InitIO (void);
void InitUART(void);
void ADC (void);			
void ProcessData(void);
void SendData(void);
void Shutdown(void);
long Map(long x, long in_min, long in_max, long out_min, long out_max);

#define SERVO_MAX 3850        //Used to set RC Servo Maximum Value changed to proper max of 3850 Revised By: Daniel Hong Sept 24       
#define SERVO_MIN 3000            //Used to set RC Servo Minimum Value changed to proper min of 3000 Revised By: Daniel Hong Sept 24
#define INPUT_MAX 4095         // Used to set the photocell input max to 4095 Revised By:  Daniel Hong Sept 24       
#define INPUT_MIN 0            // Used for photocell input min to 0 Revised By: Daniel Hong Sept 24

//Global variables
unsigned int ONTime = 0;
unsigned int Photocell = 0;     //Value received from UART1 to control the pulsing rate of an LED connected to RB5 
unsigned int PWML = 65535;        //Variable sent to the Robot PIC24H microcontroller via HC-05 Bluetooth Module			
unsigned int PWMR = 65535;        //Variable sent to the Robot PIC24H microcontroller via HC-05 Bluetooth Module			
unsigned int DirectionR = 0;        //Variable sent to the Robot PIC24H microcontroller via HC-05 Bluetooth Module			
unsigned int DirectionL = 0;        //Variable sent to the Robot PIC24H microcontroller via HC-05 Bluetooth Module		


unsigned char SendDataArray[BUFF_SIZE];     //Array to store the data to be transmitted through UART1 TX pin
unsigned char ReceiveDataArray[BUFF_SIZE];  //Array to store the data received from UART1 RX pin
unsigned char ReceivedChar = 0;             //Variable used to store the last byte received from UART1 RX pin
int i;                                      //Variable used to index through arrays
int uartcount = 0;                          //The current array index of the received data from the UART RX pin

unsigned int CommsLossCount = 0;            //Store the number of times there was a communication loss
unsigned short Communicating;               //Store the value ReceiveDataArray[20], if 1 then new data received
unsigned char StartAutonomous = 0;
int x = 0, xx = 0;

int main (void)
{
	InitIO();               //Call InitIO which configures the input and output pins
	InitUART();             //Call InitUART which configures the UART1 hardware module for communications
                            //with the HC-05 Bluetooth Module
	
	for (i=0; i<BUFF_SIZE; i++) SendDataArray[i] = 0;   //Initialize the array of chars to zero
	SendDataArray[0] = 's';                             //Set first element as 's' for data synchronization
                                                        //and for frame error checking
	while (1)               //Infinite loop
	{
        ProcessData();      //Call ProcessData to update variables for UART1 Communications
		SendData();         //Call SendData to send data through the UART1 TX pin to HC-05 Bluetooth Module
        
        if(CommsLossCount>200){     //If communication loss persists, we assume complete loss of communication
            Shutdown();             //You must define code in Shutdown function below
        }
        else{                       //If there is communication, then the following code block will be executed
                                    //This is where all your functions should be executed
            LATAbits.LATA4 = 0;     //Turn off Communication loss LED
            ADC();                  //Call ADC which configures and reads AN0 Don't call ADC Revised By: Daniel Hong October 29
                       
            PWMR = PWML = 0;                // Disable the motors if no button is pressed Revised By: Daniel Hong November 5 2018
            if (PORTBbits.RB10) {           // Set directions based on the left button being pressed
                DirectionL = 1;             // Revised By: Daniel Hong November 5 2018
                DirectionR = 1;
            } else if (PORTBbits.RB11) {    // Set directions based on the reverse button being pressed
                DirectionL = 0;             // Revised By: Daniel Hong November 5 2018
                DirectionR = 1;
            } else if (PORTBbits.RB12) {    // Set directions based on the forward button being pressed
                DirectionL = 1;             // Revised By: Daniel Hong November 5 2018
                DirectionR = 0;
            } else if (PORTBbits.RB13) {    // Set directions based on the right button being pressed
                DirectionL = 0;             // Revised By: Daniel Hong November 5 2018
                DirectionR = 0;
            } else {
                PWMR = PWML = 65535;
            }
            
            if (PORTBbits.RB15) {
//                DirectionL = 0;
//                DirectionR = 0;
//                PWMR = 30000;
//                PWML = 55000;
                StartAutonomous = 1;
            }
        } 
    }
}
/*****************************************************************************************************************/
void InitIO (void)
{
	TRISAbits.TRISA0 = 1;	//Set RA0 (AN0) as input
    
    AD1PCFGLbits.PCFG9 = 1; //Set RB13 As Digital Instead of Analog
    AD1PCFGLbits.PCFG11 = 1; //Set RB13 As Digital Instead of Analog
    AD1PCFGLbits.PCFG12 = 1; //Set RB12 as Digital Instead of analog
    TRISBbits.TRISB10 = 1;	//Set RB6 as output for limit switch from Robot MCU Revised By: Daniel Hong Oct 29
    TRISBbits.TRISB11 = 1;	//Set RB6 as output for limit switch from Robot MCU Revised By: Daniel Hong Oct 29
    TRISBbits.TRISB12 = 1;	//Set RB6 as output for limit switch from Robot MCU Revised By: Daniel Hong Oct 29
    TRISBbits.TRISB13 = 1;	//Set RB6 as output for limit switch from Robot MCU Revised By: Daniel Hong Oct 29
    
    TRISBbits.TRISB15 = 1;	//Set RB6 as output for limit switch from Robot MCU Revised By: Daniel Hong Oct 29
    
    TRISAbits.TRISA2 = 1;	//Set RA2 as input for switch Revised By: Daniel Hong Oct 29
    TRISBbits.TRISB6 = 0;	//Set RB6 as output for limit switch from Robot MCU Revised By: Daniel Hong Oct 29
    
	TRISAbits.TRISA4 = 0;   //Set RA4 as output for LED to indicate communication loss
    TRISBbits.TRISB5 = 0;	//Set RB5 as output for LED to indicate photocell value from Robot MCU
    
    // Set pins for motor control
    TRISBbits.TRISB2 = 1;
    
	//RP8 TO U1RX           //Set the RP8 pin to UART1 RX pin
	RPINR18bits.U1RXR = 8;	//See TABLE 11-1: SELECTABLE INPUT SOURCES (MAPS INPUT TO FUNCTION),Page 136
                            //and REGISTER 11-8: RPINR18: PERIPHERAL PIN SELECT INPUT REGISTER 18,Page 146

    CNPU2bits.CN22PUE = 1;	//Enable weak pull-up of Receive pin CN22 (Corresponds to RP8)
                            //This is needed for v1.06 Bluetooth boards to pull up the receive line

	//RP9 TO U1TX           //Set the RP9 pin to UART1 TX pin
	RPOR4bits.RP9R = 3;     //See TABLE 11-2: OUTPUT SELECTION FOR REMAPPABLE PIN (RPn), Page 137
                            //and REGISTER 11-19: RPOR4: PERIPHERAL PIN SELECT OUTPUT REGISTERS 4, Page 154
}
/*****************************************************************************************************************/
void InitUART(void)
// For more information on PIC24H UART Peripheral Module Library Help refer to link below:
// file:///C:/Program%20Files%20(x86)/Microchip/xc16/v1.32/docs/periph_libs/dsPIC30F_dsPIC33F_PIC24H_dsPIC33E_PIC24E_UART_Help.htm
{
 	IEC0bits.U1TXIE = 0; 		//Disable UART1 TX Interrupt
    IFS0bits.U1RXIF = 0;		//Clear the Receive Interrupt Flag
	U1MODEbits.STSEL = 0; 		//1 Stop bit
	U1MODEbits.PDSEL = 0;	 	//8-bit data, no parity 
	U1MODEbits.BRGH = 0;		//16x baud clock, Standard mode
	U1MODEbits.URXINV = 0;		//Idle State 1 for RX pin
	U1MODEbits.ABAUD = 0;		//Auto-Baud Disabled
	U1MODEbits.RTSMD = 1;		//Simplex Mode, no flow control
	U1MODEbits.UARTEN = 1; 		//Enable UART1
	U1STAbits.UTXISEL0 = 0; 	//Interrupt after one TX character is transmitted
	U1STAbits.UTXISEL1 = 0; 	//Interrupt after one TX character is transmitted
	U1STAbits.UTXEN = 1; 		//Enable UART1 to control TX pin
	U1BRG = 1;                  //BAUD Rate Setting for 115200
	IEC0bits.U1RXIE = 1;		//Enable UART1 RX interrupt
}
/*****************************************************************************************************************/
void ADC (void)
// For more information on PIC24H ADC Peripheral Module Library refer to link below:
// file:///C:/Program%20Files%20%28x86%29/Microchip/xc16/v1.32/docs/periph_libs/dsPIC33F_PIC24H_dsPIC33E_PIC24E_ADC_Library_Help.htm
{                                   //12-bit sampling
                                    //Use dedicated ADC RC oscillator
                                    //Automatically start new conversion after previous
                                    //Use Avdd and Avss as reference levels
	OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
		ADC_DMA_BUF_LOC_1,
		ENABLE_AN4_ANA,
		0,		
		0,
		0);
                                    //Select AN0
	SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN4);
	AD1CON1bits.ADON = 1;           //Turn on ADC hardware module
	while (AD1CON1bits.DONE == 0);	//Wait until conversion is done
    ONTime = Map(ReadADC1(0), INPUT_MIN, INPUT_MAX, SERVO_MIN, SERVO_MAX);
	AD1CON1bits.ADON = 0;           //Turn off ADC hardware module
}
/*****************************************************************************************************************/
long Map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
/********************************************************************************************************/
void ProcessData(void)
{
	SendDataArray[1] = (PWMR >> 8);       //Populate the array one byte at a time
	SendDataArray[2] = PWMR;              // Revised By: Daniel Hong November 5 2018
    
    SendDataArray[3] = (PWML >> 8);       //Populate the array one byte at a time
	SendDataArray[4] = PWML;              // Revised By: Daniel Hong November 5 2018
    
    SendDataArray[5] = DirectionR;       //Set right directions Revised By: Daniel Hong November 5 2018
    SendDataArray[6] = DirectionL;       //Set left direction Revised By: Daniel Hong November 5 2018

    SendDataArray[7] = StartAutonomous;
    StartAutonomous = 0;

    SendDataArray[8] = (ONTime >> 8);       //Populate the array one byte at a time
	SendDataArray[9] = ONTime;              // Revised By: Daniel Hong November 5 2018

    SendDataArray[20] = 1;                  //Sending a 1 for robot to check for communication (i.e. new data)
    Communicating = ReceiveDataArray[20];   //Checking if the robot sent us a 1, which will indicate communication
    
    x = (ReceiveDataArray[1] << 8) + ReceiveDataArray[2];
    xx = (ReceiveDataArray[3] << 8) + ReceiveDataArray[4];
    
    if(Communicating){                      //If there is communication, reset the communication loss counter
        CommsLossCount = 0;
    }
    else if(!Communicating){                //If there is an interruption (i.e. single loss of communication),
        CommsLossCount++;                   //then increment the communication loss counter
    }
    ReceiveDataArray[20] = 0;               //Reset the communication to 0, If next time we look at it and it's 
                                            //still 0, then no communication. If next time we look at it and
                                            //robot has changed it to 1, then we have communication
}
/*****************************************************************************************************************/
void SendData(void)
{
	for (i=0;i<BUFF_SIZE;i++)           //Index through the array from the start to the end of the array 
	{                                   //Note: The first byte is an ASCII Character "s" to denote the start
		WriteUART1(SendDataArray[i]);	//Send one byte of the data array to UART TX Pin
		while(BusyUART1());             //Wait while the UART1 is busy (sending the last byte)
	}
}
/*****************************************************************************************************************/
void Shutdown(void){
    //This function is called when there's a communication loss. When communication is lost then the last values
    //received (ReceiveDataArray) will not change, so if any motors are running they will continue to run and may
    //cause problems. Therefore, enter your code to disable/stop anything that could potentially keep running
    
    LATAbits.LATA4 = 1;                 //Turn on communication error LED 
}
/*****************************************************************************************************************/
// UART1 Receive Interrupt
void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void) 
{
	DisableIntU1RX;             //Disable the UART1 receive interrupt
	IFS0bits.U1RXIF = 0;        //Reset the UART1 receive interrupt flag
	ReceivedChar = U1RXREG;     //Store the latest received character

//Need to synchronize the data being received by looking for the 's' which denotes the start of the array
    if ((uartcount == 0) && (ReceivedChar == 's'))  //Note: uartcount=0 until we receive a 's'
    {
        ReceiveDataArray[uartcount] = ReceivedChar; //Store 's' into the 0 element of the array
        uartcount++;                                //Increment array index for next byte being received
    }
    else if (uartcount != 0)
//Data has been synchronized; update the array of data being received until buffer size has been reached
    {
        ReceiveDataArray[uartcount] = ReceivedChar; //Update array with the latest data byte being received
        uartcount++;                                //Increment array index for next byte being received
        if (uartcount==BUFF_SIZE) uartcount=0;      //All data in array has been received
    }
     EnableIntU1RX;                                 //Enable the UART1 receive interrupt
}