/*================================================================================================================
 Program4-Robot.c;	(Note: Program for the Robot PIC24H microcontroller) 

 A Photocell is connected to AN4 with the ADC results being stored into a variable
 (Photocell) which will be added to an array (SendDataArray). This array will
 utilize the UART1 hardware module to send this data through the TX pin to a HC-05
 Bluetooth Module. Another HC-05 Bluetooth Module which is paired will be connected to 
 the Controller PIC24H microcontroller. The Robot PIC24H microcontroller will also utilize
 the UART1 module to receive data through the RX pin connected to the HC-05 Bluetooth Module
 and store this data into an array (ReceiveDataArray). Values from this array will be
 stored into a variable (ONTime) and be used to adjust the duration of Timer1's interrupt.
 This in turn sets the ON Time of the square wave signal sent to the RC Servo Motor
 connected to RB6.  The ON Time, also referred to as Positive Width, will determine the
 angular position of the RC Servo Motor assuming the OFF Time is set properly. Also,
 included in this code is a Map function which will limit the RC Servo Motor's position
 within its operating range (i.e. 0 to 180 degrees) by using define statements for the
 appropriate values for these limits (Note: these limit values may need to be
 adjusted for different RC Servo Motors).  The OFF time is also fixed at a value that
 represents ~19ms. (Note: Map function implemented to set limits of RC Servo Motor)

 Note: Communications is setup for 115200 Baud, 8 Data Bits, 1 Stop Bit and No Parity Bit

 For information on PIC24H Compiler Peripheral Libraries refer to link below:
 file:///C:/Program%20Files%20(x86)/Microchip/xc16/v1.32/docs/periph_libs/16-bit%20Peripheral%20Libraries.htm
=================================================================================================================*/
#include "p24HJ128GP502.h"  //Include device library file
#include "timer.h"          //Include TIMER library file
#include "uart.h"           //Include UART library file
#include "adc.h"            //Include ADC library file
#include "outcompare.h"		//Include Output Compare (PWM) library file
#define FCY     3685000     //Defines device instruction frequency which is needed
                            //for Software Delay Routine (i.e. __delay_ms(250)
#include "libpic30.h"       //Includes delay library file

#define BUFF_SIZE 32
#define RMOTORDIRECTIONPORT LATBbits.LATB10
#define LMOTORDIRECTIONPORT LATBbits.LATB12

//Configuration Bits
_FOSCSEL(FNOSC_FRC & IESO_OFF);
_FOSC	(POSCMD_NONE & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSDCMD);
_FWDT	(FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR32 & WDTPOST_PS1);
_FPOR	(FPWRT_PWR1 & ALTI2C_ON);
_FICD	(ICS_PGD1 & JTAGEN_OFF);

//Function prototypes
void InitIO (void);									
void InitTimer (void);				
void InitUART(void);
void ADC (void);			
void ProcessData();
void SendData(void);
void Shutdown(void);				
void InitPWM(void);		
void Drive(void);
void Autonomous(void);


unsigned int PWMReceivedDC = 0;            //Variable used to store value of ONTime while it is being received and mapped

unsigned int OFFTime = 3000;                //Variable used to control the period of the square wave	
unsigned char SendDataArray[BUFF_SIZE];     //Array to store the data to be transmitted through UART1 TX pin
unsigned char ReceiveDataArray[BUFF_SIZE];  //Array to store the data received from UART1 RX pin
unsigned char ReceivedChar = 0;             //Variable used to store the last byte received from UART1 RX pin
int i;                                      //Variable used to index through arrays
int uartcount = 0;                          //The current array index of the received data from the UART RX pin

unsigned int CommsLossCount = 0;            //Store the number of times there was a communication loss
unsigned int PWML = 65535;	//Initialize PWM1, used to store value of PWM Signal to motor 1
unsigned int PWMR = 65535;	//Initialize PWM2, used to store value of PWM Signal to motor 2
                            //Note the value of 65535 represents 100% Duty Cycle = motor off 
unsigned int PhotoLeft = 0;	//Initialize PWM2, used to store value of PWM Signal to motor 2
unsigned int PhotoRight = 0;	//Initialize PWM2, used to store value of PWM Signal to motor 2

int main (void)
{
	InitIO();                //Call InitIO which configures the input and output pins
	InitPWM();
    InitUART();         //Call InitUART which configures the UART hardware module for communications
                        //with the HC-05 Bluetooth Module
	
	for (i=0; i<BUFF_SIZE; i++) SendDataArray[i] = 0;   //Initialize the array of chars to zero
	SendDataArray[0] = 's';                             //Set first element as 's' for data synchronization
                                                       //and for framing error checking
//    Autonomous();

	while (1) {            
        ProcessData();	//Call ProcessData to update variables for UART1 Communications
        SendData(); 	//Call SendData to send data through the UART1 TX pin to HC-05 Bluetooth Module
        
        Shutdown();             //You must define code in Shutdown function below

        if(CommsLossCount>200){     //If communication loss persists, we assume complete loss of communication
            Shutdown();             //You must define code in Shutdown function below
        }
        else{                       //If there is communication, then the following code block will be executed
                                    //This is where all your functions should be executed
            LATAbits.LATA4 = 0;     //Turn off Communication loss LED
            ADC();
            Drive();
        }            
    }
    
    return 0;
}
/*****************************************************************************************************************/
void Autonomous(void) {    
    ADC();
    unsigned int threshold = 0;
    unsigned int endthreshold = 20000;
    int balance = 0;

    while (1) {
        
        ADC();
        if (PhotoLeft >= threshold && PhotoRight >= threshold) {
            balance = 0;
            PWML = PWMR = 20000;
        } else if (PhotoLeft < threshold && PhotoRight < threshold) {
            if (balance > 0) {
                balance = 2;
                PWML = 20000;
                PWMR = 65535;
            } else if (balance < 0) {
                balance = -2;
                PWML = 65535;
                PWMR = 20000;
            }
        } else {
            if (PhotoLeft < threshold && PhotoRight > threshold) {
                balance = 1;
                PWML = 20000;
                PWMR = 30000;
            } else if (PhotoRight < threshold && PhotoLeft > threshold) {
                balance = -1;
                PWML = 30000;
                PWMR = 20000;
            }
        }

        if (PhotoLeft > endthreshold && PhotoRight > endthreshold) break;

        ProcessData();
        
        if(CommsLossCount>200){     //If communication loss persists, we assume complete loss of communication
            Shutdown();             //You must define code in Shutdown function below
        }
        else{                       //If there is communication, then the following code block will be executed
                                    //This is where all your functions should be executed
            LATAbits.LATA4 = 0;     //Turn off Communication loss LED
            
            Drive();
        }           
    }

    PWML = PWMR = 65535;
    Drive();
}
/*****************************************************************************************************************/
void InitIO (void) {
    // Set motor inputs

    AD1PCFGLbits.PCFG12 = 1; //Set RB12 as Digital Instead of analog
	TRISBbits.TRISB13 = 0;	//Set RB13 as output (PWM Signal for Left Motor)
	TRISBbits.TRISB12 = 0;	//Set RB12 as output (Direction Signal for Left Motor)
	TRISBbits.TRISB11 = 0;	//Set RB11 as output (PWM Signal for Right Motor)	
    TRISBbits.TRISB10 = 0;	//Set RB10 as output (Direction Signal for Right Motor)
    
    RPOR6bits.RP13R = 18;	//Set RP13 as OC1 output on pin RB13
 	RPOR5bits.RP11R = 19;   //Set RP11 as OC2 output on pin RB11
   
    // Set line tracking sensor inputs
    
    TRISBbits.TRISB15 = 1;	
    TRISBbits.TRISB14 = 1;	

    // Set communication loss LED
    
    TRISAbits.TRISA4 = 0;   //Set RA4 as output for LED to indicate communication loss

    // Set bluetooth configuation
    
	//RP8 TO U1RX           //Set the RP8 pin to UART1 RX pin
	RPINR18bits.U1RXR = 8;	//See TABLE 11-1: SELECTABLE INPUT SOURCES (MAPS INPUT TO FUNCTION),Page 136
                            //and REGISTER 11-8: RPINR18: PERIPHERAL PIN SELECT INPUT REGISTER 18,Page 146
    CNPU2bits.CN22PUE = 1;	//Enable weak pull-up of Receive pin CN22 (Corresponds to RP8)
                            //This is needed for v1.06 Bluetooth boards to pull up the receive line
	//RP9 TO U1TX           //Set the RP9 pin to UART1 TX pin
	RPOR4bits.RP9R = 3;     //See TABLE 11-2: OUTPUT SELECTION FOR REMAPPABLE PIN (RPn), Page 137
                            //and REGISTER 11-19: RPOR4: PERIPHERAL PIN SELECT OUTPUT REGISTERS 4, Page 154
}
void InitPWM(void) {
	DisableIntT2;		//Disable Timer2 Interrupt
	DisableIntOC1;		//Disable OC1 Interrupt
	DisableIntOC2;		//Disable OC2 Interrupt
                        //Timer2 is the clock source for OC1 and OC2
                        //Configure PWM mode for OC1 and OC2
	OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC & OC_PWM_FAULT_PIN_DISABLE, 1, 1);
	OpenOC2(OC_IDLE_CON & OC_TIMER2_SRC & OC_PWM_FAULT_PIN_DISABLE, 1, 1);							
                        //Prescaler = 1:1 and Period = 0xFFFF
	OpenTimer2(T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_1 & T2_32BIT_MODE_OFF & T2_SOURCE_INT, 0xFFFF);
}
/*****************************************************************************************************************/
void InitUART(void) {
// For more information on PIC24H UART Peripheral Module Library Help refer to link below:
// file:///C:/Program%20Files%20(x86)/Microchip/xc16/v1.32/docs/periph_libs/dsPIC30F_dsPIC33F_PIC24H_dsPIC33E_PIC24E_UART_Help.htm
	IEC0bits.U1TXIE = 0;            //Disable UART1 TX interrupt
    IFS0bits.U1RXIF = 0;            //Clear the Receive Interrupt Flag
	U1MODEbits.STSEL = 0;           //1 Stop bit
	U1MODEbits.PDSEL = 0;           //8-bit data, no parity
	U1MODEbits.BRGH = 0;            //16x baud clock, Standard mode
	U1MODEbits.URXINV = 0;          //Idle State 1 for RX pin
	U1MODEbits.ABAUD = 0;           //Auto-Baud Disabled
	U1MODEbits.RTSMD = 1;           //Simplex Mode, no flow control
	U1MODEbits.UARTEN = 1;          //Enable UART1
	U1STAbits.UTXISEL0 = 0;         //Interrupt after one TX character is transmitted
	U1STAbits.UTXISEL1 = 0;         //Interrupt after one TX character is transmitted
	U1STAbits.UTXEN = 1;            //Enable UART1 to control TX pin
	U1BRG = 1;                      //BAUD Rate Setting for 115200
	IEC0bits.U1RXIE = 1;            //Enable UART1 RX interrupt
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
		ENABLE_AN9_ANA,
		0,		
		0,
		0);
                                    //Select AN4
	SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN9);
	AD1CON1bits.ADON = 1;           //Turn on ADC hardware module
	while (AD1CON1bits.DONE == 0);	//Wait until conversion is done
	PhotoLeft = ReadADC1(0);           //ONTime = converted results
	AD1CON1bits.ADON = 0;           //Turn off ADC hardware module

	OpenADC1(ADC_MODULE_OFF & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON,
                ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC,
		ADC_DMA_BUF_LOC_1,
		ENABLE_AN10_ANA,
		0,		
		0,
		0);
                                    //Select AN4
	SetChanADC1(0, ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN10);
	AD1CON1bits.ADON = 1;           //Turn on ADC hardware module
	while (AD1CON1bits.DONE == 0);	//Wait until conversion is done
	PhotoRight = ReadADC1(0);           //ONTime = converted results
	AD1CON1bits.ADON = 0;           //Turn off ADC hardware module
}
/*****************************************************************************************************************/
void ProcessData()
{	
    // Motor Speeds and directions
    PWMR = PWML = (ReceiveDataArray[1] << 8) + ReceiveDataArray[2];  //Build integer from array of bytes 

    LATBbits.LATB10 = ReceiveDataArray[3];
    LATBbits.LATB12 = ReceiveDataArray[4];
    
    SendDataArray[20] = 1;                  //Sending a 1 for controller to check for communication
    unsigned short Communicating = ReceiveDataArray[20];   //Checking if the controller sent us a 1, which will let us know if we

    SendDataArray[1] = PhotoLeft >> 8;
    SendDataArray[2] = PhotoLeft;
    SendDataArray[3] = PhotoRight >> 8;
    SendDataArray[4] = PhotoRight;

    if(Communicating){                      //If there is communication, reset the communication loss counter
        CommsLossCount = 0;
    }
    else if(!Communicating){                //If there is an interruption (i.e. single loss of communication),
        CommsLossCount++;                   //then increment the communication loss counter
    }
    ReceiveDataArray[20] = 0;               //Reset the communication to 0, If next time we look at it and it's 
                                            //still 0, then no communication. If next time we look at it and
                                            //controller has changed it to 1, then we have communication
}
/*****************************************************************************************************************/
void SendData(void)
{
	for (i=0;i<BUFF_SIZE;i++)           //Index through the array from the start to the end of the array 
	{                                   //Note: The first byte is an ASCII Character "s" to denote the start
    	WriteUART1(SendDataArray[i]);	//Send one byte of the data array to UART TX Pin
		while(BusyUART1());             //Wait while the UART1 is busy
	}
}
/*****************************************************************************************************************/
long Map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
/*****************************************************************************************************************/
void Shutdown(void){
    //Enter your code to disable/stop anything that could potentially keep running
    //This is incase of disconnect between Bluetooth modules
    //Generally, if a pair of modules lose communication, the main program will likely continue to
    //do what it was last told to do.
    //This function will therefore be called in the case that communication is lost
    
    LATAbits.LATA4 = 1;     //Turn on communication error LED 
    SetDCOC1PWM(65535);	    //Set duty cycle of left wheel
	SetDCOC2PWM(65535);     //Set duty cycle of right wheel
}
/*********************************************************************************************************/
void Drive(void)
{	
	SetDCOC1PWM(PWML);	//Set duty cycle PWM1
	SetDCOC2PWM(PWMR);	//Set duty cycle PWM2
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
     EnableIntU1RX;             //Enable the UART1 receive interrupt
}

