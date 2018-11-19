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
void ProcessData(int manual);
void SendData(void);
void Shutdown(void);				
void InitPWM(void);		
void Drive(void);
void Autonomous(void);

unsigned int PWML = 65535;	//Initialize PWM1, used to store value of PWM Signal to motor 1
unsigned int PWMR = 65535;	//Initialize PWM2, used to store value of PWM Signal to motor 2
                            //Note the value of 65535 represents 100% Duty Cycle = motor off 
unsigned int PhotoLeft = 0;	//Initialize PWM2, used to store value of PWM Signal to motor 2
unsigned int PhotoRight = 0;	//Initialize PWM2, used to store value of PWM Signal to motor 2

int main (void)
{
	InitIO();                //Call InitIO which configures the input and output pins
    ADC();
    
	InitPWM();
    
    PWML = 45000;
    PWMR = 45000;
    
    
    LATBbits.LATB10 = 0;
    LATBbits.LATB12 = 1;
    
    while(1) {
        ADC();
        Drive();
    }
    
    return 0;
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
    
    PWML = PWMR = 65535;
    Drive();
    
    // Set line tracking sensor inputs
    
    TRISBbits.TRISB15 = 1;	
    TRISBbits.TRISB14 = 1;	

    // Set communication loss LED
    
    TRISAbits.TRISA4 = 0;   //Set RA4 as output for LED to indicate communication loss
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
/*********************************************************************************************************/
void Drive(void)
{	
	SetDCOC1PWM(PWML);	//Set duty cycle PWM1
	SetDCOC2PWM(PWMR);	//Set duty cycle PWM2
}

