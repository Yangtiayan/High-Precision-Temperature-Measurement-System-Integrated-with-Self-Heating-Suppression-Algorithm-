/**
 *****************************************************************************
   @example  ADCMeter.c
   @brief    This Simple ADC example shows how to initialize the ADC1 for 
     continuous sampling using differential inputs.
   - Inputs selected are AIN3 for +ve and AIN2 for -ve
   - When using the ADuCM360 Evaluation board, it will digitize the ADC1 reading, convert it to 
     a voltage and send this to the UART.
   - ADC Gain of 4 is used
   - The DAC is also initialised to output 150mV.
   - The DAC output may be connected to AIN1, AIN0 to ground for test purposes 
   - Default Baud rate is 9600 

   @version V0.2
   @author  ADI
   @date    February 2013

   @par     Revision History:
   - V0.1, September 2012: initial version. 
   - V0.2, February 2013: Fixed a bug with ucTxBufferEmpty.
              
All files for ADuCM360/361 provided by ADI, including this file, are
provided  as is without warranty of any kind, either expressed or implied.
The user assumes any and all risk from the use of this code.
It is the responsibility of the person integrating this code into an application
to ensure that the resulting application performs as required and is safe.

**/

#include <stdio.h>
#include <string.h>
#include <aducm360.h>


#include <math.h>
#include <..\common\AdcLib.h>
#include <..\common\IexcLib.h>
#include <..\common\DacLib.h>
#include <..\common\UrtLib.h>
#include <..\common\ClkLib.h>
#include <..\common\WutLib.h>
#include <..\common\WdtLib.h>
#include <..\common\GptLib.h>
#include <..\common\I2cLib.h>
#include <..\common\IntLib.h>
#include <..\common\PwmLib.h>
#include <..\common\DioLib.h>

void IEXCINIT(void);  
void ADC0CalibrationINIT(void);
void ADC0MeasurementINIT(void);
void SendString(void);					          // Transmit string using UART
void UARTINIT (void);
void SendResultToUART(void);			          // Send measurement results to UART - in ASCII String format
double CalculateRTDTemp(double r);              // Calculates final temperature based on RTD resistance measurement
void delay(long int);
void T1INIT(void);	
void Presample(void);
void StartorWait(void);
void Mode_SELECTION(void);
void implement_algorithm(void);


unsigned char ucStart = 0; 

volatile unsigned char bSendResultToUART = 0;	// Flag used to indicate ADC0 resutl ready to send to UART
volatile unsigned int adc_count = 0;
unsigned char szTemp[64] = "";					// Used to store ADC0 result before printing to UART
unsigned char ucTxBufferEmpty  = 0;				// Used to indicate that the UART Tx buffer is empty
volatile  long ulADC0Result = 0;		        // Variable that ADC1DAT is read into in ADC1 IRQ
volatile unsigned char ucComRx = 0;
volatile unsigned char ucADC0ERR = 0;
double  fVoltage = 0.0;   			            // ADC value converted to voltage
double fVolts = 0.0;
uint16_t szADC0;
uint16_t syADC0;
volatile unsigned char ucADCERR = 0;      // Used to indicate an ADC error

volatile unsigned char StartToSample = 0; // Flag used to indicate Timer  ready to use the ADC to sample

unsigned char nLen = 0;                       // Used to calculate string length
unsigned char i = 0;   
unsigned char ucWaitForUart = 0;			// Used by calibration routines to wait for user input


//RTD constants


double error_continuous_mode0[30]={0.0201931330,0.029373268,0.033322948,0.036835346,0.038052017,0.039268688,0.04035176,0.040733443,0.041115125,0.041456818,
                            0.041588568,0.041720318,0.041839671,0.041893943,0.041948214,0.041998413,0.042027227,0.04205604,0.042083305,0.042102441,
                            0.042121577,0.042139985,0.042154571,0.042169156,0.042183303,0.042195149,0.042206995,0.042218522,0.042228377,0.042238232,};
							
double error_discontinuous_mode1[20]={0.008099174,0.012765339,0.015408945,0.016882059,0.017692937,0.018135974,0.018377214,0.018508562,0.018580185,0.0186197,
                              0.018641941,0.018655227,0.018663902,0.018670055,0.018675172,0.01869888,0.018733096,0.01877198,0.018811695,0.01883806};
							   
							   
double error_discontinuous_mode2[11]=  {0.008099175,0.009510575,0.009747819,0.00980214,0.009833832,0.00987254,0.00982421,0.009806153,0.009924692,0.009792939,0.00986339};

							   
							   
double error_discontinuous_mode3[6]={0.008099171,0.008527981,0.008537207,0.008630379,0.00854577,0.008581585};


int count_number=0;






#define A (3.9083E-03)
#define B (-5.775E-07)
#define C (-4.183E-012))
#define R0 (100)//100.2687
//   t=((-A)+sqrt(A*A-4*B*(1-r/R0)))/(2*B);

double fVRTD =0.0 ;								    // average RTD voltage, 
double fRrtd =0.0;								    // resistance of the RTD
double fTRTD =0.0;								    // RTD temperature
double fVRTD2 =0.0;					    // average RTD voltage 
int v=0;
volatile double x=0; 
int gg=0;
int cooling_time = 0; 
int currrent_continuous = 0;

    
int main (void)
{

   pADI_WDT ->T3CON = 0;
   DioOen(pADI_GP1,0xC);                        // Set P1.3 as an output for test purposes

  
         
    
   WdtCfg(T3CON_PRE_DIV1,T3CON_IRQ_EN,T3CON_PD_DIS); // Disable Watchdog timer resets
   //Disable clock to unused peripherals
   ClkDis(CLKDIS_DISSPI0CLK|CLKDIS_DISSPI1CLK|CLKDIS_DISI2CCLK|CLKDIS_DISPWMCLK|CLKDIS_DISDMACLK);
   ClkCfg(CLK_CD0,CLK_HF,CLKSYSDIV_DIV2EN_DIS,CLK_UCLKCG);            // Select CD0 for CPU clock
   ClkSel(CLK_CD6,CLK_CD7,CLK_CD0,CLK_CD7);     // Select CD0 for UART System clock
   AdcGo(pADI_ADC0,ADCMDE_ADCMD_IDLE);			// Place ADC0 in Idle mode
   IEXCINIT();	
   UARTINIT();									// Init Uart
   ADC0CalibrationINIT();									// Setup ADC0
   IexcCfg(IEXCCON_PD_off,IEXCCON_REFSEL_Int,IEXCCON_IPSEL1_Off,IEXCCON_IPSEL0_Off); //Setup IEXC) for AIN6  

//   T0INIT();
  // AdcGo(pADI_ADC0,ADCMDE_ADCMD_CONT);			// Start ADC0 for continuous conversions
   NVIC_EnableIRQ(ADC0_IRQn);					// Enable ADC1 and UART interrupt sources
   NVIC_EnableIRQ(UART_IRQn);

   
   
    StartorWait();
    
    ucStart = 0;  ///////

    sprintf ( (char*)szTemp, "which mode do you want to apply from mode 0 to mode 5? \r\n");    //[a]   //////
	nLen = strlen((char*)szTemp);/////
    if (nLen <64)/////
    SendString();//////

    sprintf ( (char*)szTemp, "Press the corresponding number to Start measurement \r\n");       //////                  
	nLen = strlen((char*)szTemp);//////
 	if (nLen <64)//////
	    SendString();//////
    
    
    Mode_SELECTION();

       T1INIT();
       NVIC_EnableIRQ(TIMER1_IRQn);                          // Enable Timer1 IRQ 
    
    
    Presample();  
    delay(0XFF);

    ADC0MeasurementINIT();
    bSendResultToUART=0;
    
    
    while (1)
   {
 
    if(StartToSample == 1)					      
	{    
    StartToSample =0;     


	IexcCfg(IEXCCON_PD_off,IEXCCON_REFSEL_Int,IEXCCON_IPSEL1_Off,IEXCCON_IPSEL0_AIN6); //Setup IEXC) for AIN6

    AdcGo(pADI_ADC0,ADCMDE_ADCMD_SINGLE);	   

    while (	bSendResultToUART == 0)					      // Wait for 16x samples to accumulate
    {}   

        if(currrent_continuous==1)
   { 
       currrent_continuous=1; //Setup IEXC) for AIN6         
   }
   else
   {
       IexcCfg(IEXCCON_PD_off,IEXCCON_REFSEL_Int,IEXCCON_IPSEL1_Off,IEXCCON_IPSEL0_Off);
   }
    bSendResultToUART = 0;


	fVRTD2=(double)ulADC0Result;

    fRrtd=(double)ulADC0Result*2500/268435456;     

 //  fTRTD=(1-0.0017)*CalculateRTDTemp(fRrtd)-0.1;//-0.023
   
    fTRTD=CalculateRTDTemp(fRrtd);//-0.023
   
    implement_algorithm();
      
    SendResultToUART();	
   
    DioTgl(pADI_GP1,0x8);


    }
   }
  
}


void SendResultToUART(void)
{   
/*    sprintf ( (char*)szTemp, "%lf  \r\n",fRrtd);   // [a]                      
   nLen = strlen((char*)szTemp);
   if (nLen <64)
     SendString();*/

   sprintf ( (char*)szTemp, "%lf  \r\n",fTRTD);    //[a]   
	nLen = strlen((char*)szTemp);
    
	if (nLen <64)
 		SendString();
}

void SendString (void)
{
   for ( i = 0 ; i < nLen ; i++ )	// loop to send ADC0 result
	{
  		 ucTxBufferEmpty = 0;
		 UrtTx(pADI_UART,szTemp[i]);
  		 while (ucTxBufferEmpty == 0)
  		 {
  		 }
	}
} 


double CalculateRTDTemp(double r) 
{
	double t;

   t=((-A)+sqrt(A*A-4*B*(1-r/R0)))/(2*B);
    
    return t;
}


void UARTINIT (void)
{
   //Select IO pins for UART.
   pADI_GP0->GPCON |= 0x3C;                     // Configure P0.1/P0.2 for UART
//    pADI_GP0->GPCON |= 0x9000;                   // Configure P0.6/P0.7 for UART
   UrtCfg(pADI_UART,B9600,COMLCR_WLS_8BITS,0);  // setup baud rate for 9600, 8-bits
   UrtMod(pADI_UART,COMMCR_DTR,0);              // Setup modem bits
   UrtIntCfg(pADI_UART,COMIEN_ERBFI|COMIEN_ETBEI|COMIEN_ELSI|COMIEN_EDSSI|COMIEN_EDMAT|COMIEN_EDMAR);  // Setup UART IRQ sources
}

void ADC0CalibrationINIT(void)
{
   
   AdcMski(pADI_ADC0,ADCMSKI_RDY,1);              // Enable ADC ready interrupt source		
   AdcFlt(pADI_ADC0,124,14,FLT_NORMAL|ADCFLT_CHOP_ON|ADCFLT_RAVG2_ON|ADCFLT_NOTCH2);  // ADC filter set for 3.75Hz update rate with chop on enabled
   AdcRng(pADI_ADC0,ADCCON_ADCREF_INTREF,ADCMDE_PGA_G4,ADCCON_ADCCODE_INT); 
   AdcBuf(pADI_ADC0,ADCCFG_EXTBUF_VREFPN,ADC_BUF_ON); 
   AdcPin(pADI_ADC0,ADCCON_ADCCN_AIN1,ADCCON_ADCCP_AIN0); // Select AIN1 as postive input and AIN0 as negative input
   AdcGo(pADI_ADC0,ADCMDE_ADCMD_IDLE);
  
    delay(0xFF); 

    pADI_ADC0->OF=0x0000;    
    szADC0 = pADI_ADC0->OF; 
 
    pADI_ADC0->INTGN=0x5555; 
    syADC0 = pADI_ADC0->INTGN;       
}

void ADC0MeasurementINIT(void)
{
   
    AdcMski(pADI_ADC0,ADCMSKI_RDY,1);                  // Enable ADC ready interrupt source		
    AdcFlt(pADI_ADC0,124,14,FLT_NORMAL|ADCFLT_CHOP_ON|ADCFLT_RAVG2_ON|ADCFLT_NOTCH2);  // AdcFlt(pADI_ADC0,125,0,FLT_NORMAL)
    AdcRng(pADI_ADC0,ADCCON_ADCREF_EXTREF,ADCMDE_PGA_G4,ADCCON_ADCCODE_INT); // External eference selected, Gain of 32, Signed integer output
	// Turn off input buffers to ADC and external reference											
    AdcBuf(pADI_ADC0,ADCCFG_EXTBUF_VREFPN,ADC_BUF_ON); 
    AdcPin(pADI_ADC0,ADCCON_ADCCN_AIN1,ADCCON_ADCCP_AIN0); // Select AIN0 as postive input and AIN1 as negative input  
    AdcGo(pADI_ADC0,ADCMDE_ADCMD_IDLE);			// Place ADC0 in Idle mode
  
}



void IEXCINIT(void)
{
	IexcDat(IEXCDAT_IDAT_1mA,IDAT0Dis);         // Set output for 1mA
	IexcCfg(IEXCCON_PD_off,IEXCCON_REFSEL_Int,IEXCCON_IPSEL1_Off,IEXCCON_IPSEL0_Off); //Setup IEXC) for AIN6
}

void T1INIT()
{
   GptLd(pADI_TM1,cooling_time); //0XFFF2  0xff00 2.048 // FF37  1.6   //FD43  5.6 //FABE 10.76//F857 15.68 // F5EB  20.64        // Set timeout period for 5 seconds
   GptCfg(pADI_TM1,TCON_CLK_LFOSC,TCON_PRE_DIV256,TCON_MOD_PERIODIC|TCON_UP|TCON_RLD|TCON_ENABLE);
}

void GP_Tmr1_Int_Handler(void)
{
    GptClrInt(pADI_TM1,TSTA_TMOUT);   
   GptLd(pADI_TM1,cooling_time); 
//    DioTgl(pADI_GP1,0x4);
       StartToSample = 1;
  
}

// Simple Delay routine
void delay (long int length)
{
   while (length >0)
      length--;
}

void Ext_Int2_Handler ()
{           

}
void Ext_Int4_Handler ()
{         
 
}
void GP_Tmr0_Int_Handler(void)
{

}


void ADC1_Int_Handler()
{
   
}
void ADC0_Int_Handler ()
{
                 // Set flag to indicate ready to send result to UART8*/
   volatile unsigned int uiADCSTA = 0;
   volatile long ulADC0DAT = 0;
    
    adc_count++;
   uiADCSTA = pADI_ADC0->STA;          // read ADC status register
   if ((uiADCSTA & 0x4) == 0x4)        // Check for ADC0TH exceeded error condition
   {
      ucADCERR = 1;
      pADI_ADC0->MSKI &= 0xFB;         // Disable Threshold detection to clear this interrupt source
      pADI_ADC0->PRO = 0;              // Disable comparator
   } 
    ulADC0DAT = AdcRd(pADI_ADC0);       // read ADC result register;
   ulADC0Result = ulADC0DAT;
   
      bSendResultToUART = 1;           // Set flag to indicate ready to send result to UART
}
void SINC2_Int_Handler()
{
 
}
void UART_Int_Handler ()
{
   volatile unsigned char ucCOMSTA0 = 0;
	volatile unsigned char ucCOMIID0 = 0;
	
	ucCOMSTA0 = UrtLinSta(pADI_UART);			// Read Line Status register
	ucCOMIID0 = UrtIntSta(pADI_UART);			// Read UART Interrupt ID register
	if ((ucCOMIID0 & 0x2) == 0x2)	  			// Transmit buffer empty
	{
	  ucTxBufferEmpty = 1;
	}
	if ((ucCOMIID0 & 0x4) == 0x4)	  			// Receive byte
	{
		ucComRx	= UrtRx(pADI_UART);
		ucWaitForUart = 0;
		if (ucComRx == 0xD)            ///////     // "Carriage return" detected
			ucStart = 1;////////
	}
} 

void PWMTRIP_Int_Handler ()
{           
 
}
void PWM0_Int_Handler()
{

}
void PWM1_Int_Handler ()
{
 
}
void PWM2_Int_Handler()
{
  
}

void Presample(void)
{
   
    for(v=0;v<3;v++)
{

   AdcGo(pADI_ADC0,ADCMDE_ADCMD_SINGLE);
   

   while (	bSendResultToUART == 0)					      // Wait for 16x samples to accumulate
    {}   
           
    bSendResultToUART=0;
    x=(double)ulADC0Result;
}

}


void StartorWait(void)
{
   
   ucStart = 0;  ///////

    sprintf ( (char*)szTemp, "Do you want to Start to measure? \r\n");    //[a]   //////
	nLen = strlen((char*)szTemp);/////
    if (nLen <64)/////
    SendString();//////

    sprintf ( (char*)szTemp, "Press 1 to Start \r\n");       //////                  
	nLen = strlen((char*)szTemp);//////
 	if (nLen <64)//////
	    SendString();//////
    
      while (ucStart == 0)
	{     
		if (ucComRx == 0x31)                                    // Character "1" received, so increase output current
		{
            ucComRx = 0;
		}
        else
        {
          ucComRx = 0;
        }

//         delay(0XFF);
	}

}

void Mode_SELECTION(void)
{

//      while (ucStart == 0)
      while (cooling_time == 0)         
	{     
        
       switch(ucComRx)
        {
      case 0x30:
            cooling_time=0xFF00; 
            currrent_continuous=1;
            ucComRx = 0;
            break;  
      case 0x31:
            cooling_time=0xFF37;
            currrent_continuous=0;      
            ucComRx = 0;
            break;
      case 0x32:
            cooling_time=0xFD43; 
            currrent_continuous=0;   
            ucComRx = 0;
            break;       
      case 0x33:
            cooling_time=0xFABE;
            currrent_continuous=0;         
            ucComRx = 0;
            break;    
      case 0x34:
            cooling_time=0xF857;
            currrent_continuous=0;         
            ucComRx = 0;
            break;      
      case 0x35:
            cooling_time=0xF5EB;
            currrent_continuous=0;         
            ucComRx = 0;
            break;    
      case 0x0A:
           ucComRx = 0;
           break;   
        }
    }
}

void implement_algorithm(void)
{    
switch(currrent_continuous)
{  
case 1: 
  if(count_number<30)
  {
  fTRTD=fTRTD-error_continuous_mode0[count_number];
  count_number++;
  } 
  else 
  {
  fTRTD=fTRTD-0.04225;
  }
  break;
  
case 0:
      switch(cooling_time)
{	  
  case 0xFF37: 
   if(count_number<20)
   {
    fTRTD=fTRTD-error_discontinuous_mode1[count_number];
    count_number++;
   } 
   else 
   {
    fTRTD=fTRTD-0.01885;
   }
  break;  
  
  case 0xFD43:  
   if(count_number<10)
   {
    fTRTD=fTRTD-error_discontinuous_mode2[count_number];
    count_number++;
   } 
   else 
   {
   fTRTD=fTRTD-0.0099;
   }
  break;  
  
    case 0xFABE:  
   if(count_number<6)
   {
    fTRTD=fTRTD-error_discontinuous_mode3[count_number];
    count_number++;
   } 
   else 
   {
   fTRTD=fTRTD-0.0086;
   }
  break;  

    case 0xF857:  
   if(count_number<1)
   {
    fTRTD=fTRTD-0.008099175;
    count_number++;
   } 
   else 
   {
   fTRTD=fTRTD-0.008239974;
   }
  break; 
  
    case 0xF5EB:  
   
    fTRTD=fTRTD-0.0081;

    break; 
}
break;
}   
   
}   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
