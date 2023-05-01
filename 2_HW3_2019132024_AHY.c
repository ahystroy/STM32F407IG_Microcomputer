//////////////////////////////////////////////////////////////////////
// USART RX Interrupt 
// USART1(PC): TX pin: PA9,  RX pin: PA10 
// UART4(BT):  TX pin: PC10, RX pin: PC11, RST pin(GPIO): PC13
// TX: Interrupt ���, RX: Interrupt ��� 
// ���ڸ� TX�� ���� PC(Hyper terminal)�� �����ϰ�, 
// PC���� ������ ���ڸ� �޾� LCD�� ǥ��
//
// PC(Hyper terminal)���� �Է��� ���ڸ� �޾�(LCD ǥ�õ�..) Mobilephone���� �����ϰ�, 
// Mobilephone���� ������ ���ڸ� �޾�(LCDǥ�õ� �ϰ�..) PC�� ����
//////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "default.h"
#include "Util.h"

#include "Que.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void _GPIO_Init(void);
uint16_t KEY_Scan(void);

//! Process
void ExtTemp_ADC2_process(void);
void IntTemp_ADC1_process(void);

// LCD ���� ȭ��
void Display_ExtTemp_Screen(void); // �ܺ� �µ� ���� ȭ��
void Display_IntTemp_Screen(void); // ���� �µ� ���� ȭ��
uint8_t mode; // ��� ���� ���� (0=�ܺοµ�, 1=���οµ�)

// VR ADC (�ܺοµ�)
void _ADC_Init(void);
uint16_t ADC_Value, Voltage, Ext_Temp, Int_Temp; // ���� ���� ���� ����
uint8_t txTemp[2];

// ADC2 ���۽�ȣ�� Timer1
void TIMER1_OC_Init(void);
uint8_t flag_250ms;

// UART4 Bluetooth 
void UART4_Init(void);
void UART4_BRR_Configuration(uint32_t USART_BaudRate);
void SerialSendChar(uint8_t c);
void SerialSendString(char *str);
char rx_data; // Bluetooth�� ���� �ڵ������κ��� ���� ������ ����


void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

int main(void)
{
  // ��� ��ȯ�� LCD ��� ������ �� ���ʸ� ����Ǳ� ���� flag ���� 
    uint8_t ExtTemp_enter = 1;
    uint8_t IntTemp_enter = 1;
    
    LCD_Init();	// LCD ���� �Լ�
    DelayMS(100);	// LCD���� ������

    _GPIO_Init();
    GPIOG->ODR &= 0x00;	// LED0~7 Off 
    UART4_Init();
    
    Display_ExtTemp_Screen();	// �ʱ�(reset)���� �ܺοµ� ���� ȭ�� ǥ��
    BEEP();
    
    _ADC_Init();
    //ADC1->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30)
    ADC2->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30)
    TIMER1_OC_Init();
    
    while(1)
    {
      if (mode==0 /*&& flag_250ms*/) { // �ܺοµ� �������, 250ms���� ����. 
        flag_250ms = 0;
        if (ExtTemp_enter){ // ��� ��ȯ �� �ʱ� ���Խ�,
          Display_ExtTemp_Screen(); // ���� �µ� ǥ��ȭ�� ����
          ExtTemp_enter = 0;
          IntTemp_enter = 1;
        }
        //ADC
      }
      else if (mode) { // ���οµ� �������
        if (IntTemp_enter){ // ��� ��ȯ �� �ʱ� ���Խ�,
          Display_IntTemp_Screen(); // ���� �µ� ǥ��ȭ�� ����
          IntTemp_enter = 0;
          ExtTemp_enter = 1;
        }
        //IntTemp_ADC1_process();
      }
      
            switch(KEY_Scan())
            {
       break;
        case SW1_PUSH : 		//SW1
                            GPIOG->ODR |= 0x02;	// LED1 On
                            mode = 1;
                            //IntTemp_ADC1_process();
                            
                            //Starts conversion of regular channels
                            //ADC1->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30)
                            //ADC2->CR2 &= ~(1<<30);
        break;
        case SW2_PUSH : 		//SW2
                            GPIOG->ODR |= 0x04;	// LED2 On
                            mode = 0;
                            Display_ExtTemp_Screen();
                            
                            //Starts conversion of regular channels
                            //ADC2->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30)
                            //ADC1->CR2 &= ~(1<<30);
        break;
            }
    }   
    
}

/****! �ܺοµ� ADC ���μ��� !****/
void ExtTemp_ADC2_process(void)
{
  // (ADC2 = Continuous ConvMode)
  ADC_Value = ADC2->DR;   	// Read ADC result value from ADC Data Reg(ADC2->DR) 

  //! ���а� LCD�� ǥ��
  Voltage = ADC_Value * (3.3 * 100) / 4095;   // 3.3V: 4095 =  Volatge : ADC_Value 
  LCD_DisplayChar(3,15,Voltage/100 + 0x30);
  LCD_DisplayChar(3,17,Voltage%100/10 + 0x30);

  //! �µ� LCD�� ǥ��
  Ext_Temp = (3.5*Voltage*Voltage/10000 + 1); // ������ ���а����� �µ� ����
  LCD_DisplayChar(3,10,Ext_Temp/10 + 0x30);   // �µ� ���� : 1~39
  LCD_DisplayChar(3,11,Ext_Temp%10 + 0x30);
  sprintf(txTemp, "%d", Ext_Temp); // �µ����� ���ڿ��� ��ȯ�ؼ�
  SerialSendString(txTemp); // �ڵ������� UART4(Bluetooth)�� �̿��Ͽ� Transmit

  //! �µ� ũ�⸦ ��Ÿ���� ���� �׷��� ǥ��
  int pixel = Ext_Temp * 139 / 39; // �µ��� ����Ͽ� ���� ���� ����
  LCD_SetPenColor(GET_RGB(0,200,55));  // �׵θ��� £�� GREEN
  LCD_DrawRectangle(10,47,pixel,12); // ���� �׵θ� �׸���

  // ���� �� ���� �� �׸���
  if (Ext_Temp <= 15)  // �µ��� 1~15�� �� ��,
    LCD_SetBrushColor(RGB_BLUE); // �Ķ��� ����
  else if (Ext_Temp >= 16 && Ext_Temp <= 27)  // �µ��� 16~27�� �� ��,
    LCD_SetBrushColor(RGB_GREEN); // �ʷϻ� ����
  else if (Ext_Temp >= 28 && Ext_Temp <= 39)  // �µ��� 28~39�� �� ��,
    LCD_SetBrushColor(RGB_RED); // ������ ����
  LCD_DrawFillRect(11,48,pixel-1,11); // ���� �׸���

  LCD_SetBrushColor(RGB_WHITE); // ��¿��� �������� �������� ��ĥ�ߴ� �κ��� �Ͼ���� �������μ� ����.
  LCD_DrawFillRect(11+pixel,47,139-pixel,13);
}

/****! ���οµ� ADC1 ���μ��� !****/
void IntTemp_ADC1_process(void)
{
  ADC_Value = ADC1->DR;		// Reading ADC result
  Int_Temp = (ADC_Value-0.76)/2.5 + 25;

  LCD_DisplayChar(3,10,Int_Temp/10 + 0x30);   // �µ� ���� : 0~70
  LCD_DisplayChar(3,11,Int_Temp%10 + 0x30);
}

void _GPIO_Init(void)
{
	// LED (GPIO G) ����
    RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) ���� 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) ���� 
    RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}

// ADC1 & ADC2 Init
void _ADC_Init(void)
{   	
        //! ADC1, ADC2 Clock Enalbe
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// (1<<8) ENABLE ADC1 CLK (stm32f4xx.h ����)
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;	// (1<<9) ENABLE ADC2 CLK (stm32f4xx.h ����)

        //! ADC CCR Init
	ADC->CCR &= ~(0x1F<<0);		// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= (1<<16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
        ADC->CCR |= (1<<23);            // TSVREFE=1: Temperature sensor channel enabled

        //! ADC1 Init
	ADC1->CR1 &= ~(3<<24);		// RES[1:0]=0b00 : 12bit Resolution
	ADC1->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable

	ADC1->CR2 |= (1<<1);		// CONT=1: ADC_Continuous ConvMode Enable
	ADC1->CR2 &= ~(3<<28);		// EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge_None
	ADC1->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
	ADC1->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions
        
        // interrupt source
        ADC1->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC	
        ADC1->CR2 &= ~(0x0F<<24);	// 0000: Timer 1 CC1 event
        ADC1->CR2 |=  (2<<28);          // EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
        
	ADC1->SQR1 &= ~(0xF<<20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
					// 0b0000:1 conversion)
        ADC1->SQR3 &= ~(0x1F<<0);
        ADC1->SQR3 |= (16<<0);		// SQ1[4:0] : CH16
    
        ADC1->SMPR1 |= (0x7<<18);	// ADC1_CH16 Sample Time_480Cycles
        
	
        
        //! ADC2 Init - PA1(pin 41)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h ����)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
        
	ADC2->CR1 &= ~(3<<24);		// RES[1:0]=0b00 : 12bit Resolution
	ADC2->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable

	ADC2->CR2 |= (1<<1);		// CONT=1: ADC_Continuous ConvMode Enable
	ADC2->CR2 &= ~(3<<28);		// EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge_None
	ADC2->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
	ADC2->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions
        
        // interrupt source
        ADC2->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC
        ADC2->CR2 |=  (2<<28);		// EXTEN[1:0]: ADC_ExternalTrigConvEdge_Enable(Falling Edge)
	ADC2->CR2 &= ~(0x0F<<24);	// 0000: Timer 1 CC1 event

	ADC2->SQR1 &= ~(0xF<<20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
					// 0b0000:1 conversion)
    
	// Channel selection, The Conversion Sequence of PIN1(ADC2_CH1) is first, 
	// Config sequence Range is possible from 0 to 17
	ADC2->SQR3 |= (1<<0);		// SQ1[4:0]=0b0001 : CH1

        //! only ADC2 ON !//
        ADC1->CR2 &= ~(1<<0);		// ADON=1: ADC1 OFF
	ADC2->CR2 |= (1<<0);		// ADON=1: ADC2 ON
}

void ADC_IRQHandler(void)
{
  if(ADC1->SR) {
          ADC1->SR &= ~(1<<1);		// EOC flag clear
          IntTemp_ADC1_process();
          
        } 
  else if(ADC2->SR) {
          ADC2->SR &= ~(1<<1);		// EOC flag clear
          ExtTemp_ADC2_process();
          
          }
          
          
     
}

// BlueTooth(UART4) Init
void UART4_Init(void)
{
	// UART4 : TX(PC10)
	RCC->AHB1ENR	|= (1<<2);	// RCC_AHB1ENR GPIOC Enable
	GPIOC->MODER	|= (2<<2*10);	// GPIOC PIN10 Output Alternate function mode					
	GPIOC->OSPEEDR	|= (3<<2*10);	// GPIOC PIN10 Output speed (100MHz Very High speed)
	GPIOC->PUPDR 	|= (1<<2*10);	// GPIOC  PIN10 : Pull-up   
	GPIOC->AFR[1]	|= (8<<4*(10-8));	// Connect GPIOC pin10 to AF8(UART4)
    
	// UART4 : RX(PC11)
	GPIOC->MODER 	|= (2<<2*11);	// GPIOC PIN11 Output Alternate function mode
	GPIOC->OSPEEDR	|= (3<<2*11);	// GPIOC PIN11 Output speed (100MHz Very High speed
	GPIOC->PUPDR 	|= (1<<2*11);	// GPIOC PIN11 Pull-up   
	GPIOC->AFR[1]	|= (8<<4*(11-8));	// Connect GPIOC pin11 to AF8(UART4)

	// UART4 : RST(PC13) : GPIO
	GPIOC->MODER	|= (1<<2*13);	// GPIOC PIN13 Output mode					
	GPIOC->OSPEEDR	|= (3<<2*13);	// GPIOC PIN13  Output speed (100MHz Very High speed)
	GPIOC->ODR 	    |= (1<<13);	    // BT Reset

	RCC->APB1ENR	|= (1<<19);	// RCC_APB1ENR UART4 Enable
    
	UART4_BRR_Configuration(9600); // USART Baud rate Configuration
    
	UART4->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	UART4->CR1	&= ~(1<<10);	// NO USART_Parity

	UART4->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	UART4->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
	UART4->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	UART4->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
    
	UART4->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	UART4->CR1 	&= ~(1<<7);	// 0x0080, TXE interrupt Disable

	NVIC->ISER[1]	|= (1<<(52-32));// Enable Interrupt UART4 (NVIC 52��)
	UART4->CR1 	|= (1<<13);	//  0x2000, UART4 Enable
}

// UART4 Bluetooth
void UART4_IRQHandler(void)	
{       
	if ( (UART4->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
    // #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
		rx_data = UART4->DR;	// ���ŵ� ���� ����
                if (rx_data == '0') { // �ܺοµ�
                  mode = 0;
                  ADC1->CR2 &= ~(1<<0);		// ADON=1: ADC1 OFF
                  ADC2->CR2 |= (1<<0);		// ADON=1: ADC2 ON
                }
                else if (rx_data == '1') { // ���οµ�
                  mode = 1;
                  ADC1->CR2 |= (1<<0);		// ADON=1: ADC1 ON
                  ADC2->CR2 &= ~(1<<0);		// ADON=1: ADC2 OFF
                }
	}
        // DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
}

// UART4 Baud rate ����
void UART4_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB1clock = 42000000;	//PCLK1_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Find the integer part 
	if ((UART4->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
        //  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
    	{       // UART4->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)  
	}
	else    // UART4->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));  // ���Ŀ� 100�� ���� ����(�Ҽ��� �ι�°�ڸ����� �����ϱ� ����)    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Find the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((UART4->CR1 & USART_CR1_OVER8) != 0)	
	{	// 8 oversampling
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else	// 16 oversampling
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to UART BRR register
	UART4->BRR = (uint16_t)tmpreg;
}


void SerialSendChar(uint8_t Ch) // 1���� ������ �Լ�
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty���� ������ ��� ���(�۽� ������ ���±��� ���)
    	while((UART4->SR & USART_SR_TXE) == RESET); 
	UART4->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}

void SerialSendString(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialSendChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}

/*
// ADC ���۽�ȣ CC event (250ms Interrupt)
void TIMER1_OC_Init(void)
{
// Time base ����
	RCC->APB2ENR |= (1<<0);	// 0x01, RCC_APB2ENR TIMER1 Enable

	// Setting CR1 
	TIM1->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM1->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                            //  Counter Overflow/Underflow, 
                            //  Setting the UG bit Set,
                            //  Update Generation through the slave mode controller 
                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM1->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
                            //	Counter Overflow/Underflow, 
                            // Setting the UG bit Set,
                            //	Update Generation through the slave mode controller 
                            // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM1->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM1->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enalbe 
	TIM1->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM1->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)
        
        // Setting BDTR.MOE = 1
        TIM1->BDTR |= (1<<15);

	// Setting the Period
	TIM1->PSC = 1680-1;	// Prescaler=168, 168MHz/1680 = 0.1MHz (10us)
	TIM1->ARR = 25000-1;	// Auto reload  : 10us * 25k = 250ms(period)

	// Update(Clear) the Counter
	TIM1->EGR |= (1<<0);    // UG: Update generation    

// Output Compare ����
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM1->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output
	TIM1->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM1->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable
	TIM1->CCMR1 |= (3<<4);	// OC1M=0b011 (Output Compare 1 Mode : toggle)
				// OC1REF toggles when CNT = CCR1
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM1->CCER &= ~(1<<4);	// CC1E=0: CC1 channel Output Enable
				// OC1(TIM1_CH1) disactive: �ش����� ���� ��ȣ��� disable
	TIM1->CCER &= ~(1<<5);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1���� �������� ���)  
        
	TIM1->CCR1 = 100;	// TIM1 CCR1 TIM1_Pulse

	//TIM1->DIER |= (1<<0);	// UIE: Enable Tim1 Update interrupt
	TIM1->DIER |= (1<<1);	// CC1IE: Enable the Tim1 CC1 interrupt

	NVIC->ISER[0] 	|= (1<<27);	// Enable TIM1_CC global Interrupt on NVIC

	TIM1->CR1 |= (1<<0);	// CEN: Enable the Tim1 Counter  					
}*/

void TIMER1_OC_Init(void)
{
        // TIM1_CH1 (PA8) : 250ms ���ͷ�Ʈ �߻�
        // Clock Enable : GPIOA & TIMER1
	RCC->AHB1ENR	|= (1<<0);	// GPIOA Enable
	RCC->APB2ENR 	|= (1<<0);	// TIMER1 Enable 
    						
        // PA2�� ��¼����ϰ� Alternate function(TIM5_CH3)���� ��� ���� 
	GPIOA->MODER 	|= (2<<8*2);	// PA8 Output Alternate function mode					
	GPIOA->OSPEEDR 	|= (3<<8*2);	// PA8 Output speed (100MHz High speed)
	GPIOA->OTYPER	&= ~(1<<8);	// PA8 Output type push-pull (reset state)
	GPIOA->AFR[1]	|= (1<<0); 	// (AFR[1].(3~0)=0b0001): Connect TIM1 pins(PA8) to AF1(TIM1/TIM2)
					// PA8 ==> TIM1_CH1

	// Assign 'Interrupt Period' and 'Output Pulse Period'
	TIM1->PSC = 1680-1;	// Prescaler 168MHz/1680 = 0.1MHz (10us)
	TIM1->ARR = 25000;	// Auto reload  : 10us * 25K = 250ms(period)

	// CR1 : Up counting
	TIM1->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM1->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM1->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
				//	- Counter Overflow/Underflow, 
				// 	- Setting the UG bit Set,
				//	- Update Generation through the slave mode controller 
	TIM1->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM1->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM1->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM1->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
				// Center-aligned mode: The counter counts Up and DOWN alternatively

	// Event & Interrup Enable : UI  
	TIM1->EGR |= (1<<0);    // UG: Update generation    
    
	// Define the corresponding pin by 'Output'  
	TIM1->CCER |= (1<<0);	// CC1E=1: CC1 channel Output Enable
				// OC1(TIM1_CH1) Active: �ش����� ���� ��ȣ���
	TIM1->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1���� �������� ���) 

	// 'Mode' Selection : Output mode, toggle  
	TIM1->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output 
	TIM1->CCMR1 &= ~(1<<3); // OC1P=0: Output Compare 3 preload disable
	TIM1->CCMR1 |= (3<<4);	// OC1M=0b011: Output Compare 3 Mode : toggle
				// OC1REF toggles when CNT = CCR1

 	TIM1->CCR1 = 15000;	// TIM1 CCR1 TIM1_Pulse
        
        TIM1->BDTR |= (1<<15);
      
	TIM1->CR1 |= (1<<0);	// CEN: Enable the Tim1 Counter				
}

/*
// T = 250ms Interrupt Handler
void TIM1_CC_IRQHandler(void)
{
  if((TIM1->SR & 0x02) != RESET)	// Capture/Compare 1 interrupt flag
  {
    TIM1->SR &= ~(1<<1);	// CC 1 Interrupt Claer
    flag_250ms = 1;
  }
}*/

// �ܺοµ� ���� ��� ȭ��
void Display_ExtTemp_Screen(void)
{
  LCD_Clear(RGB_WHITE);
  
  // ���簢�� �׵θ� �׸���
  LCD_SetPenColor(GET_RGB(0,200,55)); // �׵θ��� £�� GREEN
  LCD_DrawRectangle(5,5,149,59);
  
  // ���� ���� ���
  LCD_SetFont(&Gulim7); // ��Ʈ ����
  LCD_SetBackColor(RGB_WHITE);	// ���ڹ��� WHITE
  LCD_SetTextColor(RGB_BLACK);	// ���ڻ� BLACK
  LCD_DisplayText(1,2,"AHY 2019132024");
  LCD_DisplayText(2,2,"[External Temperature]");
  
  LCD_SetTextColor(RGB_BLUE);	// ���ڻ� BLUE
  LCD_DisplayText(3,2,"EXT TMP:  C (   V)");

  LCD_SetTextColor(RGB_RED);	// ���ڻ� RED
  LCD_DisplayText(3,16,".");    // �Ҽ���
}

// ���οµ� ���� ��� ȭ��
void Display_IntTemp_Screen(void)
{
  LCD_Clear(RGB_WHITE);
  
  // ���簢�� �׵θ� �׸��� �� ���� ä���
  LCD_SetPenColor(GET_RGB(0,200,55)); // �׵θ��� £�� GREEN
  LCD_DrawRectangle(7,5,145,44); // �׵θ� �׸���
  LCD_SetBrushColor(RGB_YELLOW); // ���� YELLOW
  LCD_DrawFillRect(8,6,144,43); // ���� ä���
  
  // ���� ���� ���
  LCD_SetFont(&Gulim7); // ��Ʈ ����
  LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� YELLOW
  LCD_SetTextColor(RGB_BLACK);	// ���ڻ� BLACK
  LCD_DisplayText(1,2,"AHY 2019132024");
  LCD_DisplayText(2,2,"[Internal Temperature]");
  
  LCD_SetTextColor(RGB_BLUE);	// ���ڻ� BLUE
  LCD_DisplayText(3,2,"INT TMP:  C");
  
  LCD_SetTextColor(RGB_RED);	// ���ڻ� RED
  //LCD_DisplayText(3,10,"3");
  //LCD_DisplayText(3,11,"8");
}


void DelayMS(unsigned short wMS)
{	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);  // 1000us => 1ms
}
void DelayUS(unsigned short wUS)
{	volatile int Dly = (int)wUS*17;
	for(; Dly; Dly--);
}
uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	if(key_flag == 0)
        		return key;
      		else
		{	DelayMS(10);
        		key_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
        		return 0xFF00;
      		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
}
void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~(1<<9);	// PF9 'L' Buzzer off
}