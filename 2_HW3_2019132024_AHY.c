//////////////////////////////////////////////////////////////////////
// USART RX Interrupt 
// USART1(PC): TX pin: PA9,  RX pin: PA10 
// UART4(BT):  TX pin: PC10, RX pin: PC11, RST pin(GPIO): PC13
// TX: Interrupt 방식, RX: Interrupt 방식 
// 문자를 TX를 통해 PC(Hyper terminal)로 전송하고, 
// PC에서 보내온 문자를 받아 LCD에 표시
//
// PC(Hyper terminal)에서 입력한 문자를 받아(LCD 표시도..) Mobilephone으로 전송하고, 
// Mobilephone에서 보내온 문자를 받아(LCD표시도 하고..) PC로 전송
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

// LCD 구동 화면
void Display_ExtTemp_Screen(void); // 외부 온도 측정 화면
void Display_IntTemp_Screen(void); // 내부 온도 측정 화면
uint8_t mode; // 모드 설정 변수 (0=외부온도, 1=내부온도)

// VR ADC (외부온도)
void _ADC_Init(void);
uint16_t ADC_Value, Voltage, Ext_Temp, Int_Temp; // 관련 저장 공간 마련
uint8_t txTemp[2];

// ADC2 시작신호용 Timer1
void TIMER1_OC_Init(void);
uint8_t flag_250ms;

// UART4 Bluetooth 
void UART4_Init(void);
void UART4_BRR_Configuration(uint32_t USART_BaudRate);
void SerialSendChar(uint8_t c);
void SerialSendString(char *str);
char rx_data; // Bluetooth를 통해 핸드폰으로부터 받은 데이터 저장


void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

int main(void)
{
  // 모드 전환시 LCD 배경 세팅을 한 차례만 진행되기 위해 flag 변수 
    uint8_t ExtTemp_enter = 1;
    uint8_t IntTemp_enter = 1;
    
    LCD_Init();	// LCD 구동 함수
    DelayMS(100);	// LCD구동 딜레이

    _GPIO_Init();
    GPIOG->ODR &= 0x00;	// LED0~7 Off 
    UART4_Init();
    
    Display_ExtTemp_Screen();	// 초기(reset)에는 외부온도 측정 화면 표시
    BEEP();
    
    _ADC_Init();
    //ADC1->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30)
    ADC2->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30)
    TIMER1_OC_Init();
    
    while(1)
    {
      if (mode==0 /*&& flag_250ms*/) { // 외부온도 측정모드, 250ms마다 진행. 
        flag_250ms = 0;
        if (ExtTemp_enter){ // 모드 변환 후 초기 진입시,
          Display_ExtTemp_Screen(); // 내부 온도 표시화면 설정
          ExtTemp_enter = 0;
          IntTemp_enter = 1;
        }
        //ADC
      }
      else if (mode) { // 내부온도 측정모드
        if (IntTemp_enter){ // 모드 변환 후 초기 진입시,
          Display_IntTemp_Screen(); // 내부 온도 표시화면 설정
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

/****! 외부온도 ADC 프로세스 !****/
void ExtTemp_ADC2_process(void)
{
  // (ADC2 = Continuous ConvMode)
  ADC_Value = ADC2->DR;   	// Read ADC result value from ADC Data Reg(ADC2->DR) 

  //! 전압값 LCD에 표시
  Voltage = ADC_Value * (3.3 * 100) / 4095;   // 3.3V: 4095 =  Volatge : ADC_Value 
  LCD_DisplayChar(3,15,Voltage/100 + 0x30);
  LCD_DisplayChar(3,17,Voltage%100/10 + 0x30);

  //! 온도 LCD에 표시
  Ext_Temp = (3.5*Voltage*Voltage/10000 + 1); // 측정된 전압값으로 온도 산출
  LCD_DisplayChar(3,10,Ext_Temp/10 + 0x30);   // 온도 범위 : 1~39
  LCD_DisplayChar(3,11,Ext_Temp%10 + 0x30);
  sprintf(txTemp, "%d", Ext_Temp); // 온도값을 문자열로 변환해서
  SerialSendString(txTemp); // 핸드폰으로 UART4(Bluetooth)를 이용하여 Transmit

  //! 온도 크기를 나타내는 막대 그래프 표시
  int pixel = Ext_Temp * 139 / 39; // 온도에 비례하여 막대 길이 설정
  LCD_SetPenColor(GET_RGB(0,200,55));  // 테두리색 짙은 GREEN
  LCD_DrawRectangle(10,47,pixel,12); // 막대 테두리 그리기

  // 막대 색 지정 후 그리기
  if (Ext_Temp <= 15)  // 온도가 1~15도 일 때,
    LCD_SetBrushColor(RGB_BLUE); // 파란색 막대
  else if (Ext_Temp >= 16 && Ext_Temp <= 27)  // 온도가 16~27도 일 때,
    LCD_SetBrushColor(RGB_GREEN); // 초록색 막대
  else if (Ext_Temp >= 28 && Ext_Temp <= 39)  // 온도가 28~39도 일 때,
    LCD_SetBrushColor(RGB_RED); // 빨간색 막대
  LCD_DrawFillRect(11,48,pixel-1,11); // 막대 그리기

  LCD_SetBrushColor(RGB_WHITE); // 고온에서 저온으로 내려갈때 색칠했던 부분을 하얀색을 덮음으로서 지움.
  LCD_DrawFillRect(11+pixel,47,139-pixel,13);
}

/****! 내부온도 ADC1 프로세스 !****/
void IntTemp_ADC1_process(void)
{
  ADC_Value = ADC1->DR;		// Reading ADC result
  Int_Temp = (ADC_Value-0.76)/2.5 + 25;

  LCD_DisplayChar(3,10,Int_Temp/10 + 0x30);   // 온도 범위 : 0~70
  LCD_DisplayChar(3,11,Int_Temp%10 + 0x30);
}

void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
    RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) 설정 
    RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}

// ADC1 & ADC2 Init
void _ADC_Init(void)
{   	
        //! ADC1, ADC2 Clock Enalbe
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// (1<<8) ENABLE ADC1 CLK (stm32f4xx.h 참조)
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;	// (1<<9) ENABLE ADC2 CLK (stm32f4xx.h 참조)

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
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
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

	NVIC->ISER[1]	|= (1<<(52-32));// Enable Interrupt UART4 (NVIC 52번)
	UART4->CR1 	|= (1<<13);	//  0x2000, UART4 Enable
}

// UART4 Bluetooth
void UART4_IRQHandler(void)	
{       
	if ( (UART4->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
    // #define  USART_SR_RXNE ((uint16_t)0x0020)    //  Read Data Register Not Empty     
	{
		rx_data = UART4->DR;	// 수신된 문자 저장
                if (rx_data == '0') { // 외부온도
                  mode = 0;
                  ADC1->CR2 &= ~(1<<0);		// ADON=1: ADC1 OFF
                  ADC2->CR2 |= (1<<0);		// ADON=1: ADC2 ON
                }
                else if (rx_data == '1') { // 내부온도
                  mode = 1;
                  ADC1->CR2 |= (1<<0);		// ADON=1: ADC1 ON
                  ADC2->CR2 &= ~(1<<0);		// ADON=1: ADC2 OFF
                }
	}
        // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다. 즉 clear 할 필요없음 
}

// UART4 Baud rate 설정
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
		integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)  
	}
	else    // UART4->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)    
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


void SerialSendChar(uint8_t Ch) // 1문자 보내기 함수
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
    	while((UART4->SR & USART_SR_TXE) == RESET); 
	UART4->DR = (Ch & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendString(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialSendChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
}

/*
// ADC 시작신호 CC event (250ms Interrupt)
void TIMER1_OC_Init(void)
{
// Time base 설정
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

// Output Compare 설정
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM1->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output
	TIM1->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM1->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable
	TIM1->CCMR1 |= (3<<4);	// OC1M=0b011 (Output Compare 1 Mode : toggle)
				// OC1REF toggles when CNT = CCR1
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM1->CCER &= ~(1<<4);	// CC1E=0: CC1 channel Output Enable
				// OC1(TIM1_CH1) disactive: 해당핀을 통해 신호출력 disable
	TIM1->CCER &= ~(1<<5);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1으로 반전없이 출력)  
        
	TIM1->CCR1 = 100;	// TIM1 CCR1 TIM1_Pulse

	//TIM1->DIER |= (1<<0);	// UIE: Enable Tim1 Update interrupt
	TIM1->DIER |= (1<<1);	// CC1IE: Enable the Tim1 CC1 interrupt

	NVIC->ISER[0] 	|= (1<<27);	// Enable TIM1_CC global Interrupt on NVIC

	TIM1->CR1 |= (1<<0);	// CEN: Enable the Tim1 Counter  					
}*/

void TIMER1_OC_Init(void)
{
        // TIM1_CH1 (PA8) : 250ms 인터럽트 발생
        // Clock Enable : GPIOA & TIMER1
	RCC->AHB1ENR	|= (1<<0);	// GPIOA Enable
	RCC->APB2ENR 	|= (1<<0);	// TIMER1 Enable 
    						
        // PA2을 출력설정하고 Alternate function(TIM5_CH3)으로 사용 선언 
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
				// OC1(TIM1_CH1) Active: 해당핀을 통해 신호출력
	TIM1->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1으로 반전없이 출력) 

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

// 외부온도 측정 모드 화면
void Display_ExtTemp_Screen(void)
{
  LCD_Clear(RGB_WHITE);
  
  // 직사각형 테두리 그리기
  LCD_SetPenColor(GET_RGB(0,200,55)); // 테두리색 짙은 GREEN
  LCD_DrawRectangle(5,5,149,59);
  
  // 고정 글자 출력
  LCD_SetFont(&Gulim7); // 폰트 설정
  LCD_SetBackColor(RGB_WHITE);	// 글자배경색 WHITE
  LCD_SetTextColor(RGB_BLACK);	// 글자색 BLACK
  LCD_DisplayText(1,2,"AHY 2019132024");
  LCD_DisplayText(2,2,"[External Temperature]");
  
  LCD_SetTextColor(RGB_BLUE);	// 글자색 BLUE
  LCD_DisplayText(3,2,"EXT TMP:  C (   V)");

  LCD_SetTextColor(RGB_RED);	// 글자색 RED
  LCD_DisplayText(3,16,".");    // 소수점
}

// 내부온도 측정 모드 화면
void Display_IntTemp_Screen(void)
{
  LCD_Clear(RGB_WHITE);
  
  // 직사각형 테두리 그리기 및 배경색 채우기
  LCD_SetPenColor(GET_RGB(0,200,55)); // 테두리색 짙은 GREEN
  LCD_DrawRectangle(7,5,145,44); // 테두리 그리기
  LCD_SetBrushColor(RGB_YELLOW); // 배경색 YELLOW
  LCD_DrawFillRect(8,6,144,43); // 배경색 채우기
  
  // 고정 글자 출력
  LCD_SetFont(&Gulim7); // 폰트 설정
  LCD_SetBackColor(RGB_YELLOW);	// 글자배경색 YELLOW
  LCD_SetTextColor(RGB_BLACK);	// 글자색 BLACK
  LCD_DisplayText(1,2,"AHY 2019132024");
  LCD_DisplayText(2,2,"[Internal Temperature]");
  
  LCD_SetTextColor(RGB_BLUE);	// 글자색 BLUE
  LCD_DisplayText(3,2,"INT TMP:  C");
  
  LCD_SetTextColor(RGB_RED);	// 글자색 RED
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