/////////////////////////////////////////////////////////////////////////
// PR1: �ڵ��� ���� �ý���
// ������: 2019132024 ��ȣ��
// �ֿ� ����
// - �Ÿ�����: ADC2_IN1(PA1) �̿� 
//          ==> ADC�� start trigger ��ȣ (t=300ms) ==> TIM1_CH3
// - ������ ���� : TIM2_CH4(PB11) �̿�  
//              ==> �����Ÿ��� ���� PWM ��ȣ�� DR ���� ==> LED�� Ȯ��
// - ���/���� ��� : USART1: PC���� ���
//                SW0(GPIO PH8), SW7(EXTI15): key �Է������� ���
// - ���� �� ���� �� ���� ����: FRAM �̿� (1126����)
/////////////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

void _GPIO_Init(void);
void DisplayTitle(void);

// ADC2 - �������� (�Ÿ�)
void _ADC2_Init(void);
void TIMER1_OC_Init(void);

// UART1 - PC�� ���
void USART1_Init(void);
void USART_BRR_Configuration(uint32_t USART_BaudRate);
void SerialSendChar(uint8_t c);
void SerialSendString(char *s);

// PWM - DR
void TIMER2_PWM_Init(void);

// EXTI15 (SW7)
void _EXTI_Init(void);

// ���� ���� ==> 1:MOVE, 0:STOP
int ms_flag;

uint16_t KEY_Scan(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

int main(void)
{
	LCD_Init();	  // LCD ���� �Լ�
	DelayMS(10);	  // LCD ���� ������
 	DisplayTitle();	  // LCD �ʱ�ȭ�鱸�� �Լ�
        
        //! Init Routine
        _GPIO_Init();
	_ADC2_Init();
        TIMER1_OC_Init();
        USART1_Init();
        TIMER2_PWM_Init();
        _EXTI_Init();
        Fram_Init();
        
        //! ���� �� ���õǸ� ������ ���� ����
        if (Fram_Read(1126) == 'M') {
          ms_flag = 1;
          LCD_DisplayChar(4,17,'M');
        }
        else if (Fram_Read(1126) == 'S') {
          ms_flag = 0;
          LCD_DisplayChar(4,17,'S');
        }
        
 	while(1)
	{
          switch(KEY_Scan()) {
          case SW0_PUSH: // SW0 ==> MOVE Key
            ms_flag = 1; 
            Fram_Write(1126, 'M');     // FRAM�� �� ���� ����
            LCD_DisplayChar(4,17,'M'); // 4th line ���� 'M' ǥ��
          }
	}
}

uint16_t ADC_Value, Voltage;
int distance; // �������� ���а����� ���� �Ÿ���
int pixel_x, pixel_y, distance_bar; // ���� �׸��� ���� ����
uint8_t distance_str[4]; // PC�� ���ڿ� ������ ���� ����
int DR; // DR�� ���� ����
void ADC_IRQHandler(void)
{
        if(ADC2->SR) {
          ADC2->SR &= ~(1<<1);		// EOC flag clear

          ADC_Value = ADC2->DR;		// Reading ADC result
          
          Voltage = ADC_Value * 3.3 * 100 / 4095;   // 3.3 : 4095 =  Volatge : ADC_Value 
                                                    // 100 = �Ҽ��� �Ʒ� �߸� ���� ����
          
          if (ms_flag) { //! MOVE ����
             // MOVE �����϶� �Ÿ�ǥ�ø� ��.
            distance = (Voltage*10)/100 + 5; // 5~38
            
             // �����Ÿ� ǥ��
            if (distance/10 == 0) LCD_DisplayChar(3,16,' ');
            else LCD_DisplayChar(3,16,distance/10+0x30);
            LCD_DisplayChar(3,17,distance%10+0x30);
          
             // �Ÿ��� ����ϴ� ������ ����ǥ�� (5~38m)
            distance_bar = distance*80/34; // (5m = 10) ~ (38m = 90) 
            LCD_DrawRectangle(33,40,distance_bar,10);
            LCD_SetBrushColor(RGB_RED);
            LCD_DrawFillRect(34,41,distance_bar-1,9);
            LCD_SetBrushColor(RGB_YELLOW);
            LCD_DrawFillRect(34+distance_bar,40,90-distance_bar,11);
          }
          else { //! STOP ����
              // STOP ���¸� �Ÿ��� 0
            distance = 0;
            LCD_DisplayChar(3,16,' ');
            LCD_DisplayChar(3,17,'0'); // STOP ���¸� �Ÿ��� '0'
            LCD_SetBrushColor(RGB_YELLOW);
            LCD_DrawFillRect(33,40,91,11);  // ���� ����
          }
          
          // UART1 PC ����
          sprintf(distance_str, "%dm ", distance);
          SerialSendString(distance_str);

          // �����Ÿ��� ���� DR�� ������
          if (ms_flag == 0) DR = 0; // STOP ����: PWM DR = 00%
          else if (distance >= 5 && distance <= 8) DR = 1;    // DR = 10%
          else if (distance >= 9 && distance <= 12) DR = 2;   // DR = 20%
          else if (distance >= 13 && distance <= 16) DR = 3;
          else if (distance >= 17 && distance <= 20) DR = 4;
          else if (distance >= 21 && distance <= 24) DR = 5;
          else if (distance >= 25 && distance <= 28) DR = 6;
          else if (distance >= 29 && distance <= 32) DR = 7;
          else if (distance >= 33 && distance <= 36) DR = 8;
          else if (distance >= 37 && distance <= 38) DR = 9;  // DR = 90%
          
          TIM2->CCR4 = DR*5000; // DR�� ������� ���� CCR������ ����
          
          LCD_DisplayChar(4,8, DR+0x30); // GLCD 3rd line�� DR�� ǥ��
        }
}

// ADC2: �������� ���а��� ���� �Ÿ���
void _ADC2_Init(void)
{   	// ADC2: PA1(pin 41)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h ����)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
						
        RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;	// ENABLE ADC2 CLK (stm32f4xx.h ����)

	ADC->CCR &= ~(0X1F<<0);		// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |=  (1<<16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
        
	ADC2->CR1 &= ~(3<<24);		// RES[1:0]=0b00  : 12bit Resolution
	ADC2->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable
	ADC2->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC

	ADC2->CR2 &= ~(1<<1);		// CONT=0: ADC_Continuous ConvMode Disable
	ADC2->CR2 |=  (2<<28);		// EXTEN[1:0] ADC_ExternalTrigConvEdge_Enable(Falling Edge)
        ADC2->CR2 |=  (2<<24);	        // EXTSEL[3:0]=0b0010: Timer 1 CC3 event
	ADC2->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
	ADC2->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC2->SQR1 &= ~(0xF<<20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
					// 0b0000:1 conversion)
 	//Channel selection, The Conversion Sequence of PIN1(ADC2_CH1) is first, Config sequence Range is possible from 0 to 17
	ADC2->SQR3 |= (1<<0);		// SQ1[4:0]=0b0001 : CH1
        ADC2->SMPR2 |= (0x7<<(3*1));	// ADC2_CH1 Sample Time_480Cycles (3*Channel_1)

	NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt

        ADC2->CR2 |= (1<<0);		// ADON: ADC On
}

// ADC start trigger ��ȣ (CC event)
void TIMER1_OC_Init(void)
{
        // TIM1_CH3 : 300ms �ֱ�� CC event �߻�
        // Clock Enable : TIMER1
	RCC->APB2ENR 	|= (1<<0);	// TIMER1 Enable 
        
	// Assign 'Interrupt Period' and 'Output Pulse Period'
	TIM1->PSC = 1680-1;	// Prescaler 168MHz/1680 = 0.1MHz (10us)
	TIM1->ARR = 30000;	// Auto reload  : 10us * 30K = 300ms(period)

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
	TIM1->CCER |= (1<<8);	// CC3E=1: CC3 channel Output Enable
				// OC3(TIM1_CH3) Active: ��ȣ��� Ȱ��ȭ
	TIM1->CCER &= ~(1<<9);	// CC3P=0: CC3 channel Output Polarity (OCPolarity_High : OC3���� �������� ���) 

	// 'Mode' Selection : Output mode, toggle  
	TIM1->CCMR2 &= ~(3<<0); // CC3S(CC3 channel) = '0b00' : Output 
	TIM1->CCMR2 &= ~(1<<3); // OC3P=0: Output Compare 3 preload disable
	TIM1->CCMR2 |= (3<<4);	// OC3M=0b011: Output Compare 3 Mode : toggle
				// OC3REF toggles when CNT = CCR3

 	TIM1->CCR3 = 1000;	// TIM1 CCR3 TIM1_Pulse
        
        TIM1->BDTR |= (1<<15);
      
	TIM1->CR1 |= (1<<0);	// CEN: Enable the Tim1 Counter				
}

int send_ch;
void USART1_IRQHandler(void)	
{       
	// TX Buffer Empty Interrupt
        if ( (USART1->SR & USART_SR_TXE) )	// USART_SR_TXE=(1<<7)
	{
	  	send_ch++;
		USART1->DR = send_ch;	// ����
		USART1->CR1 &= ~(1<<7);	// TXE interrupt Disable 
	} 
	// RX Buffer Full interrupt
	if ( (USART1->SR & USART_SR_RXNE) ) 	// USART_SR_RXNE=(1<<5) 
	{
		char ch;
		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
		SerialSendChar(ch); 		// Echo
                
                // PC���� MOVE(M) or STOP(S) ����� ���� ������ 3rd line ���� 'M'�̳� 'S'�� ǥ�� 
                if (ch == 'M' || ch == 'S') {
                  if (ch == 'M')  ms_flag = 1;  // MOVE
                  else            ms_flag = 0;  // STOP
                  Fram_Write(1126, ch);     // FRAM�� �� ���� ����
                  LCD_DisplayChar(4,17,ch); // 4th line ���� ���� ǥ��
                }
	} 
	// DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
}

// USART1: PC�� ���
void USART1_Init(void)
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR	|= (1<<0);	// RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER	|= (2<<2*9);	// GPIOB PIN9 Output Alternate function mode					
	GPIOA->OSPEEDR	|= (3<<2*9);	// GPIOB PIN9 Output speed (100MHz Very High speed)
	GPIOA->AFR[1]	|= (7<<4);	// Connect GPIOA pin9 to AF7(USART1)
    
	// USART1 : RX(PA10)
	GPIOA->MODER 	|= (2<<2*10);	// GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR	|= (3<<2*10);	// GPIOA PIN10 Output speed (100MHz Very High speed)
	GPIOA->AFR[1]	|= (7<<8);	// Connect GPIOA pin10 to AF7(USART1)

	RCC->APB2ENR	|= (1<<4);	// RCC_APB2ENR USART1 Enable
    
	USART_BRR_Configuration(19200); // USART Baud rate Configuration
    
	USART1->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
	USART1->CR1	&= ~(1<<10);	// NO USART_Parity
	USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
	USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
	USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
	USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA (Reset State)
    
	USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
	USART1->CR1 	|= (1<<7); 	// 0x0080, TXE interrupt Enable 

	NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37��)
	USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}

// Baud rate ����
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;	//PCLK2_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;

	// Determine the integer part 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
        {                                         // USART1->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));    
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{	// Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));    
	}
	tmpreg = (integerdivider / 100) << 4;
  
	// Determine the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
	{
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else 						// 16 oversampling
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}

	// Write to USART BRR register
	USART1->BRR = (uint16_t)tmpreg;
}

void SerialSendChar(uint8_t Ch) // 1���� ������ �Լ�
{
        while((USART1->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), �۽� ������ ���±��� ���

	USART1->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}

void SerialSendString(char *str) // �������� ������ �Լ�
{
	while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
	{
		SerialSendChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
		str++; 			// ������ ��ġ ����
	}
}

// PWM �޽� �߻�
void TIMER2_PWM_Init(void)
{
// PWM ��: PB11(TIM2_CH4),
// Clock Enable : GPIOB & TIMER2
	RCC->AHB1ENR	|= (1<<1);	// GPIOB Enable
	RCC->APB1ENR 	|= (1<<0);	// TIMER2 Enable 
    						
// PB11�� ��¼����ϰ� Alternate function(TIM2_CH4)���� ��� ���� : PWM ���
	GPIOB->MODER 	|= (2<<2*11);	// PB11 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<2*11);	// PB11 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<11);	// PB11 Output type push-pull (reset state)
	GPIOB->AFR[1]	|= (1<<12); 	// Connect TIM2 pins(PB11) to AF1(TIM1/TIM2)
					// PB11 ==> TIM2_CH4
        
// TIM2 Channel 4 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM2->PSC	= 8400-1;	// Prescaler 84,000,000Hz/8400 = 10kHz (0.1ms)
	TIM2->ARR	= 50000-1;	// Auto reload  (0.1ms * 50k = 5s : PWM Period)
    
	// Define the corresponding pin by 'Output'  
	// CCER(Capture/Compare Enable Register) : Enable "Channel 4" 
	TIM2->CCER	|= (1<<12);	// CC4E=1: OC4(TIM2_CH4) Active(Capture/Compare 4 output enable)
					// �ش����� ���� ��ȣ���
	TIM2->CCER	&= ~(1<<13);	// CC4P=0: CC4 output Polarity High (OC4���� �������� ���)

	// Duty Ratio 
	TIM2->CCR4	= 5000;		// CCR4 value

	// 'Mode' Selection : Output mode, PWM 1
	// CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4
	TIM2->CCMR2 	&= ~(3<<8); 	// CC4S(CC4 channel)='0b00' : Output 
	TIM2->CCMR2 	|= (1<<11); 	// OC4PE=1: Output Compare 4 preload Enable

	TIM2->CCMR2	|= (6<<12);	// OC4M: Output compare 4 mode: PWM 1 mode
	TIM2->CCMR2	|= (1<<15);	// OC4CE: Output compare 4 Clear enable

	// CR1 : Up counting & Counter TIM5 enable
	TIM2->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM2->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM2->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	TIM2->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
	TIM2->CR1	|= (1<<0);	// CEN: Counter TIM5 enable
}

/* EXTI10~15 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void)   // EXTI15 : SW7 ==> STOP Key
{		
	if(EXTI->PR & 0x008000) 		// EXTI15 Interrupt Pending(�߻�) ����?
	{
		EXTI->PR |= 0x8000; 		// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
		ms_flag = 0;			// STOP Key
                Fram_Write(1126, 'S');          // FRAM�� �� ���� ����
                LCD_DisplayChar(4,17,'S');      // 4th line ���� 'S' ǥ��
	}
}

/* EXTI15(SW7) �ʱ� ���� */
void _EXTI_Init(void)
{
  /* EXTI15(PH15) setting */
    	RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= ~0xC0000000;	// GPIOH PIN15 Input mode (reset state)
  
        SYSCFG->EXTICR[3] |= 0x7000; 	// EXTI15�� ���� �ҽ� �Է��� GPIOH�� ����
					// EXTI15 <- PH15 
					// EXTICR4(EXTICR[3])�� �̿� 
	
	EXTI->FTSR |= 0x008000;		// EXTI15: Falling Trigger Enable 
        
    	EXTI->IMR  |= 0x008000;  	// EXTI15 ���ͷ�Ʈ mask (Interrupt Enable) ����
		
	NVIC->ISER[1] |= (1<<(40-32));  // Enable 'Global Interrupt EXTI15'
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

void DisplayTitle(void)
{
	LCD_Clear(RGB_WHITE);
        
        LCD_SetPenColor(RGB_GREEN);
        LCD_DrawRectangle(7,5,141,67);
        LCD_SetBrushColor(RGB_YELLOW);
        LCD_DrawFillRect(8,6,140,66);
        
	LCD_SetFont(&Gulim8);		//��Ʈ 
	
        LCD_SetBackColor(RGB_YELLOW);	//���ڹ���
        LCD_SetTextColor(RGB_BLUE);	// ���ڻ�: BLUE
        LCD_DisplayText(1,2,"AHY 2019132024");
        
	LCD_SetTextColor(RGB_BLACK);	//���ڻ�
       	LCD_DisplayText(2,2,"Tracking Car");
      	LCD_DisplayText(3,2,"D: ");
        LCD_DisplayText(4,2,"S(DR):  %");
        
        LCD_SetTextColor(RGB_BLUE); // �ֱ������� �ٲ�� ���� ���� BLUE
        LCD_DisplayText(4,9,"0");
        
        LCD_SetPenColor(RGB_GREEN); // ���� �� �׵θ� �� : GREEN
        LCD_SetBrushColor(RGB_RED); // ���� �� ä��� �� : RED
}

void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= (1<<9);	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~(1<<9);	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
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