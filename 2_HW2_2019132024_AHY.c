///////////////////////////////////////////////////////////////////////
// HW2: STEP Motor 및 DC Motor 구동 드라이브신호 발생
// 제출자: 2019132024, 안호연
// 개요: 
// 1. OC Mode를 이용한 Step Motor 속도 제어용 구동펄스 발생
//		TIM12 - General purpose Timer / APB1 (84MHz) / 2ch Timer / 16bit
//      TIM12_CH1 (PH6) OC Mode
//              PSC = 8400-1
//
// 2. Original Counter Mode를 이용한 펄스(TIM12_CH1) 주파수 측정
//		TIM3 - General purpose Timer / APB1 (84MHz) / 4ch Timer / 16bit
//      TIM3_CH2 (PB5 / CAN2_RX) => External Clock Mode 1
//              PSC = 1-1
//      Trigger = TI2FP2 신호
//      40핀 박스헤더 PH6(16번) - PB5(15번) 연결
//
//      + TIM6으로 1초마다 UI 발생 -> 1초동안 발생한 펄스 수 (TIM3->CNT값 읽음)
//		  TIM6 - Basic Timer / APB1 (84MHz) / 0ch Timer / 16bit
//              PSC = 8400-1
//
// 3. PWM 1 Mode
//		TIM14 General purpose Timer / APB1 (84MHz) / 1ch Timer / 16bit
//      TIM14_CH1 (PF9 / BUZZER)
//				PSC = 840-1
//
// >> SW0 누를 때마다 Step Motor speed level0~5 변동 (주파수)
// >> SW7 누를 때마다 DC Motor torque level0~6 변동 (주파수, 펄스 폭)
///////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

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
void DisplayInitScreen(void);

// Timer Init function
void TIMER12_OC_Init(void);     // Step motor 구동 pulse
void TIMER3_COUNTER_Init(void); // External clock mode 1
void TIMER6_Init(void);         // Generate 1s Update Interrupt
void TIMER14_PWM_Init(void);    // DC Motor 구동 PWM Pulse

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

int SW0_flag = 0;
uint8_t SW0_up_flag = 1; 
// SW0_up_flag = 1 -> 스위치를 누를때 마다 level up
// SW0_up_flag = 0 -> 스위치를 누를때 마다 level down

int SW7_flag = 0;
uint8_t SW7_up_flag = 1; 
// SW7_up_flag = 1 -> 스위치를 누를때 마다 level up
// SW7_up_flag = 0 -> 스위치를 누를때 마다 level down

uint16_t pulse_cnt;     // TIM3->CNT 펄스 수 저장 변수
uint8_t cnt[5];         // 10의자리, 1의자리 표시 용 변수

int main(void)
{
	_GPIO_Init();           // GPIO init
	LCD_Init();		// LCD 구동 함수
	DelayMS(10);		// LCD 구동 딜레이
	DisplayInitScreen();	// LCD 초기화면구동 함수

	GPIOG->ODR &= ~0x00FF;	// 초기값: LED0~7 Off
	GPIOG->ODR |= 0x01; 	// LED0 ON
    
        // Timer Init function
        TIMER12_OC_Init();
        TIMER3_COUNTER_Init();
        TIMER6_Init();
        TIMER14_PWM_Init();
        
	while(1)
	{
		switch (KEY_Scan())
		{
                        case SW0_PUSH : // SW0 -> TIM12 OC mode pulse의 주파수 변경
                  
			if (SW0_flag == -1) { // Level 0
				TIM12->CR1 &= ~(1<<0); // pulse 발생 안됨 (0Hz)
                                LCD_DisplayText(3, 13, "0");
                                LCD_DisplayText(4, 12, "00");
				SW0_flag++;
				SW0_up_flag = 1;
			}
			else if (SW0_flag == 0) { // Level 1
				TIM12->CR1 |= (1<<0); // pulse 발생 시킴
				TIM12->ARR = 500; // 10Hz
                                LCD_DisplayText(3, 13, "1");
                                LCD_DisplayText(4, 12, "10");
				if (SW0_up_flag) SW0_flag++; // SW0_up_flag가 1이면 level up
				else SW0_flag--; // SW0_up_flag가 0이면 level down
			}
			else if (SW0_flag == 1) { // Level 2
				TIM12->ARR = 250; // 20Hz
                                LCD_DisplayText(3, 13, "2");
                                LCD_DisplayText(4, 12, "20");
				if (SW0_up_flag) SW0_flag++;
				else SW0_flag--;
			}
			else if (SW0_flag == 2) { // Level 3
				TIM12->ARR = 167; // 30Hz
                                LCD_DisplayText(3, 13, "3");
                                LCD_DisplayText(4, 12, "30");
				if (SW0_up_flag) SW0_flag++;
				else SW0_flag--;
			}
			else if (SW0_flag == 3) { // Level 4
				TIM12->ARR = 125; // 40Hz
                                LCD_DisplayText(3, 13, "4");
                                LCD_DisplayText(4, 12, "40");
				if (SW0_up_flag) SW0_flag++;
				else SW0_flag--;
			}
			else if (SW0_flag == 4) { // Level 5
				TIM12->ARR = 100; // 50Hz
                                LCD_DisplayText(3, 13, "5");
                                LCD_DisplayText(4, 12, "50");
				SW0_flag--;
				SW0_up_flag = 0;
			}
                        break;

                        case SW7_PUSH : // SW7 -> TIM14 PWM 펄스의 DR, Frequency 변동
                  
			if (SW7_flag == -1) { // Level 0
				// Freq = 262Hz, DR = 0%
				TIM14->ARR = 381 - 1;
				TIM14->CCR1 = 0; // DR = 0%
				LCD_DisplayText(7, 14, "0");
				SW7_flag++;
				SW7_up_flag = 1;
			}
			else if (SW7_flag == 0) { // Level 1
				// Freq = 262Hz, DR = 20%
				TIM14->ARR = 381 - 1;
				TIM14->CCR1 = 76; // 381 * 0.2 = 76.2
				LCD_DisplayText(7, 14, "1");
				if (SW7_up_flag) SW7_flag++; // SW7_up_flag가 1이면 level up
				else SW7_flag--; // SW7_up_flag가 0이면 level down
			}
			else if (SW7_flag == 1) { // Level 2
				// Freq = 262Hz, DR = 40%
				TIM14->ARR = 381 - 1;
				TIM14->CCR1 = 152; // 381 * 0.4 = 152.4
				LCD_DisplayText(7, 14, "2");
				if (SW7_up_flag) SW7_flag++;
				else SW7_flag--;
			}
			else if (SW7_flag == 2) { // Level 3
				// Freq = 262Hz, DR = 60%
				TIM14->ARR = 381 - 1;
				TIM14->CCR1 = 229; //  381 * 0.6 = 228.6
				LCD_DisplayText(7, 14, "3");
				if (SW7_up_flag) SW7_flag++;
				else SW7_flag--;
			}
			else if (SW7_flag == 3) { // Level 4
				// Freq = 330Hz, DR = 60%
				TIM14->ARR = 303 - 1;  // (330Hz) 3.03ms = 0.01ms * 303
				TIM14->CCR1 = 182; // 303 * 0.6 = 181.8
				LCD_DisplayText(7, 14, "4");
				if (SW7_up_flag) SW7_flag++;
				else SW7_flag--;
			}
			else if (SW7_flag == 4) { // Level 5
				// Freq = 392Hz, DR = 60%
				TIM14->ARR = 255 - 1;  // (392Hz) 2.55ms = 0.01ms * 255
				TIM14->CCR1 = 153; // 255 * 0.6 = 153
				LCD_DisplayText(7, 14, "5");
				if (SW7_up_flag) SW7_flag++;
				else SW7_flag--;
			}
			else if (SW7_flag == 5) { // Level 6
				// Freq = 523Hz, DR = 60%
				TIM14->ARR = 191 - 1;  // (523Hz) 1.91ms = 0.01ms * 191
				TIM14->CCR1 = 115; // 191 * 0.6 = 114.6
				LCD_DisplayText(7, 14, "6");
				SW7_flag--;
				SW7_up_flag = 0;
			}
			break;
		}
      
	}
}

// TIM12_CH1 (PH6)
// step motor 구동용 pulse 출력
void TIMER12_OC_Init(void)
{
// PH6: TIM12_CH1
// PH6을 출력설정하고 Alternate function(TIM12_CH1)으로 사용 선언
	RCC->AHB1ENR	|= (1<<7);	// RCC_AHB1ENR GPIOH Enable : AHB1ENR.7

	GPIOH->MODER    |= (2<<12);	// GPIOH PIN6 Output Alternate function mode 					
	GPIOH->OSPEEDR 	|= (3<<12);	// GPIOH PIN6 Output speed (100MHz High speed)
	GPIOH->OTYPER	&= ~(1<<6);	// GPIOH PIN6 Output type push-pull (reset state)
	GPIOH->PUPDR    |= (1<<12); 	// GPIOH PIN6 Pull-up
  					// PH6 ==> TIM12_CH1
	GPIOH->AFR[0]	|= (9<<4*6);  // (AFR[0].(27~24)=0b1001): Connect TIM12 pins(PH6) to AF9
 
// Time base 설정
	RCC->APB1ENR |= (1<<6);	// RCC_APB1ENR TIMER12 Enable

	// Setting CR1 : 0x0000 
	TIM12->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM12->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                            //  Counter Overflow/Underflow, 
                            //  Setting the UG bit Set,
                            //  Update Generation through the slave mode controller 
                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM12->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
                            //	Counter Overflow/Underflow, 
                            // Setting the UG bit Set,
                            //	Update Generation through the slave mode controller 
                            // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM12->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM12->CR1 &= ~(1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enalbe 
	TIM12->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM12->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)

	// Setting the Period
	TIM12->PSC = 8400-1;	// Prescaler=84, 84MHz/8.4k = 10kHz (0.1ms)
	TIM12->ARR = 500-1;	// Auto reload  : 0.1ms * 500 = 50ms (20Hz) : 50ms
                                                          // OC mode의 파형이므로 주기는 두배인 100ms
       
	// Update(Clear) the Counter
	TIM12->EGR |= (1<<0);    // UG: Update generation    

// Output Compare 설정
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM12->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output
	TIM12->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM12->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable(CCR1에 언제든지 새로운 값을 loading 가능) 
	TIM12->CCMR1 |= (3<<4);	// OC1M=0b011 (Output Compare 1 Mode : toggle)
				// OC1REF toggles when CNT = CCR1
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM12->CCER |= (1<<0);	// CC1E=1: CC1 channel Output Enable
				// OC1(TIM12_CH1) Active: 해당핀(40핀 박스헤더 16번 핀)을 통해 신호출력
	TIM12->CCER &= ~(1<<1);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1으로 반전없이 출력)  

	TIM12->CR1 &= ~(1<<0);	// Disable TIM12 (Level 0일 때는 pulse 발생 안되므로)				
}

// TIM3_CH2 (PB5 / CAN2_RX)
// External Clock mode 1
void TIMER3_COUNTER_Init(void)
{
// Encoder 입력(Counting) 핀: PB5 (TIM3_CH2)
// Clock Enable : GPIOB & TIMER3
	RCC->AHB1ENR	|= (1<<1);	// GPIOB Enable
	RCC->APB1ENR 	|= (1<<1);	// TIMER3 Enable 

// PB5: TIM3_CH2
// PB5를 Alternate function(TIM3_CH2)으로 사용 선언
	GPIOB->MODER 	|= (2<<10);	// (MODER.(11,10)), GPIOB PIN5 intput Alternate function mode 					
	GPIOB->OSPEEDR 	|= (2<<10);	// (OSPEEDER.(11,10)), GPIOB PIN5 Output speed (50MHz High speed)
	GPIOB->PUPDR	&= ~(3<<10); 	// GPIOB PIN5 NO Pull-up
	GPIOB->AFR[0]	|= (2<<20);	// (AFR[0].(23~20)): Connect TIM3 pins(PB5) to AF2(TIM3..5)
  
// Time base Mode
	// Setting CR1 : 0x0000 
	TIM3->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM3->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM3->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)
	TIM3->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM3->CR1 |=  (1<<7);	// ARPE=1(ARR Preload Enable)
	TIM3->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM3->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
 
	// PSC, ARR
	TIM3->PSC = 1-1;	// Prescaler=1
	TIM3->ARR = 100;	// Auto reload  :  count값 범위: 0~100
        
	// Update(Clear) the Counter
	TIM3->EGR |= (1<<0);    // UG=1, REG's Update (CNT clear) 

// External Clock Mode 1
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM3->CCMR1 |= (1<<8); 	// CC2S(CC2 channel) = '0b01' : Input 
	TIM3->CCMR1 &= ~(15<<12); // IC2F='0b0000: No Input Filter 
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 2" 
	TIM3->CCER &= ~(1<<4);	// CC2E=0: Capture Disable
	TIM3->CCER &= ~(1<<5);	// CC2P=0: TI2FP2 NonInverting / Rising Edge   
	TIM3->CCER &= ~(1<<7);	// CC2NP=0:   
        
	// SMCR(Slave Mode Control Reg.) : External Clock Enable
	TIM3->SMCR |= (6<<4);	// TS(Trigger Selection)=0b100 : TI2FP2 (TI2 Edge Detector 출력신호)
	TIM3->SMCR |= (7<<0);	// SMS(Slave Mode Selection)=0b111 : External Clock Mode 1

	TIM3->CR1 |= (1<<0);	// CEN: Enable the Tim4 Counter  	
}

// TIMER6 - Generate 1s interrupt 
void TIMER6_Init(void)
{
	RCC->APB1ENR |= (1<<4);	// RCC_APB1ENR TIMER6 Enable
    
	// Setting CR1 : 0x0000 
	TIM6->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM6->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
				//	Counter Overflow/Underflow, 
				// 	Setting the UG bit Set,
				//	Update Generation through the slave mode controller 
	TIM6->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
				//	Counter Overflow/Underflow, 
				// 	Setting the UG bit Set,
				//	Update Generation through the slave mode controller 
	TIM6->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM6->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM6->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM6->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
				// Center-aligned mode: The counter counts Up and DOWN alternatively
   
	// Deciding the Period
	TIM6->PSC = 8400-1;	// Prescaler 84,000,000Hz/8400 = 10,000 Hz (0.1ms)  (1~65536)
	TIM6->ARR = 10000-1;	// Auto reload  0.1ms * 10000 = 1s

	// Clear the Counter
	TIM6->EGR |= (1<<0);	// UG(Update generation)=1 (Re-initialize the counter & generates an update    

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[1] |= (1<<(54-32)); // Enable Timer3 global Interrupt
 	TIM6->DIER |= (1<<0);	// Enable the Tim3 Update interrupt

	TIM6->CR1 |= (1<<0);	// Enable the Tim3 Counter (clock enable)   
}

// 1s Interrupt Handler
// 1초마다 펄스 수를 측정하여 LCD 6번째줄 ('In-Freq.')에 표시
void TIM6_DAC_IRQHandler(void)
{
      TIM6->SR &= ~(1<<0);	// Interrupt flag Clear
      
      pulse_cnt = TIM3->CNT;	// TIM3 CNT값(펄스 수) 읽음
      TIM3->CNT = 0;
      
      cnt[1]= pulse_cnt%100/10;	// 10 자리
      cnt[0]= pulse_cnt%10;	// 1 자리
      LCD_DisplayChar(5,11,cnt[1]+0x30); // LCD 6번째줄 ('In-Freq.')에 표시
      LCD_DisplayChar(5,12,cnt[0]+0x30);
}

// TIM14_CH1 (PF9 / BUZZER)
// PWM 1 Mode
void TIMER14_PWM_Init(void)
{  
// DC Motor 구동(PWM)핀:PF9 (TIM14_CH1)
// Clock Enable : GPIOF & TIMER14
	RCC->AHB1ENR	|= (1<<5);	// GPIOF Enable
	RCC->APB1ENR 	|= (1<<8);	// TIMER14 Enable 
    						
// PF9를 출력으로설정하고 Alternate function(TIM14_CH1)으로 사용 선언 : PWM 출력
	GPIOF->MODER 	|= (2<<2*9);	// PF9 Output Alternate function mode					
	GPIOF->OSPEEDR 	|= (3<<2*9);	// PF9 Output speed (100MHz High speed)
	GPIOF->OTYPER	&= ~(1<<2*9);	// PF9 Output type push-pull (reset state)
	GPIOF->AFR[1]	|= (9<<4); 	// 0x00000090	(AFR[1].(7~4)=0b1001): Connect TIM14 pins(PF9) to AF9(CAN1/CAN2, TIM12..14)
					// PF9 ==> TIM14_CH1
          
// TIM14 Channel 1 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM14->PSC	= 840-1;	// Prescaler 84,000,000Hz/840 = 100,000 Hz (100kHz) 
	TIM14->ARR	= 381-1;	// Auto reload  0.01ms * 381 = 3.81ms  (262Hz) 
    
	// Define the corresponding pin by 'Output'  
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM14->CCER	|= (1<<0);	// CC1E=1: OC1(TIM14_CH1) Active(Capture/Compare 1 output enable)
					// 해당핀(박스헤더 16번 핀)을 통해 신호출력
	TIM14->CCER	&= ~(1<<1);	// CC1P=0: CC1 output Polarity High (OC1으로 반전없이 출력)

	// Duty Ratio 
	TIM14->CCR1	= 0;		// Level 0의 DR = 0%

	// 'Mode' Selection : Output mode, PWM 1
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM14->CCMR1 	&= ~(3<<0); 	// CC1S(CC1 channel)='0b00' : Output 
	TIM14->CCMR1 	|= (1<<3); 	// OC1PE=1: Output Compare 1 preload Enable

	TIM14->CCMR1	|= (6<<4);	// OC1M: Output compare 1 mode: PWM 1 mode
	TIM14->CCMR1	|= (1<<7);	// OC1CE: Output compare 1 Clear enable

	// CR1 : Up counting & Counter TIM5 enable
	TIM14->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM14->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM14->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	TIM14->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
	TIM14->CR1	|= (1<<0);	// CEN: Counter TIM14 enable
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
}	

void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
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

void DisplayInitScreen(void)
{
	LCD_Clear(RGB_YELLOW);		// 화면 클리어
	LCD_SetFont(&Gulim8);		// 폰트 : 굴림 8
        
	LCD_SetBackColor(RGB_BLACK);	// 글자배경색 : BLACK
	LCD_SetTextColor(RGB_WHITE);	// 글자색 : WHITE
        LCD_DisplayText(0,0,"Motor Control    ");
        LCD_DisplayText(1,0," :AHY 2019132024 ");

        LCD_SetBackColor(RGB_YELLOW);	// 글자배경색 : YELLOW
	LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
        LCD_DisplayText(2,0,"[Step Motor]");
        LCD_DisplayText(6,0,"[DC Motor]");
        
        LCD_SetTextColor(RGB_BLACK);	// 글자색 : BLACK
        LCD_DisplayText(3,0,">Speed Level:");
        LCD_DisplayText(4,0,">Out-Freq.: ");
        LCD_DisplayText(5,0,">In-Freq.: ");
        LCD_DisplayText(7,0,"*Torque Level: ");
        LCD_DisplayText(4,14,"Hz");
        LCD_DisplayText(5,13,"Hz");
        
        LCD_SetTextColor(RGB_RED);	// 글자색 : RED
        LCD_DisplayText(3,13,"0");
        LCD_DisplayText(4,12,"00");
        LCD_DisplayText(5,11,"00");
        LCD_DisplayText(7,14,"0");
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

