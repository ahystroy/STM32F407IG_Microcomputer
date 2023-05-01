//////////////////////////////////////////////////////////////////////////
// PR2: 반도체 공정제어기 (Semiconductor Processing Controller)
// 제출자: 2019132024 안호연
//
// 주요 내용
// - 가변 저항: 각 resource별로 입력값 조절 (ADC2_IN1, PA1)
//           ADC 작동은 SWSTART를 사용하여 인터럽트로 처리
//
// - Switch4~7 (PH12 ~ PH15) 
//    # 스위치 기능
//    > SW4: 세부입력모드에서 입력값 결정 및 저장
//    > SW5: 모드 변환 (입력 모드 'I' <-> 실행 모드 'R')
//    > SW6: 각 모드에서 세부 프로세스 시작 스위치
//    > SW7: 입력모드에서 세부입력모드간의 변경 (온도->압력->O2->Plasma->온도->...)
//
// - LED0~7: 현재 진행중인 모드 및 프로세스 표시 (PG0 ~ PG7) 
//
// - TIMER2: Plasma PWM (TIM2_CH3, PB10) 
// - TIMER3: 0.1s timer (UI Interrupt Generate) -> 각 장치들의 작동 시간 count
//
// - FRAM: 각 입력값 저장 (SPI2)
//
// - GLCD: 현재 제어상황 출력 (FSMC)
//
//////////////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "FRAM.h"
#include "GLCD.h"

#define RESET   0
#define SET     1

void _GPIO_Init(void);
void _EXTI_Init(void);
void LCD_Display_Init(void);

uint8_t getValueDivide_7(uint8_t x);    // ADC값 전체 영역을 7등분하여 입력값 종류에 따라 값을 얻어오는 함수
uint8_t getValueDivide_10(void);        // ADC값 전체 영역을 10등분하여 DR값을 얻어오는 함수

void _ADC_Init(void);
void TIMER3_Init(void);      // 0.1s count
void TIMER2_PWM_Init(void);  // Plasma PWM

void LED(uint8_t x, uint8_t state);  // LED'x'를 'state'로 설정 (SET=1 or RESET=0)

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

uint8_t fram_save;            // SW4: 현재값 FRAM에 저장
uint8_t mode;                 // SW5: 모드 변경 (입력 <-> 실행)
uint8_t input_key;            // SW6: 세부 입력 모드
uint8_t start_key;            // SW6: 실행 시작 명령
uint8_t input_x;              // SW7: 입력할 데이터의 종류

uint8_t TE, PR, O2, DR;             // resource value
uint8_t TE_s, PR_s, O2_s, DR_s;     // set time

// 실행모드에서 flag 변수 (인터럽트 루틴에서 flag 변수를 변하게 하여 메인의 while 루프를 탈출)
uint8_t Heater_on, AirPump_on, O2_on, Plasma_on;
uint8_t count; // TIMER3 0.1s count 변수

uint16_t ADC_Value;
uint16_t ADC_Value_7  = 4095/ 7;
double   ADC_Value_10 = 4095/10;

int main(void)
{
    // Init Routine
    LCD_Init();	        // LCD Init
    DelayMS(10);	// LCD 구동 딜레이
    LCD_Display_Init();	// LCD 초기화면구동 함수
    _GPIO_Init();
    _EXTI_Init();
    Fram_Init();            // FRAM H/W 초기화
    Fram_Status_Config();   // FRAM S/W 초기화
    _ADC_Init();
    TIMER3_Init();
    TIMER2_PWM_Init();
    
    // 초기에는 입력모드 실행
    LED(0, SET);
    LED(1, RESET);
    
    // 초기에 FRAM 값을 읽어 해당 값을 화면에 표시
    TE = (uint8_t)Fram_Read(1203);
    LCD_DisplayChar(5,7, TE/10+0x30);
    LCD_DisplayChar(5,9, TE%10+0x30);
    PR = (uint8_t)Fram_Read(1204);
    LCD_DisplayChar(6,7, PR/10+0x30);
    LCD_DisplayChar(6,9, PR%10+0x30);
    O2 = (uint8_t)Fram_Read(1205);
    LCD_DisplayChar(7,7, O2/10+0x30);
    LCD_DisplayChar(7,9, O2%10+0x30);
    DR = (uint8_t)Fram_Read(1206);
    LCD_DisplayChar(8,7, DR+0x30);
    
    while(1)
    {
        if (start_key == 1) { // S8: 실행시작
            
        //! (1) Wafer IN
            LCD_DisplayChar(4,16,'W'); // 'E' -> 'W'
            LED(3, SET);
            DelayMS(500); // 강제 delay 0.5s
            
            
        //! (2) Heater On
            Heater_on = 1;
            LED(4, SET);
            LCD_DisplayChar(5,3,'*'); // 'TE'앞에 '*'표시
            TE_s = (uint8_t)Fram_Read(1203); // ON 시간 read
            
            TIM3->CR1 |= (1<<0); // 0.1s timer on
            // ON 시간이 될 때까지 0.1초 주기의 UI 인터럽트를 이용하여 화면에 0.1초씩 증가하도록 표시
            while (Heater_on) {
                LCD_DisplayChar(5,12, count/10 + 0x30);
                LCD_DisplayChar(5,14, count%10 + 0x30);
            }
            // 인터럽트 핸들러의 조작에 의해 무한루프 빠져나오면서 LED4 OFF, '*' 삭제
            TIM3->CR1 &= ~(1<<0); count = 0; // timer off
            LED(4, RESET);
            LCD_DisplayChar(5,3,' ');
            
            
        //! (3) Air Pump ON
            AirPump_on = 1;
            LED(5, SET);
            LCD_DisplayChar(6,3,'*'); // 'PR'앞에 '*'표시
            PR_s = (uint8_t)Fram_Read(1204); // ON 시간 read
            
            TIM3->CR1 |= (1<<0); // 0.1s timer on
            // ON 시간이 될 때까지 0.1초 주기의 UI 인터럽트를 이용하여 화면에 0.1초씩 증가하도록 표시
            while (AirPump_on) {
                LCD_DisplayChar(6,12, count/10 + 0x30);
                LCD_DisplayChar(6,14, count%10 + 0x30);
            }
            // 인터럽트 핸들러의 조작에 의해 무한루프 빠져나오면서 LED5 OFF, '*' 삭제
            TIM3->CR1 &= ~(1<<0); count = 0; // timer off
            LED(5, RESET);
            LCD_DisplayChar(6,3,' ');
            
            
        //! (4) O2 ON
        //! (5) Plasma ON
            O2_on = 1; Plasma_on = 1; // O2 On 시작과 동시에 Plasma도 동시에 ON
            LED(6, SET); LED(7, SET);
            LCD_DisplayChar(7,3,'*'); // 'O2'앞에 '*'표시
            LCD_DisplayChar(8,3,'*'); // 'DR'앞에 '*'표시
            
            O2_s = (uint8_t)Fram_Read(1205); // O2 ON 시간 read
            DR_s = (uint8_t)Fram_Read(1206); // DR ON 시간 read
            TIM2->CCR3 = DR_s*2000;
            
            TIM3->CR1 |= (1<<0); // 0.1s timer on
            TIM2->CR1 |= (1<<0); // PWM 발생 (PWM 유지 시간은 현재의 O2값)
            // ON 시간이 될 때까지 0.1초 주기의 UI 인터럽트를 이용하여 화면에 0.1초씩 증가하도록 표시
            while (O2_on) {
                LCD_DisplayChar(7,12, count/10 + 0x30);
                LCD_DisplayChar(7,14, count%10 + 0x30);
            }
            // 인터럽트 핸들러의 조작에 의해 무한루프 빠져나오면서
            TIM3->CR1 &= ~(1<<0); count = 0; // timer off
            TIM2->CCR3 = 0; // DR = 0으로 하여 PWM 신호 발생 차단
            TIM2->CR1 &= ~(1<<0); // PWM 발생 off
            // LED6,7 OFF, '*' 삭제
            LED(6, RESET); LED(7, RESET);
            LCD_DisplayChar(7,3,' ');
            LCD_DisplayChar(8,3,' ');
            

        //! (6) Wafer Out
            DelayMS(500); // 강제 delay 0.5s
            LCD_DisplayChar(4,16,'E'); // 'W' -> 'E'
            LED(3,RESET);
          
            start_key = 2;
        }
    }
}

void ADC_IRQHandler(void)
{
	ADC1->SR &= ~(1<<1);		// EOC flag clear

	ADC_Value = ADC1->DR;		// Reading ADC result
        
        if (input_x == 1) { // 온도값 입력
          TE = getValueDivide_7(input_x);       // 가변저항 값에 따라 온도값 환산하여 얻음
          LCD_DisplayChar(5,7, TE/10+0x30);     // LCD 표시
          LCD_DisplayChar(5,9, TE%10+0x30);
        }
        else if (input_x == 2) { // 압력값 입력
          PR = getValueDivide_7(input_x);       // 가변저항 값에 따라 압력값 환산하여 얻음
          PR = (40-PR);  // Air Pump 가동시간은 압력에 '반비례'
          LCD_DisplayChar(6,7, PR/10+0x30);     // LCD 표시
          LCD_DisplayChar(6,9, PR%10+0x30);
        }
        else if (input_x == 3) { // O2값 입력
          O2 = getValueDivide_7(input_x);       // 가변저항 값에 따라 O2값 환산하여 얻음
          LCD_DisplayChar(7,7, O2/10+0x30);     // LCD 표시
          LCD_DisplayChar(7,9, O2%10+0x30);
        }
        else if (input_x == 4) { // Plasma 세기 입력
          DR = getValueDivide_10();
          LCD_DisplayChar(8,7, DR+0x30);
        }
        
 	//Starts conversion of regular channels
	ADC1->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30) 
}

//! ADC(12비트, 0~4095)값 전체영역을 7등분하여 입력값의 종류에 맞게끔 각 구간별로 반환
uint8_t getValueDivide_7(uint8_t x)
{
  if      (ADC_Value <= ADC_Value_7)                                return 0  + (3-x)*5;  // 입력하는 값 종류(x)에 따라 반환값 결정
  else if (ADC_Value > ADC_Value_7*1 && ADC_Value <= ADC_Value_7*2) return 5  + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*2 && ADC_Value <= ADC_Value_7*3) return 10 + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*3 && ADC_Value <= ADC_Value_7*4) return 15 + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*4 && ADC_Value <= ADC_Value_7*5) return 20 + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*5 && ADC_Value <= ADC_Value_7*6) return 25 + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*6 && ADC_Value <= ADC_Value_7*7) return 30 + (3-x)*5;
}

//! ADC(12비트, 0~4095)값 전체영역을 10등분하여 최소구간은 DR:0, 최대구간은 DR:9가 되도록 반환
uint8_t getValueDivide_10(void)
{
  if      (ADC_Value <= ADC_Value_7)                                   return 0;
  else if (ADC_Value > ADC_Value_10*1 && ADC_Value <= ADC_Value_10*2)  return 1;
  else if (ADC_Value > ADC_Value_10*2 && ADC_Value <= ADC_Value_10*3)  return 2;
  else if (ADC_Value > ADC_Value_10*3 && ADC_Value <= ADC_Value_10*4)  return 3;
  else if (ADC_Value > ADC_Value_10*4 && ADC_Value <= ADC_Value_10*5)  return 4;
  else if (ADC_Value > ADC_Value_10*5 && ADC_Value <= ADC_Value_10*6)  return 5;
  else if (ADC_Value > ADC_Value_10*6 && ADC_Value <= ADC_Value_10*7)  return 6;
  else if (ADC_Value > ADC_Value_10*7 && ADC_Value <= ADC_Value_10*8)  return 7;
  else if (ADC_Value > ADC_Value_10*8 && ADC_Value <= ADC_Value_10*9)  return 8;
  else if (ADC_Value > ADC_Value_10*9)                                 return 9;
}

void _ADC_Init(void)
{	// ADC1: PA1(pin 41) 가변저항
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h 참조)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
        
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// (1<<8) ENABLE ADC1 CLK (stm32f4xx.h 참조)

	ADC->CCR &= ~(0X1F<<0);		// MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |=  (1<<16); 		// 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
        
	ADC1->CR1 &= ~(3<<24);		// RES[1:0]=0b00 : 12bit Resolution
	ADC1->CR1 &= ~(1<<8);		// SCAN=0 : ADC_ScanCovMode Disable
	ADC1->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC

	ADC1->CR2 &= ~(1<<1);		// CONT=1: ADC_Continuous ConvMode Disable
	ADC1->CR2 &= ~(3<<28);		// EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge_None
	ADC1->CR2 &= ~(1<<11);		// ALIGN=0: ADC_DataAlign_Right
	ADC1->CR2 &= ~(1<<10);		// EOCS=0: The EOC bit is set at the end of each sequence of regular conversions

	ADC1->SQR1 &= ~(0xF<<20);	// L[3:0]=0b0000: ADC Regular channel sequece length 
					// 0b0000:1 conversion)
    
        ADC1->SMPR2 |= (0x7<<(3*1));	// ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
 	//Channel selection, The Conversion Sequence of PIN1(ADC1_CH1) is first, Config sequence Range is possible from 0 to 17
	ADC1->SQR3 |= (1<<0);		// SQ1[4:0]=0b0001 : CH1

	NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt

	ADC1->CR2 &= ~(1<<0);		// ADON: ADC Off
}

void TIM3_IRQHandler(void)  	// 0.1s Interrupt
{
	TIM3->SR &= ~(1<<0);	// Interrupt flag Clear
        
        // if문에서 현재 ON 되어있는 장치로 들어감 (장치는 한 순간에 하나만 ON 되도록 설정하였음)
        if (Heater_on) {
            if (count == TE_s) { // 설정한 ON 시간이 되면
                Heater_on = 0; // 인터럽트 루틴에서 특정 플래그 변수를 변하게 함 --> 무한루프를 빠져나오게 됨.
            }
            else count++; // ON 시간이 아직 안됬으면 count 계속 증가. (0.1s 주기)
        }
        else if (AirPump_on) {
            if (count == PR_s) { // 설정한 ON 시간이 되면
                AirPump_on = 0; // 인터럽트 루틴에서 특정 플래그 변수를 변하게 함 --> 무한루프를 빠져나오게 됨.
            }
            else count++; // ON 시간이 아직 안됬으면 count 계속 증가. (0.1s 주기)
        }
        else if (O2_on) {
            if (count == O2_s) { // 설정한 ON 시간이 되면
                O2_on = 0; // 인터럽트 루틴에서 특정 플래그 변수를 변하게 함 --> 무한루프를 빠져나오게 됨.
            }
            else count++; // ON 시간이 아직 안됬으면 count 계속 증가. (0.1s 주기)
        }
        else if (Plasma_on) {
            if (count == DR_s) { // 설정한 ON 시간이 되면
                Plasma_on = 0; // 인터럽트 루틴에서 특정 플래그 변수를 변하게 함 --> 무한루프를 빠져나오게 됨.
            }
            else count++; // ON 시간이 아직 안됬으면 count 계속 증가. (0.1s 주기)
        }
}

/* 0.1s UI 발생 */
void TIMER3_Init(void)
{
	RCC->APB1ENR |= 0x02;	// RCC_APB1ENR TIMER3 Enable

	// CR1 setting 생략 : 0x0000 (Reset State)

        // Deciding the Period
	TIM3->PSC = 8400-1;	// Prescaler 84MHz / 8400 = 10kHz (0.1ms)
	TIM3->ARR = 1000-1;	// Auto reload --> t = 0.1ms * 1000 = 0.1s

   	// Clear the Counter
	TIM3->EGR |= (1<<0);	// UG(Update generation)=1 
                        // Re-initialize the counter(CNT=0) & generates an update of registers

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1<<29); // Enable Timer3 global Interrupt
 	TIM3->DIER |= (1<<0);	// Enable the Tim3 Update interrupt

	TIM3->CR1 &= ~(1<<0);	// Enable the Tim3 Counter (clock enable)
}

/* Plasma PWM 신호 발생 */ 
void TIMER2_PWM_Init(void)
{   
// TIM2_CH3 : PB10 (79번 핀)
// Clock Enable : GPIOB & TIMER2
	RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
	RCC->APB1ENR 	|= (1<<0);	// TIMER2 CLOCK Enable 
    						
// PB10을 출력설정하고 Alternate function(TIM2_CH3)으로 사용 선언 : PWM 출력
	GPIOB->MODER 	|= (2<<20);	// PB10 Output Alternate function mode					
	GPIOB->OSPEEDR 	|= (3<<20);	// PB10 Output speed (100MHz High speed)
	GPIOB->OTYPER	&= ~(1<<10);	// PB10 Output type push-pull (reset state)
	GPIOB->PUPDR	|= (1<<20);	// PB10 Pull-up
 	GPIOB->AFR[1]	|= (1<<8);	// (AFR[1].(3~0)=0b0001): Connect TIM2 pins(PB10) to AF1(TIM1/TIM2)
    
// TIM2 Channel 3 : PWM 1 mode
	// Assign 'PWM Pulse Period'
	TIM2->PSC	= 8400-1;	// Prescaler 84MHz/8.4k = 10kHz (0.1ms)
	TIM2->ARR	= 20000-1; 	// Auto reload  (0.1ms * 20k = 2s : PWM Period)

// Setting CR1 : 0x0000 (Up counting)
	TIM2->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM2->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM2->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)g events
	TIM2->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM2->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enable 
	TIM2->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM2->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 : Edge-aligned mode(reset state)
				
// Define the corresponding pin by 'Output'  
// CCER(Capture/Compare Enable Register) : Enable "Channel 3" 
	TIM2->CCER |= (1<<8);	// CC3E=1: OC3(TIM2_CH3) Active(Capture/Compare 3 output enable)
					// 해당핀(79번)을 통해 신호출력
	TIM2->CCER &= ~(1<<9);	// CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3으로 반전없이 출력)

// Duty Ratio 
	TIM2->CCR3 = 0;	// CCR3 value

// 'Mode' Selection : Output mode, PWM 1
// CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4
	TIM2->CCMR2 &= ~(3<<0); // CC3S(CC3 channel)= '0b00' : Output 
	TIM2->CCMR2 |= (1<<3); 	// OC3PE=1: Output Compare 3 preload Enable
	TIM2->CCMR2 |= (6<<4);	// OC3M=0b110: Output compare 3 mode: PWM 1 mode
	TIM2->CCMR2 |= (1<<7);	// OC3CE=1: Output compare 3 Clear enable
	
// Plasma On 시에 PWM 발생하기 위해 TIM2 disable.
	TIM2->CR1 &= ~(1<<0);	// CEN: Counter TIM2 disable
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

/* SW4 ~ SW7's Interrupt Handler Routine */
void EXTI15_10_IRQHandler(void)
{
  // EXTI12 ~ EXTI15 Interrupt Pending(발생) 여부 체크
  
     //! SW4: 세부입력모드에서 입력값 결정 및 FRAM에 저장
	if (EXTI->PR & 0x1000)
	{
            EXTI->PR |= 0x1000; 		// Pending bit Clear
            
            ADC1->CR2 |= ADC_CR2_SWSTART;
            if (input_key) {      // FRAM에 저장
              if      (input_x == 1) Fram_Write(1203, TE);
              else if (input_x == 2) Fram_Write(1204, PR);
              else if (input_x == 3) Fram_Write(1205, O2);
              else if (input_x == 4) Fram_Write(1206, DR);
            }
	}
        
     //! SW5: 모드 변환 (입력 <-> 실행)
        else if (EXTI->PR & 0x2000)
	{
            EXTI->PR |= 0x2000; 		// Pending bit Clear

            if (mode) {
                if (!start_key) { //! 실행중이 아닐 때만 입력모드로 변경
                    LED(0, SET);
                    LED(1, RESET); 
                    LCD_DisplayChar(4,8,'I');
                    mode = 0; // 입력모드
                }
            }
            else {
                if (!input_key) { //! 세부 입력모드를 빠져나왔을때만 모드 전환
                    LED(0, RESET);
                    LED(1, SET);
                    LCD_DisplayChar(4,8,'R');
                    mode = 1; // 실행모드
                }
            }
	}
        
     //! SW6: 각 모드에서 세부 프로세스 시작 (세부명령 입력모드 시작)
        else if (EXTI->PR & 0x4000)
	{
            EXTI->PR |= 0x4000; 		// Pending bit Clear
            
            if (!mode) { //! 입력모드에서 "SW6 = 입력 시작 명령"
                if (input_key) {
                    ADC1->CR2 &= ~(1<<0); // ADC Off
                    LED(2, RESET); 
                    // LED4~7 Off (문제에 제시되어있지 않지만 흐름상 끄고 실행모드로 진입하는게 맞는 것 같아 이렇게 설정하였습니다.)
                    GPIOG->ODR &= ~(0xF<<4);
                    LCD_DisplayChar(5,3,' '); // LCD '*'도 삭제
                    LCD_DisplayChar(6,3,' ');
                    LCD_DisplayChar(7,3,' ');
                    LCD_DisplayChar(8,3,' ');
                    input_key = 0; // 입력 안함
                }
                else {
                    ADC1->CR2 |= (1<<0); // ADC On
                    ADC1->CR2 |= ADC_CR2_SWSTART;
                    LED(2, SET);
                    if (input_x != 0) {
                      LED(input_x+3, SET); // 현재 입력할 값의 LED On
                      LCD_DisplayChar(4+input_x,3,'*'); // 현재 입력할 값 앞에 '*' 표시
                    }
                    input_key = 1; // 입력 시작
                }
            }
            // mode = 0 : 입력모드
            // mode = 1 : 실행모드
            else { //! 실행모드에서 "SW6 = 실행 시작 명령"
                if (start_key == 0) {
                    LED(2, SET);
                    start_key = 1; // 실행 시작
                }
                // start_key = 0 : 실행을 시작하지 않은 상태
                // start_key = 1 : 실행 중인 상태 --> 실행 중에는 중단 불가하도록 설정
                // start_key = 2 : 실행을 마친 상태
                else if (start_key == 2) {
                    LED(2, RESET); // 실행을 끝마치고 중단을 확실히 함.
                    start_key = 0;
                }
            }
	}
        
     //! SW7: 입력모드에서 세부입력모드간의 변경 (온도 -> 압력 -> O2 -> Plasma -> 온도 ...)
        else if (EXTI->PR & 0x8000)
	{
            EXTI->PR |= 0x8000; 		// Pending bit Clear
            if (input_key) {
                ADC1->CR2 |= ADC_CR2_SWSTART;
                
                input_x++; // 다음 입력
                
                // LED 모두 Off 한 후, 해당 LED On
                GPIOG->ODR &= ~(0xF<<4);
                if (input_x == 5) input_x = 1;
                LED(input_x+3, SET);
                
                // 해당 입력값에 * 표시
                LCD_DisplayChar(5,3,' ');
                LCD_DisplayChar(6,3,' ');
                LCD_DisplayChar(7,3,' ');
                LCD_DisplayChar(8,3,' ');
                LCD_DisplayChar(4+input_x,3,'*');
            }
	}
}

/* EXTI12 ~ EXTI15 setting */
void _EXTI_Init(void)
{
        /* Clock Enable */
    	RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= ~0xFF000000;	// GPIOH PIN12~PIN15 Input mode (reset state)
        
        /* EXTI12 ~ EXTI15 setting */
        SYSCFG->EXTICR[3] |= 0x7777;    // EXTI12~15에 대한 소스 입력은 GPIOH로 설정
        EXTI->FTSR |= (0xF<<12);        // EXTI12~15: Falling Trigger Enable
        EXTI->IMR  |= (0xF<<12);        // EXTI12~15: Interrupt Enable
        NVIC->ISER[1] |= (1<<(40-32));  // Enable 'Global Interrupt EXTI15_10'
}


/*!  LED'x'를 'state'로 설정  !*/
void LED(uint8_t x, uint8_t state)
{
  if (state) GPIOG->ODR |= (1<<x);
  else GPIOG->ODR &= ~(1<<x);
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

void LCD_Display_Init(void)
{
	LCD_Clear(RGB_YELLOW);          // 배경색   : YELLOW 
        
        LCD_SetPenColor(RGB_BLUE);      // 테두리 색 : BLUE
        LCD_DrawRectangle(14,7,134,115);
        
	LCD_SetFont(&Gulim8);		// 폰트 
	LCD_SetBackColor(RGB_YELLOW);	// 글자배경색 : YELLOW
        
	LCD_SetTextColor(RGB_BLUE);	// 제목 글자색: BLUE
       	LCD_DisplayText(1,3,"AHY 2019132024");
      	LCD_DisplayText(2,3,"SPC monitor");
        
        LCD_SetTextColor(RGB_BLACK);	// 고정 글자색: BLACK
        LCD_DisplayText(4,3,"Mode:  Wafer:");
        LCD_DisplayText(5,4,"TE:   s    s");
        LCD_DisplayText(6,4,"PR:   s    s");
        LCD_DisplayText(7,4,"O2:   s    s");
        LCD_DisplayText(8,4,"DR:");
        
        LCD_SetTextColor(RGB_RED);      // 변동 글자색: RED
        LCD_DisplayChar(4,8,'I');       // 초기  Mode = 'I'
        LCD_DisplayChar(4,16,'E');      // 초기 Wafer = 'E'
        LCD_DisplayChar(5,3,'*');
        
        LCD_DisplayText(5,7,"1.0");
        LCD_DisplayText(6,7,"0.5");
        LCD_DisplayText(7,7,"0.0");
        LCD_DisplayText(8,7,"1");
        
        LCD_DisplayText(5,12,"0.0");
        LCD_DisplayText(6,12,"0.0");
        LCD_DisplayText(7,12,"0.0");
}