/////////////////////////////////////////////////////////////
// HW1: 디지털 시계 제작(Stop Watch 기능 포함)
// 제출자: 2019132024 안호연
// 주요 내용
// -Time: TIM2의 Down-Counting mode(1sec) 이용
// -Stop watch: TIM9 Up-counting mode(100msec) 이용 
/////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"

void _RCC_Init(void);
void _GPIO_Init(void);
void _EXTI_Init(void);
void TIMER2_Init(void);
void TIMER9_Init(void);

void DisplayInitScreen(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

uint8_t	SW6_Flag = 0, SW7_Flag = 0; // SW6, 7 Flag 변수


uint8_t time_min10, time_min1, time_sec10, time_sec1;   // Time 시간 변수
uint8_t sw_sec10, sw_sec1, sw_sec_1;     // S/W 시간변수
uint8_t sw_status = 0;                   // 스톱워치의 상태 종류를 표시

int main(void)
{
    _GPIO_Init();  		// GPIO 초기화
    _EXTI_Init();		// 외부인터럽트 초기화
    LCD_Init();		        // GLCD 초기화
    Fram_Init();                // FRAM H/W 초기화
    Fram_Status_Config();       // FRAM S/W 초기화
    DelayMS(10);			
    BEEP();			// Beep 한번 

    GPIOG->ODR &= ~0x007F;	// 초기값: LED0~6 Off
    GPIOG->ODR |= 0x0080;       // 초기값: LED7 On
    
    TIMER2_Init();		// 범용타이머(TIM2) 초기화 : down counting mode
    TIMER9_Init();		// 범용타이머(TIM9) 초기화 : up counting mode
    
    	
    // LCD 초기화면 (TIMER 초기화 이후에 DisplayInit해야 57초부터 Time이 동작해서 부득이하게 TIMER_Init()후에 DisplayInitScreen() 함수 호출하였습니다.)
    DisplayInitScreen();
    
    while(1)
    {
        //EXTI SW6이 High에서 Low가 될 때 (Falling edge Trigger mode)
        if(SW6_Flag) // 작동 or 중단 명령 SW
        {
          SW6_Flag = 0; // Flag Clear
          BEEP(); // BZ 1회
          
          if(sw_status == 0 || sw_status == 1) // 초기화 상태 or 중단 상태에서 SW6 누르면
          { // 작동상태로 동작
            sw_status = 2;        // 작동상태
            GPIOG->ODR |= 0x40;   // LED6 On
            GPIOG->ODR &= ~0x80;  // LED7 Off
            TIM9->CR1 |= (1<<0);  // Start
          }
          else // 작동상태에서 SW6 누르면
          { // 중단상태로 동작
            sw_status = 1;        // 중단상태
            GPIOG->ODR &= ~0xC0;  // LED6,7 Off
            TIM9->CR1 &= ~(1<<0); // Stop
            
            LCD_DisplayChar(2,10,sw_sec10+0x30);
            LCD_DisplayChar(2,11,sw_sec1+0x30);
            LCD_DisplayChar(2,13,sw_sec_1+0x30);  // (xx:x) LCD Update
            
            // Fram에 저장
            Fram_Write(900, sw_sec10); // 900번지: sec의 10의 자리값
            Fram_Write(901, sw_sec1);  // 901번지: sec의 1의 자리값
            Fram_Write(902, sw_sec_1); // 902번지: 0.1sec값 저장
          }
        }
        
        //EXTI SW7이 High에서 Low가 될 때 (Falling edge Trigger mode)
        if(SW7_Flag) // 초기화 명령 SW
        {
          SW7_Flag = 0; // Flag Clear
          
          // 작동상태에서 초기화할 땐, 스톱워치 Stop 시켜야함
          if(sw_status == 2) TIM9->CR1 &= ~(1<<0); // Stop
          
          sw_status = 0; // 초기화 상태
          
          GPIOG->ODR &= ~0x40;  // LED6 Off
          GPIOG->ODR |= 0x80;   // LED7 On
          sw_sec10 = sw_sec1 = sw_sec_1 = 0; // 초기화 명령 시, 00:0
          LCD_DisplayChar(2,4,sw_sec10+0x30);
          LCD_DisplayChar(2,5,sw_sec1+0x30);
          LCD_DisplayChar(2,7,sw_sec_1+0x30);  // S/W Time LCD Update
          BEEP(); // BZ 1회
        }
    }
}

void TIMER2_Init(void)
{
	RCC->APB1ENR |= 0x01;	// RCC_APB1ENR TIMER2 Enable

	// Setting CR1 : 0x0000 
	TIM2->CR1 |= (1<<4);  // DIR=1(Down counter)
	TIM2->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                            //  Counter Overflow/Underflow, 
                            //  Setting the UG bit Set,
                            //  Update Generation through the slave mode controller 
                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM2->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events
                            //	Counter Overflow/Underflow, 
                            // Setting the UG bit Set,
                            //	Update Generation through the slave mode controller 
                            // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM2->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM2->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM2->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM2->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
                            // Center-aligned mode: The counter counts UP and DOWN alternatively


    // Deciding the Period
	TIM2->PSC = 84-1;	// Prescaler 84,000,000Hz/84 = 1 MHz (1us)  (1~65536)
	TIM2->ARR = 1000000;	// Auto reload  1us * 1,000,000 = 1s

   	// Clear the Counter
	TIM2->EGR |= (1<<0);	// UG(Update generation)=1 
                        // Re-initialize the counter(CNT=0) & generates an update of registers   

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1<<28); // Enable Timer2 global Interrupt
 	TIM2->DIER |= (1<<0);	// Enable the Tim2 Update interrupt

        time_min10 = 5; time_min1 = 9; time_sec10 = 5; time_sec1 = 7; // Time 초기값 시각은 59:57 
	TIM2->CR1 |= (1<<0);	// Enable the Tim2 Counter (clock enable)
}

void TIMER9_Init(void)
{
	RCC->APB2ENR |= 0x010000;	// RCC_APB2ENR TIMER9 Enable

	// Setting CR1 : 0x0000 
	TIM9->CR1 &= ~(1<<4);  // DIR=0(Up counter)(reset state)
	TIM9->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
                            //  Counter Overflow/Underflow, 
                            //  Setting the UG bit Set,
                            //  Update Generation through the slave mode controller 
                            // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
	TIM9->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events
                            //	Counter Overflow/Underflow, 
                            // Setting the UG bit Set,
                            //	Update Generation through the slave mode controller 
                            // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
	TIM9->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM9->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM9->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM9->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
                            // Center-aligned mode: The counter counts UP and DOWN alternatively


    // Deciding the Period
	TIM9->PSC = 8400-1;	// Prescaler 168,000,000Hz/8400 = 20 KHz (50us)  (1~65536)
	TIM9->ARR = 2000-1;	// Auto reload  50us * 2000 = 0.1s

   	// Clear the Counter
	TIM9->EGR |= (1<<0);	// UG(Update generation)=1 
                        // Re-initialize the counter(CNT=0) & generates an update of registers   

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1<<24); // Enable Timer9 global Interrupt
 	TIM9->DIER |= (1<<0);	// Enable the Tim9 Update interrupt
        
        sw_sec10 = 0, sw_sec1 = 0, sw_sec_1 = 0; // S/W 시작 시간 초기화
        TIM9->CR1 &= ~(1<<0);	// Disable the Tim9 Counter (clock disable)   
}

void TIM2_IRQHandler(void)  	// 1s Interrupt
{
	TIM2->SR &= ~(1<<0);	// Interrupt flag Clear
        
        time_sec1++; // 1초마다 핸들러 실행 되므로 Time '1초 변수' 1씩 증가
        LCD_DisplayChar(1,9, time_sec1+0x30); // LCD Update
        if(time_sec1 >= 10)
        {       time_sec1 = 0; // 10초가 되면 '1초 변수' 0으로 초기화
                LCD_DisplayChar(1,9, time_sec1+0x30); // LCD Update
                time_sec10++; // 10초마다 '10초 변수' 1씩 증가
                LCD_DisplayChar(1,8, time_sec10+0x30); // LCD Update
                if(time_sec10 >= 6)
                {       time_sec10 = 0; // 60초가 되면 '10초 변수' 0으로 초기화
                        LCD_DisplayChar(1,8, time_sec10+0x30); // LCD Update
                        time_min1++; // 60초마다 '1분 변수' 1씩 증가
                        LCD_DisplayChar(1,6, time_min1+0x30); // LCD Update
                        if(time_min1 >= 10)
                        {       time_min1 = 0; // 10분이 되면 '1분 변수' 0으로 초기화
                                LCD_DisplayChar(1,6, time_min1+0x30); // LCD Update
                                time_min10++; // 10분마다 '10분 변수' 1씩 증가
                                LCD_DisplayChar(1,5, time_min10+0x30); // LCD Update
                                if(time_min10 >= 6)
                                {       time_min10 = 0; // 60분이 되면 '10분 변수' 0으로 초기화
                                        LCD_DisplayChar(1,5, time_min10+0x30); // LCD Update
                                }
                        }
                }
        }
}

void TIM1_BRK_TIM9_IRQHandler(void)  	// 0.1s Interrupt
{
	TIM9->SR &= ~(1<<0);	// Interrupt flag Clear
        
        sw_sec_1++; // 0.1초마다 핸들러 실행 되므로 s/w '0.1초 변수' 1씩 증가
        LCD_DisplayChar(2,7, sw_sec_1+0x30); // LCD Update
        if(sw_sec_1 >= 10)
        {       sw_sec_1 = 0; // 0.1 * 10 = 1초가 되면 '0.1초 변수' 0으로 초기화
                LCD_DisplayChar(2,7, sw_sec_1+0x30); // LCD Update
                sw_sec1++; // 1초마다 '1초 변수' 1씩 증가
                LCD_DisplayChar(2,5, sw_sec1+0x30); // LCD Update
                if(sw_sec1 >= 10)
                {       sw_sec1 = 0; // 10초가 되면 '1초 변수' 0으로 초기화
                        LCD_DisplayChar(2,5, sw_sec1+0x30); // LCD Update
                        sw_sec10++; // 10초마다 '10초 변수' 1씩 증가
                        LCD_DisplayChar(2,4, sw_sec10+0x30); // LCD Update
                        if(sw_sec10 >= 6)
                        {       sw_sec10 = 0; // 60초가 되면 '10초 변수' 0으로 초기화
                                LCD_DisplayChar(2,4, sw_sec10+0x30); // LCD Update
                        }
                }
        }
}

void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	    // GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) 설정 
    RCC->AHB1ENR	|=  0x00000020;     // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	    // GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}	

/* EXTI 초기 설정  */
/* EXTI14(GPIOH.14, SW14), EXTI15(GPIOH.15, SW7) */
void _EXTI_Init(void)
{
    /* RCC setting */
    RCC->AHB1ENR |= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
    RCC->APB2ENR |= 0x00004000;	// System Configuration(SYSCFG) Controller Clock Enable

    /* GPIOH Input Mode setting*/
    GPIOH->MODER &= ~0xF0000000;	// GPIOH 14,15 : Input mode (reset state)

    /* EXTI Setting */
    // 1. SYSCFG->EXTICR[x] setting
    SYSCFG->EXTICR[3] |= 0x7700; 	// EXTI14, EXTI15에 대한 소스 입력은 GPIOH로 설정

    // 2. EXTI 세부 설정
    EXTI->FTSR |= 0x00C000;		// EXTI14,15: Falling Trigger Enable
    EXTI->IMR |= 0x00C000;  	// EXTI14,15 인터럽트 mask (Interrupt Enable) 설정

    // 3. NVIC setting
    NVIC->ISER[1] |= (1 << (40 - 32));  // Enable 'Global Interrupt EXTI14,15'
    // Interrupt vector table에서 
    // EXTI14,15에 해당하는(EXTI15_10 interrupts) exception의 Interrupt number은 40
    // NVIC->ISER[1]이므로 NIVC->ISER[0]에 해당하는 비트 수 만큼 빼준다.
}

void EXTI15_10_IRQHandler(void)		// EXTI 10~15 인터럽트 핸들러
{
	if(EXTI->PR & 0x4000) 		// EXTI14 nterrupt Pending?
	{
		EXTI->PR |= 0x4000; 	// Pending bit Clear
		SW6_Flag = 1;
	}
	else if(EXTI->PR & 0x8000) 	// EXTI15 Interrupt Pending?
	{
		EXTI->PR |= 0x8000; 	// Pending bit Clear
		SW7_Flag = 1;	
	}
}

void BEEP(void)			/* beep for 30 ms */
{ 	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(30);		// Delay 30 ms
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
    LCD_Clear(RGB_YELLOW);	// 화면 클리어
    LCD_SetFont(&Gulim8);	// 폰트 : 굴림 8
    LCD_SetBackColor(RGB_YELLOW);// 글자배경색 : Green
    LCD_SetTextColor(RGB_BLUE);// 글자색 : Black
    LCD_DisplayText(0,0,"AHY 2019132024");  // Title

    LCD_SetTextColor(RGB_RED);	        //글자색 : Red
    LCD_DisplayText(1,0,"Time");
    LCD_DisplayText(2,0,"S/W");  
    
    LCD_SetTextColor(RGB_BLACK);	//글자색 : Black
    LCD_DisplayText(1,5,"59 57");
    LCD_DisplayText(2,4,"00 0 (    )");
    
    // Fram에 저장된 값을 Read하여 화면에 표시
    LCD_DisplayChar(2,10, Fram_Read(900)+0x30); // 900번지: sec의 10의 자리값
    LCD_DisplayChar(2,11, Fram_Read(901)+0x30); // 901번지: sec의 1의 자리값
    LCD_DisplayChar(2,13, Fram_Read(902)+0x30); // 902번지: 0.1sec값 저장
    
    LCD_SetTextColor(RGB_RED);	        //글자색 : Red
    LCD_DisplayChar(1,7,':');
    LCD_DisplayChar(2,6,':');
    LCD_DisplayChar(2,12,':');
    
    LCD_SetTextColor(RGB_BLACK);	//글자색 : Black 
    // 앞으로 LCD에 Update할 문자는 시간 표시 문자로 모두 Black 임
}