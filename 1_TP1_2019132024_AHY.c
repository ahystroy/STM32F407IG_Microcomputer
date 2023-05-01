/* TP1. Two Elevators */

#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"
//#include <stdlib.h>

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

#define LE 1
#define RE 2    // 두 엘리베이터 중 어느 엘리베이터가 움직일지 정해주는 상수

void _GPIO_Init(void);
void _EXTI_Init(void);
void DisplayInitScreen(void);

void FillRect(uint8_t fl, uint8_t l_r);

void SetStartFL(int sf);
void SetDesFL(int df);

void Move(uint8_t start, uint8_t end, uint8_t e);

uint16_t KEY_Scan(void);
void BEEP(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

// 출발층, 목표층 정의 (0층일때 층을 선택할 수 있게함)
// 즉, 0층은 아직 층이 선택되지 않았음을 의미
uint8_t StartFloor = 0;
uint8_t DesFloor = 0;

// 각각의 엘리베이터의 현재 층 정의
uint8_t LEFloor, REFloor;

// 스위치0 Flag 선언 (EXTI)
uint8_t SW0_Flag;

int main(void)
{
    _GPIO_Init(); 	// GPIO (LED,SW,Buzzer) 초기화
    _EXTI_Init();   // EXTI (EXTI8, EXTI15) 초기화
    LCD_Init();	// LCD 모듈 초기화
    DelayMS(10);

    Fram_Init();            // FRAM 초기화 H/W 초기화
    Fram_Status_Config();   // FRAM 초기화 S/W 초기화
    
    // 초기 LED 상태: LED7(ON), LED0~6(OFF)
    GPIOG->ODR &= ~0xFF00;	// LED0~6 Off
    GPIOG->ODR |= 0x0080;       // LED7 On
    
    DisplayInitScreen();    // LCD 초기화면

    while (1) {
        /* 층선택 모드 (FL) */

        /* 출발층이 아직 선택되지 않았을 때 출발층 먼저 설정 */
        if (StartFloor == 0) {
            switch (KEY_Scan()) {
            case SW1_PUSH: SetStartFL(SW1_PUSH);
                break;
            case SW2_PUSH: SetStartFL(SW2_PUSH);
                break;
            case SW3_PUSH: SetStartFL(SW3_PUSH);
                break;
            case SW4_PUSH: SetStartFL(SW4_PUSH);
                break;
            case SW5_PUSH: SetStartFL(SW5_PUSH);
                break;
            case SW6_PUSH: SetStartFL(SW6_PUSH);
                break;
            }
        }

        /* 출발층이 선택되고 목표층이 선택되지 않았을때 목표층을 그 다음으로 설정 */
        if (StartFloor != 0 && DesFloor == 0) {
            switch (KEY_Scan()) {
            case SW1_PUSH: SetDesFL(SW1_PUSH);
                break;
            case SW2_PUSH: SetDesFL(SW2_PUSH);
                break;
            case SW3_PUSH: SetDesFL(SW3_PUSH);
                break;
            case SW4_PUSH: SetDesFL(SW4_PUSH);
                break;
            case SW5_PUSH: SetDesFL(SW5_PUSH);
                break;
            case SW6_PUSH: SetDesFL(SW6_PUSH);
                break;
            }
        }

        /* 인터럽트 EXTI8 발생 (실행모드) */
        if (SW0_Flag)
        {
            // 층 선택이 완료 되었을때 실행모드로 전환
            if (StartFloor != 0 && DesFloor != 0) {
                GPIOG->ODR |= 0x0001;   // LED0 On
                GPIOG->ODR &= ~0x0080;   // LED7 Off

                LCD_SetTextColor(RGB_RED);	// 글자색 : RED
                LCD_DisplayText(1, 8, "EX");    // 실행 모드

                /* 왼쪽, 오른쪽 엘리베이터 중에서 어느 엘리베이터가 이동할지 결정 */
                int LEoper, REoper;
                if (StartFloor - LEFloor >= 0)
                    LEoper = StartFloor - LEFloor;
                else
                    LEoper = LEFloor - StartFloor;
                if (StartFloor - REFloor >= 0)
                    REoper = StartFloor - REFloor;
                else
                    REoper = REFloor - StartFloor;
                /* <stdlib.h>의 abs()를 이용하여 구할수도 있음 */

                if (LEoper <= REoper) {
                    // 출발층이 왼쪽 엘리베이터에서 더 가깝거나 거리가 같은 경우
                    // LE 이동
                    Move(LEFloor, StartFloor, LE); // 출발층으로 이동
                    Move(StartFloor, DesFloor, LE); // 목표층으로 이동
                    LEFloor = DesFloor;
                    Fram_Write(1001, LEFloor); // 도착 후 왼쪽 엘리베이터의 현재층 저장
                }
                else {
                    // 출발층이 오른쪽 엘리베이터에서 더 가까운 경우
                    // RE 이동
                    Move(REFloor, StartFloor, RE); // 출발층으로 이동
                    Move(StartFloor, DesFloor, RE); // 목표층으로 이동
                    REFloor = DesFloor;
                    Fram_Write(1002, REFloor); // 도착 후 오른쪽 엘리베이터의 현재층 저장
                }
                LCD_SetTextColor(RGB_RED);	// 글자색 : RED
                LCD_DisplayText(1, 8, "FL");    // 실행이 종료되면 층 선택 모드로 변경

                GPIOG->ODR &= ~0x007F;   // LED0~6 Off
                GPIOG->ODR |= 0x0080;   // LED7 On

                BEEP();
                DelayMS(100);
                BEEP();  // 부저 3번 울림
                DelayMS(100);
                BEEP();

                // 출발층과 목표층을 새로 Setting하기 위해 0으로 초기화
                StartFloor = 0;
                DesFloor = 0;
            }
            SW0_Flag = 0; // 실행모드에서 빠져나옴을 의미
        }
    }
}

/* GPIO (GPIOG(LED), GPIOH(Switch), GPIOF(Buzzer), GPIOI(Joy stick)) 초기 설정	*/
void _GPIO_Init(void)
{
    // LED (GPIO G) 설정
    RCC->AHB1ENR |= 0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
    GPIOG->MODER |= 0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
    GPIOG->OTYPER &= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
    GPIOG->OSPEEDR |= 0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 

    // SW (GPIO H) 설정 
    RCC->AHB1ENR |= 0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
    GPIOH->MODER &= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
    GPIOH->PUPDR &= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

    // Buzzer (GPIO F) 설정 
    RCC->AHB1ENR |= 0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
    GPIOF->MODER |= 0x00040000;	// GPIOF 9 : Output mode (0b01)						
    GPIOF->OTYPER &= ~0x0200;	// GPIOF 9 : Push-pull  	
    GPIOF->OSPEEDR |= 0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}

    // "EXTI 초기설정은 지난 HW04때와 동일한 코드입니다!"
/* EXTI (EXTI8(GPIOH.8, SW0), EXTI15(GPIOH.15, SW7)) 초기 설정  */
void _EXTI_Init(void)
{
    /* RCC setting */
    RCC->AHB1ENR |= 0x00000080;	// RCC_AHB1ENR GPIOH Enable
    RCC->APB2ENR |= 0x00004000;	// System Configuration(SYSCFG) Controller Clock Enable

    /* GPIOH Input Mode setting*/
    GPIOH->MODER &= ~0xFFFF0000;	// GPIOH PIN8~PIN15 Input mode (reset state)				 
    
    /*=============================================================================*/

    /* EXTI8 setting */
    // 1. SYSCFG->EXTICR[x] setting
    SYSCFG->EXTICR[2] |= 0x0007; 	// EXTI8에 대한 소스 입력은 GPIOH로 설정

    // 2. EXTI 세부 설정
    EXTI->FTSR |= 0x000100;		// EXTI8: Falling Trigger Enable 
    EXTI->IMR |= 0x000300;  		// EXTI8,9 인터럽트 mask (Interrupt Enable) 설정

    // 3. NVIC setting
    NVIC->ISER[0] |= (1 << 23);  // Enable 'Global Interrupt EXTI8,9'
    // Interrupt vector table에서 
    // EXTI8에 해당하는(EXTI9_5 interrupts) exception의 Interrupt number은 23

    /*=============================================================================*/

    /* EXTI15 setting */
    // 1. SYSCFG->EXTICR[x] setting
    SYSCFG->EXTICR[3] |= 0x7000; 	// EXTI15에 대한 소스 입력은 GPIOH로 설정

    // 2. EXTI 세부 설정
    EXTI->FTSR |= 0x008000;		// EXTI15: Falling Trigger Enable 
    EXTI->IMR |= 0x008000;  		// EXTI15 인터럽트 mask (Interrupt Enable) 설정

    // 3. NVIC setting
    NVIC->ISER[1] |= (1 << (40 - 32));  // Enable 'Global Interrupt EXTI15'
    // Interrupt vector table에서 
    // EXTI8에 해당하는(EXTI15_10 interrupts) exception의 Interrupt number은 40
    // NVIC->ISER[1]이므로 NIVC->ISER[0]에 해당하는 비트 수 만큼 빼준다.
}

/* EXTI10~15 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void) // EXTI15 (SW7) 중단 모드 (Hold)
{
    // SW0_Flag가 1일때, 즉 EXTI8(실행모드) 중에만 중단모드로 전환되기 위해 if문 추가
    if (SW0_Flag) {
        if (EXTI->PR & 0x008000) 	// EXTI15 Interrupt Pending(발생) 여부?
        {
            EXTI->PR |= 0x8000; 	// Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
            
            GPIOG->ODR &= ~0x0001;   // LED0 Off
            GPIOG->ODR |= 0x0080;   // LED7 On -> '중단 모드'

            LCD_SetTextColor(RGB_RED);	// 글자색 : RED
            LCD_DisplayText(1, 8, "HD");    // 실행 모드

            // DelayMS(500) 10회 반복 -> 5초 중단
            for (int i = 0; i < 10; i++) {
                BEEP();     // 중단된 5초간 부저가 0.5초 간격으로 울림
                DelayMS(500);
            }
            GPIOG->ODR |= 0x0001;   // LED0 On -> '실행 모드'
            GPIOG->ODR &= ~0x0080;   // LED7 Off

            LCD_SetTextColor(RGB_RED);	// 글자색 : RED
            LCD_DisplayText(1, 8, "EX");    // 중단 모드가 끝난 후 다시 실행 모드 마저 진행
        }
    }
}

/* EXTI5~9 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void) // EXTI8 (SW0) 실행 모드 (Execute)
{
    if (EXTI->PR & 0x000100) 	// EXTI8 Interrupt Pending(발생) 여부?
    {
        EXTI->PR |= 0x0100; 	// Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
        SW0_Flag = 1;   // 실행 모드가 진행중임을 알리는 Flag
    }
}

/* GLCD 초기화면 설정 */
void DisplayInitScreen(void)
{
        LCD_Clear(RGB_YELLOW);		// 화면 클리어, 배경색: YELLOW
        LCD_SetFont(&Gulim8);		// 폰트 : 굴림 8
        LCD_SetBackColor(RGB_YELLOW);	// 글자배경색 : YELLOW

        LCD_SetTextColor(RGB_BLACK);	// 글자색 : BLACK
        LCD_DisplayText(0,0,"MC-Elevator(AHY)");  // Title

        LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
        LCD_DisplayChar(1, 4, '6');
        LCD_DisplayChar(2, 4, '5');
        LCD_DisplayChar(3, 4, '4');
        LCD_DisplayChar(4, 4, '3');
        LCD_DisplayChar(5, 4, '2');
        LCD_DisplayChar(6, 4, '1');
        LCD_DisplayText(3, 8, "L-E");   // Left Elevator

        LCD_SetTextColor(RGB_RED);	// 글자색 : RED
        LCD_DisplayText(1, 8, "FL");    // 층 선택 모드
        LCD_DisplayChar(4, 9, 'S');     // Stop 정지 상태
        LCD_DisplayChar(5, 8, Fram_Read(1001) + 0x30); // Fram에서 층을 불러와서 LCD에 표기
        LCD_DisplayChar(5, 9, '>');     // ‘FRAM 1001번지 값 > FRAM 1001번지 값’
        LCD_DisplayChar(5, 10, Fram_Read(1001) + 0x30); // Fram에서 층을 불러와서 LCD에 표기

        LCD_SetTextColor(RGB_GREEN);	// 글자색 : GREEN
        LCD_DisplayChar(1, 14, '6');
        LCD_DisplayChar(2, 14, '5');
        LCD_DisplayChar(3, 14, '4');
        LCD_DisplayChar(4, 14, '3');
        LCD_DisplayChar(5, 14, '2');
        LCD_DisplayChar(6, 14, '1');    // Right Elevator

        LCD_SetPenColor(RGB_BLUE);    // Left Elevator = BLUE bar
        LCD_SetBrushColor(RGB_BLUE);
        FillRect(Fram_Read(1001), 0); // 현재층 정보에 따라 LCD에 엘리베이터 bar를 채움
        LEFloor = Fram_Read(1001); // 왼쪽 엘리베이터의 현재층을 FRAM에서 불러와 저장
        
        LCD_SetPenColor(RGB_GREEN);    // Right Elevator = GREEN bar
        LCD_SetBrushColor(RGB_GREEN);
        FillRect(Fram_Read(1002), 1); // 현재층 정보에 따라 LCD에 엘리베이터 bar를 채움
        REFloor = Fram_Read(1002); // 오른쪽 엘리베이터의 현재층을 FRAM에서 불러와 저장
}

/* 엘리베이터 종류(L,R)와 층 정보를 매개변수로 받아 LCD 초기화면 구성시에 동작 */
void FillRect(uint8_t fl, uint8_t l_r) // 매개변수 => (층, 왼쪽or오른쪽)
{
    if (l_r == 0) 
        LCD_DrawFillRect(16, 79 - (13 * (fl-1)), 10, 11 + (13 * (fl-1)));
    else 
        LCD_DrawFillRect(126, 79 - (13 * (fl-1)), 10, 11 + (13 * (fl-1)));
}

/* 출발층을 설정해주는 함수 */
/* 해당 층 LED On, LCD에 출발층 표기 */
void SetStartFL(int sf)
{
    LCD_SetTextColor(RGB_RED);
    switch (sf) {
    case SW1_PUSH: // 1층
        GPIOG->ODR |= 0x0002;   // LED1 On
        LCD_DisplayChar(5, 8, '1');  // LCD에 출발층 1층 표기
        StartFloor = 1;
        break;
    case SW2_PUSH: // 2층
        GPIOG->ODR |= 0x0004;   // LED2 On
        LCD_DisplayChar(5, 8, '2');  // LCD에 출발층 2층 표기
        StartFloor = 2;
        break;
    case SW3_PUSH: // 3층
        GPIOG->ODR |= 0x0008;   // LED3 On
        LCD_DisplayChar(5, 8, '3');  // LCD에 출발층 3층 표기
        StartFloor = 3;
        break;
    case SW4_PUSH: // 4층
        GPIOG->ODR |= 0x0010;   // LED4 On
        LCD_DisplayChar(5, 8, '4');  // LCD에 출발층 4층 표기
        StartFloor = 4;
        break;
    case SW5_PUSH: // 5층
        GPIOG->ODR |= 0x0020;   // LED5 On
        LCD_DisplayChar(5, 8, '5');  // LCD에 출발층 5층 표기
        StartFloor = 5;
        break;
    case SW6_PUSH: // 6층
        GPIOG->ODR |= 0x0040;   // LED6 On
        LCD_DisplayChar(5, 8, '6');  // LCD에 출발층 6층 표기
        StartFloor = 6;
        break;
    }
    BEEP();
}

/* 목표층을 설정해주는 함수 */
/* 해당 층 LED On, LCD에 목표층 표기 */
void SetDesFL(int df)
{
    LCD_SetTextColor(RGB_RED);
    switch (df) {
    case SW1_PUSH: // 1층
        if (StartFloor == 1) return; // 출발층과 목표층이 같을시에는 return
        GPIOG->ODR |= 0x0002;   // LED1 On
        LCD_DisplayChar(5, 10, '1');  // LCD에 출발층 1층 표기
        DesFloor = 1;
        break;
    case SW2_PUSH: // 2층
        if (StartFloor == 2) return; // 출발층과 목표층이 같을시에는 return
        GPIOG->ODR |= 0x0004;   // LED2 On
        LCD_DisplayChar(5, 10, '2');  // LCD에 출발층 2층 표기
        DesFloor = 2;
        break;
    case SW3_PUSH: // 3층
        if (StartFloor == 3) return; // 출발층과 목표층이 같을시에는 return
        GPIOG->ODR |= 0x0008;   // LED3 On
        LCD_DisplayChar(5, 10, '3');  // LCD에 출발층 3층 표기
        DesFloor = 3;
        break;
    case SW4_PUSH: // 4층
        if (StartFloor == 4) return; // 출발층과 목표층이 같을시에는 return
        GPIOG->ODR |= 0x0010;   // LED4 On
        LCD_DisplayChar(5, 10, '4');  // LCD에 출발층 4층 표기
        DesFloor = 4;
        break;
    case SW5_PUSH: // 5층
        if (StartFloor == 5) return; // 출발층과 목표층이 같을시에는 return
        GPIOG->ODR |= 0x0020;   // LED5 On
        LCD_DisplayChar(5, 10, '5');  // LCD에 출발층 5층 표기
        DesFloor = 5;
        break;
    case SW6_PUSH: // 6층
        if (StartFloor == 6) return; // 출발층과 목표층이 같을시에는 return
        GPIOG->ODR |= 0x0040;   // LED6 On
        LCD_DisplayChar(5, 10, '6');  // LCD에 출발층 6층 표기
        DesFloor = 6;
        break;
    }
    BEEP();
}

/* 엘리베이터가 위-아래로 동작하는 함수 */
/* (시작층, 도착층, 엘리베이터 종류)를 매개변수로 받아서 동작 */
void Move(uint8_t start, uint8_t end, uint8_t e)
{
    uint8_t start_ = start;
    uint8_t end_ = end;
    if (e == LE) {
        LCD_SetTextColor(RGB_BLUE);
        LCD_DisplayText(3, 8, "L-E");   // Left Elevator
        if (start > end) { // Down
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayChar(4, 9, 'D'); // LCD 표기
            DelayMS(500);
            for (start_--; start_ != end_ - 1; start_--) { // for loop는 목적지 층에 도착할 때까지 반복함
                LCD_SetBrushColor(RGB_YELLOW); // 내려갈때는 배경색으로 bar를 덮음
                LCD_DrawFillRect(16, 14, 10, 13 * (6-start_)); // 엘리베이터 동작을 LCD에 표현
                DelayMS(500);   // 0.5초 지연
            }
        }
        else if (start < end) { // Up
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayChar(4, 9, 'U'); // LCD 표기
            DelayMS(500);
            for (start_++; start_ != end_ + 1; start_++) { // for loop는 목적지 층에 도착할 때까지 반복함
                LCD_SetBrushColor(RGB_BLUE); // 올라갈때는 색을 채움
                LCD_DrawFillRect(16, 79 - (13 * (start_-1)), 10, 11 + (13 * (start_-1))); // 엘리베이터 동작을 LCD에 표현
                DelayMS(500);   // 0.5초 지연
            }
        }
        else return; 
        // 시작층과 도착층이 같은경우는 return
        // Ex) 엘리베이터의 현재 층이 출발층인 경우
    }
    else if (e == RE) {
        LCD_SetTextColor(RGB_GREEN);
        LCD_DisplayText(3, 8, "R-E");   // Right Elevator
        if (start > end) { // Down
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayChar(4, 9, 'D'); // LCD 표기
            DelayMS(500);
            for (start_--; start_ != end_ - 1; start_--) { // for loop는 목적지 층에 도착할 때까지 반복함
                LCD_SetBrushColor(RGB_YELLOW); // 내려갈때는 배경색으로 bar를 덮음
                LCD_DrawFillRect(126, 14, 10, 13 * (6 - start_)); // 엘리베이터 동작을 LCD에 표현
                DelayMS(500);   // 0.5초 지연
            }
        }
        else if (start < end) { // Up
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayChar(4, 9, 'U'); // LCD 표기
            DelayMS(500);
            for (start_++; start_ != end_ + 1; start_++) { // for loop는 목적지 층에 도착할 때까지 반복함
                LCD_SetBrushColor(RGB_GREEN); // 올라갈때는 색을 채움
                LCD_DrawFillRect(126, 79 - (13 * (start_-1)), 10, 11 + (13 * (start_-1))); // 엘리베이터 동작을 LCD에 표현
                DelayMS(500);   // 0.5초 지연
            }
        }
        else return;
        // 시작층과 도착층이 같은경우는 return
        // Ex) 엘리베이터의 현재 층이 출발층인 경우
    }
    LCD_SetTextColor(RGB_RED);
    LCD_DisplayChar(4, 9, 'S');    // 움직이고자하는 층에 도착하였으면 Stop
    DelayMS(500);   // 0.5초 지연
}

/* Switch가 입력되었는지를 여부와 어떤 switch가 입력되었는지의 정보를 return하는 함수 */ 
uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	
        if(key_flag == 0)
        		return key;
      	else {	
            DelayMS(10);
        	key_flag = 0;
        	return key;
        }
    }
  	else				// if key input, check continuous key
	{	
        if(key_flag != 0)	// if continuous key, treat as no key input
        	return 0xFF00;
        else {			// if new key,delay for debounce
            key_flag = 1;
			DelayMS(10);
 			return key;
        }
	} 
}

/* Buzzer: Beep for 30 ms */
void BEEP(void)			
{ 	
	GPIOF->ODR |=  0x0200;	// PF9 'H' Buzzer on
	DelayMS(30);		// Delay 30 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);         		// 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
    	for(; Dly; Dly--);
}
