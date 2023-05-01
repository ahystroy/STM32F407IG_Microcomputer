/* HW04 엘리베이터 제어기 설계 */

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
void _EXTI_Init(void);
void DisplayInitScreen(void);

void CurFL_Init(void);

void Elevator_operation(uint16_t desfl);
void Up(uint16_t desfl);
void Down(uint16_t desfl);
void LEDxOn(uint16_t x);
void LEDxOff(uint16_t x);

uint16_t KEY_Scan(void);
void BEEP(void);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

int main(void)
{
    _GPIO_Init(); 	// GPIO (LED,SW,Buzzer) 초기화
    _EXTI_Init();   // EXTI (EXTI8, EXTI15) 초기화
    LCD_Init();	// LCD 모듈 초기화
    DelayMS(10);

    Fram_Init();            // FRAM 초기화 H/W 초기화
    Fram_Status_Config();   // FRAM 초기화 S/W 초기화
 
    DisplayInitScreen();    // LCD 초기화면
    CurFL_Init();   // FRAM에서 현재 층 정보를 Read하여 GLCD와 LED에 표시

    BEEP(); // Buzzer ON (BEEP() 1회)

    LCD_SetTextColor(RGB_RED);	// 글자색 : RED
    while (1) {
        uint16_t desfl; // 목적지 층 변수
        switch (KEY_Scan()) { // 층 선택 SW 입력에 따른 엘리베이터 동작 (SW1~6은 GPIO)
        case SW1_PUSH: // 1층
            desfl = 1;
            Elevator_operation(desfl);
            break;
        case SW2_PUSH: // 2층
            desfl = 2;
            Elevator_operation(desfl);
            break;
        case SW3_PUSH: // 3층
            desfl = 3;
            Elevator_operation(desfl);
            break;
        case SW4_PUSH: // 4층
            desfl = 4;
            Elevator_operation(desfl);
            break;
        case SW5_PUSH: // 5층
            desfl = 5;
            Elevator_operation(desfl);
            break;
        case SW6_PUSH: // 6층
            desfl = 6;
            Elevator_operation(desfl);
            break;
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
void EXTI15_10_IRQHandler(void) // EXTI15 (SW7) 7층
{
    if (EXTI->PR & 0x008000) 	// EXTI15 Interrupt Pending(발생) 여부?
    {
        EXTI->PR |= 0x8000; 	// Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
        uint16_t desfl = 7;
        if (Fram_Read(024) != desfl) // 현재 층과 같은 Key를 누르면 변화없음
            Elevator_operation(desfl);
    }
}

/* EXTI5~9 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void) // EXTI8 (SW0) 0층
{
    if (EXTI->PR & 0x000100) 	// EXTI8 Interrupt Pending(발생) 여부?
    {
        EXTI->PR |= 0x0100; 	// Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
        uint16_t desfl = 0;
        if (Fram_Read(024) != desfl) // 현재 층과 같은 Key를 누르면 변화없음
            Elevator_operation(desfl);
    }
}

/* GLCD 초기화면 설정 */
void DisplayInitScreen(void)
{
        LCD_Clear(RGB_YELLOW);		// 화면 클리어, 배경색: YELLOW
        LCD_SetFont(&Gulim8);		// 폰트 : 굴림 8
        LCD_SetBackColor(RGB_YELLOW);	// 글자배경색 : YELLOW

        LCD_SetTextColor(RGB_BLUE);	// 글자색 : BLUE
        LCD_DisplayText(0,0,"MC Elevator (AHY)");  // Title

        LCD_SetTextColor(RGB_BLACK);	// 글자색 : Black
        LCD_DisplayText(1,0,"Cur FL: ");  // 현재 층
        LCD_DisplayText(2,0,"Des FL: ");  // 목적지 층

        LCD_SetTextColor(RGB_RED);	// 글자색 : RED
        LCD_DisplayChar(1,8,'0');  // 초기층
        LCD_DisplayChar(2,8,'-');  // 초기
}

/* FRAM에서 현재 층 정보를 Read하여 GLCD와 LED에 표시 */
void CurFL_Init(void)
{
    // FRAM 024번지에서 reset전에 저장된 현재 층 정보를 read
    // FRAM 저장 주소는 학번 끝 3자리 (2019132024)

    LCD_SetTextColor(RGB_RED); // 글자색: RED
    LCD_DisplayChar(1, 8, Fram_Read(024) + 0x30); // 현재 층 GLCD 표시

    GPIOG->ODR &= ~0x00FF;	// LED 초기값: LED0~7 Off
    LEDxOn(Fram_Read(024)); // 현재 층 LED 표시
}

/* 엘리베이터 프로그램 동작 */
void Elevator_operation(uint16_t desfl)
{
    if (Fram_Read(024) == desfl)
        return; // 현재 층과 같은 Key를 누르면 변화없음
    BEEP(); // Buzzer 1회
    LCD_DisplayChar(2, 8, desfl + 0x30); // Des FL 표시
    DelayMS(500);

    /* 엘리베이터 이동 중 */
    if (Fram_Read(024) > desfl) // 현재층이 목적지 층보다 높은 경우
        Down(desfl); // 내려가기
    else if (Fram_Read(024) < desfl) // 현재층이 목적지 층보다 낮은 경우
        Up(desfl); // 올라가기

    /* 목표 층 도착 */
    LCD_DisplayChar(1, 8, desfl + 0x30); // Cur FL 표시
    LCD_DisplayChar(2, 8, '-'); // Des FL는 -로 표시 
    Fram_Write(024, desfl); // Fram에 현재층 정보를 저장 
    // FRAM 저장 주소는 학번 끝 3자리 2019132(024)

    BEEP();
    DelayMS(300);
    BEEP(); // Buzzer 3회 울림
    DelayMS(300);
    BEEP();
}

/* 현재 층에서 목적지 층으로 '내려갈 때' 점멸되는 LED의 번호가 순차적으로 낮아짐 */
void Down(uint16_t desfl)
{
    uint16_t curfl = Fram_Read(024); // 현재 층 정보를 불러옴
    for (; curfl != desfl; curfl--) { // for loop는 목적지 층에 도착할 때까지 반복함
        LEDxOn(curfl-1); // 한 칸 내려가서 LED On
        LEDxOff(curfl); // 그 전 층은 LED Off
        DelayMS(500);
    }
}

/* 현재 층에서 목적지 층으로 '올라갈 때' 점멸되는 LED의 번호가 순차적으로 높아짐 */
void Up(uint16_t desfl)
{
    uint16_t curfl = Fram_Read(024); // 현재 층 정보를 불러옴
    for (; curfl != desfl; curfl++) { // for loop는 목적지 층에 도착할 때까지 반복함
        LEDxOn(curfl+1); // 한 칸 올라가서 LED On
        LEDxOff(curfl); // 그 전 층은 LED Off
        DelayMS(500);
    }
}

/* GPIOG의 LED On */
void LEDxOn(uint16_t x)
{
    if (x == 0)
        GPIOG->ODR |= 0x0001;   // LED0 On
    else if (x == 1)
        GPIOG->ODR |= 0x0002;   // LED1 On
    else if (x == 2)
        GPIOG->ODR |= 0x0004;   // LED2 On
    else if (x == 3)
        GPIOG->ODR |= 0x0008;   // LED3 On
    else if (x == 4)
        GPIOG->ODR |= 0x0010;   // LED4 On
    else if (x == 5)
        GPIOG->ODR |= 0x0020;   // LED5 On
    else if (x == 6)
        GPIOG->ODR |= 0x0040;   // LED6 On
    else
        GPIOG->ODR |= 0x0080;   // LED7 On
}

/* GPIOG의 LED Off */
void LEDxOff(uint16_t x)
{
    if (x == 0)
        GPIOG->ODR &= ~0x0001;   // LED0 Off
    else if (x == 1)
        GPIOG->ODR &= ~0x0002;   // LED1 Off
    else if (x == 2)
        GPIOG->ODR &= ~0x0004;   // LED2 Off
    else if (x == 3)
        GPIOG->ODR &= ~0x0008;   // LED3 Off
    else if (x == 4)
        GPIOG->ODR &= ~0x0010;   // LED4 Off
    else if (x == 5)
        GPIOG->ODR &= ~0x0020;   // LED5 Off
    else if (x == 6)
        GPIOG->ODR &= ~0x0040;   // LED6 Off
    else
        GPIOG->ODR &= ~0x0080;   // LED7 Off
}

/* Switch가 입력되었는지를 여부와 어떤 switch가 입력되었는지의 정보를 return하는 함수 */ 
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
