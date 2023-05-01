/* TP2 OMOK GAME 2019132024_안호연 */

#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"

#define SW1_PUSH        0xFD00  //PH9
#define SW6_PUSH        0xBF00  //PH14

#define NAVI_PUSH	0x03C0  //PI5 0000 0011 1100 0000 
#define NAVI_UP		0x03A0  //PI6 0000 0011 1010 0000 
#define NAVI_DOWN       0x0360  //PI7 0000 0011 0110 0000 
#define NAVI_RIGHT	0x02E0  //PI8 0000 0010 1110 0000 
#define NAVI_LEFT	0x01E0  //PI9 0000 0001 1110 0000 

void _GPIO_Init(void);
void _EXTI_Init(void);
void DisplayInitScreen(void);
void Stone_Init(void);

uint16_t KEY_Scan(void);
uint16_t JOY_Scan(void);

void BEEP(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

uint8_t	SW0_Flag; // 적돌 선택
uint8_t SW7_Flag; // 청돌 선택

typedef struct {
    int x;
    int y;
}Stone;
Stone R; // 적돌 좌표
Stone B; // 청돌 좌표
Stone X[81]; // 착돌된 좌표 기록
int index; // X[index]

int Rw; // 적돌 승리 횟수
int Bw; // 청돌 승리 횟수

int main(void)
{
    _GPIO_Init(); // GPIO (LED & SW) 초기화
    _EXTI_Init(); // EXTI 초기화
    LCD_Init();	// LCD 모듈 초기화
    Fram_Init(); // FRAM H/W 초기화
    Fram_Status_Config(); // FRAM S/W 초기화

    DisplayInitScreen();	// LCD 초기화면
    Stone_Init(); // 오목 좌표 관련 초기화

    while (1) {
        switch (JOY_Scan()) {
        case NAVI_UP: // y-1
            if (SW0_Flag) { // 적돌 선택
                BEEP();
                R.y--;
                if (R.y < 0) R.y = 0; // 최소:0
                // 글자색 선택은 EXTI핸들러에서 돌 색별로 이미 지정했으므로 생략. 이하 동일
                LCD_DisplayChar(9, 5, R.y + 0x30); // 변경된 좌표 표기
            }
            else if (SW7_Flag) { // 청돌 선택
                BEEP();
                B.y--;
                if (B.y < 0) B.y = 0; // 최소:0
                LCD_DisplayChar(9, 16, B.y + 0x30); // 변경된 좌표 표기
            }
            break;
        case NAVI_DOWN: // y+1
            if (SW0_Flag) { // 적돌 선택
                BEEP();
                R.y++;
                if (R.y > 9) R.y = 9; // 최대:9
                LCD_DisplayChar(9, 5, R.y + 0x30); // 변경된 좌표 표기
            }
            else if (SW7_Flag) { // 청돌 선택
                BEEP();
                B.y++;
                if (B.y > 9) B.y = 9; // 최대:9
                LCD_DisplayChar(9, 16, B.y + 0x30); // 변경된 좌표 표기
            }
            break;
        case NAVI_LEFT: // x-1
            if (SW0_Flag) { // 적돌 선택
                BEEP();
                R.x--;
                if (R.x < 0) R.x = 0; // 최소:0
                LCD_DisplayChar(9, 3, R.x + 0x30); // 변경된 좌표 표기
            }
            else if (SW7_Flag) { // 청돌 선택
                BEEP();
                B.x--;
                if (B.x < 0) B.x = 0; // 최소:0
                LCD_DisplayChar(9, 14, B.x + 0x30); // 변경된 좌표 표기
            }
            break;
        case NAVI_RIGHT: // x+1
            if (SW0_Flag) { // 적돌 선택
                BEEP();
                R.x++;
                if (R.x > 9) R.x = 9; // 최대:9
                LCD_DisplayChar(9, 3, R.x + 0x30); // 변경된 좌표 표기
            }
            else if (SW7_Flag) { // 청돌 선택
                BEEP();
                B.x++;
                if (B.x > 9) B.x = 9; // 최대:9
                LCD_DisplayChar(9, 14, B.x + 0x30); // 변경된 좌표 표기
            }
            break;
        } // switch (JOY_Scan())
        switch (KEY_Scan()) {
        case SW1_PUSH: // 적돌 승
            LCD_SetTextColor(RGB_RED); // 적돌
            if (++Rw > 9) Rw = 0; // 9 다음은 0
            LCD_DisplayChar(9, 8, Rw + 0x30); // 적돌 점수 표기
            Fram_Write(300, Rw); // Fram에 기록
            BEEP();
            DelayMS(150);
            BEEP();
            DelayMS(150);
            BEEP(); // 5-Buzzer
            DelayMS(150);
            BEEP();
            DelayMS(150);
            BEEP();
            DelayMS(5000); // 5초후 프로그램 재시작
            DisplayInitScreen(); // 오목판 초기화
            Stone_Init(); // 오목알 초기화
            break;
        case SW6_PUSH: // 청돌 승
            LCD_SetTextColor(RGB_BLUE); // 청돌
            if (++Bw > 9) Bw = 0; // 9 다음은 0
            LCD_DisplayChar(9, 11, Bw + 0x30); // 청돌 점수 표기
            Fram_Write(301, Bw); // Fram에 기록
            BEEP();
            DelayMS(150);
            BEEP();
            DelayMS(150);
            BEEP(); // 5-Buzzer
            DelayMS(150);
            BEEP();
            DelayMS(150);
            BEEP();
            DelayMS(5000); // 5초후 프로그램 재시작
            DisplayInitScreen(); // 오목판 초기화
            Stone_Init(); // 오목알 초기화
            break;
        } // switch (KEY_Scan())
    } // while(1)
}

/* GPIO 초기설정 */
void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
        RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00004001;	// GPIOG 0,7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00000081;	// GPIOG 0,7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00004001;	// GPIOG 0,7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xF00F0000;	// GPIOH 8,9,14,15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xF00F0000;	// GPIOH 8,9,14,15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) 설정 
        RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x00000200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 

	//Joy Stick SW (PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	&= ~0x000FFC00;	// GPIOI 5~9 : Input mode (reset state)
	GPIOI->PUPDR    &= ~0x000FFC00;	// GPIOI 5~9 : Floating input (No Pull-up, pull-down) (reset state)
}	

/* EXTI 초기 설정  */
/* EXTI5(GPIOI.5, J-P), EXTI8(GPIOH.8, SW0), EXTI15(GPIOH.15, SW7) */
void _EXTI_Init(void)
{
    /* RCC setting */
    RCC->AHB1ENR |= 0x00000180;	// RCC_AHB1ENR GPIOH, GPIOI Enable
    RCC->APB2ENR |= 0x00004000;	// System Configuration(SYSCFG) Controller Clock Enable

    /* GPIOH Input Mode setting*/
    GPIOH->MODER &= ~0xC0030000;	// GPIOH 8,15 : Input mode (reset state)				 
    GPIOI->MODER &= ~0x00000C00;	// GPIOI 5 : Input mode (reset state)

    /* EXTI Setting */
    // 1. SYSCFG->EXTICR[x] setting
    SYSCFG->EXTICR[1] |= 0x0080;    // EXTI5에 대한 소스 입력은 GPIOI로 설정
    SYSCFG->EXTICR[2] |= 0x0007; 	// EXTI8에 대한 소스 입력은 GPIOH로 설정
    SYSCFG->EXTICR[3] |= 0x7000; 	// EXTI15에 대한 소스 입력은 GPIOH로 설정

    // 2. EXTI 세부 설정
    EXTI->FTSR |= 0x008120;		// EXTI5,8,15: Falling Trigger Enable
    EXTI->IMR |= 0x008120;  	// EXTI5,8,15 인터럽트 mask (Interrupt Enable) 설정

    // 3. NVIC setting
    NVIC->ISER[0] |= (1 << 23);  // Enable 'Global Interrupt EXTI5 and EXTI8'
    // Interrupt vector table에서 
    // EXTI5와 EXTI8에 해당하는(EXTI9_5 interrupts) exception의 Interrupt number은 23
    NVIC->ISER[1] |= (1 << (40 - 32));  // Enable 'Global Interrupt EXTI15'
    // Interrupt vector table에서 
    // EXTI15에 해당하는(EXTI15_10 interrupts) exception의 Interrupt number은 40
    // NVIC->ISER[1]이므로 NIVC->ISER[0]에 해당하는 비트 수 만큼 빼준다.
}

/* EXTI10~15 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void) // EXTI15 (SW7)
{
    if (EXTI->PR & 0x008000) 	// EXTI15 Interrupt Pending(발생) 여부?
    {
        EXTI->PR |= 0x8000; 	// Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
        SW0_Flag = 0;   // 적돌 선택 x
        SW7_Flag = 1;   // 청돌 선택임을 알리는 Flag
        
        GPIOG->ODR &= ~0x0001; // LED0 Off
        GPIOG->ODR |= 0x0080; // LED7 On
        
        LCD_SetTextColor(RGB_BLUE); // 청돌 선택시 글자색은 BLUE
        LCD_DisplayChar(9,1,' ');
        LCD_DisplayChar(9,18,'*'); // 청돌 선택 표시
    }
}

/* EXTI5~9 인터럽트 핸들러(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void) // EXTI5 (JP), EXTI8 (SW0)
{
    if (EXTI->PR & 0x000020) 	// EXTI5 Interrupt Pending(발생) 여부?
    {
        EXTI->PR |= 0x0020; 	// Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
        int z = 1; // 착돌 가능 여부를 판단하는 변수
        if (SW0_Flag) { // (Rx,Ry)에 적돌 착돌
            for (int i = 0; i < index; i++) { // for문을 돌면서 착돌하려는 위치가 이미 착돌되어 있는지 여부 판단
                if (X[i].x == R.x && X[i].y == R.y) { 
                    // 착돌하려는 위치에 이미 돌이 놓여있는 경우
                    DelayMS(1000); // 1초후
                    BEEP();
                    DelayMS(150);
                    BEEP(); // 3-Buzzer
                    DelayMS(150);
                    BEEP();
                    z = -1; // z = -1 이면 착돌 불가
                    break;
                }
            }
            if (z) { // 착돌 가능
                LCD_SetBrushColor(RGB_RED); // 적돌
                LCD_DrawFillRect(36 + R.x * 9 - 3, 27 + R.y * 9 - 3, 7, 7); // 착돌
                BEEP();
                X[index++] = R; // 착돌된 위치 기록
            }
        }
        else if (SW7_Flag) {  // (Bx,By)에 청돌 착돌
            for (int i = 0; i < index; i++) { // for문을 돌면서 착돌하려는 위치가 이미 착돌되어 있는지 여부 판단
                if (X[i].x == B.x && X[i].y == B.y) { 
                    // 착돌하려는 위치에 이미 돌이 놓여있는 경우
                    DelayMS(1000); // 1초후
                    BEEP();
                    DelayMS(150);
                    BEEP(); // 3-Buzzer
                    DelayMS(150);
                    BEEP();
                    z = -1; // z = -1 이면 착돌 불가
                    break;
                }
            }
            if (z) { // 착돌 가능
                LCD_SetBrushColor(RGB_BLUE); // 청돌
                LCD_DrawFillRect(36 + B.x * 9 - 3, 27 + B.y * 9 - 3, 7, 7); // 착돌
                BEEP();
                X[index++] = B; // 착돌된 위치 기록
            }
        }
    }
    if (EXTI->PR & 0x000100) 	// EXTI8 Interrupt Pending(발생) 여부?
    {
        EXTI->PR |= 0x0100; 	// Pending bit Clear (clear를 안하면 인터럽트 수행후 다시 인터럽트 발생)
        SW0_Flag = 1;   // 적돌 선택임을 알리는 Flag
        SW7_Flag = 0;   // 청돌 선택x
        
        GPIOG->ODR |= 0x0001; // LED0 On
        GPIOG->ODR &= ~0x0080; // LED7 Off
        
        LCD_SetTextColor(RGB_RED); // 적돌 선택시 글자색은 RED
        LCD_DisplayChar(9,1,'*'); // 적돌 선택 표시
        LCD_DisplayChar(9,18,' ');
    }
}

/* GLCD 초기화면 설정 */
void DisplayInitScreen(void)
{
    LCD_Clear(RGB_YELLOW);	// 화면 클리어, 배경 YELLOW
    LCD_SetFont(&Gulim8);	// 폰트 : 굴림 8
    LCD_SetBackColor(RGB_YELLOW); // 글자배경색 : YELLOW

    LCD_SetTextColor(RGB_BLACK); // 글자색 : Black
    LCD_DisplayText(0, 0, "Mecha-OMOK(AHY)");  // Title
    LCD_DisplayChar(1, 4, '0');
    LCD_DisplayChar(1, 10, '5');
    LCD_DisplayChar(1, 14, '9');
    LCD_DisplayChar(5, 3, '5');
    LCD_DisplayChar(8, 3, '9'); // 이상 오목판 index
    LCD_DisplayText(9, 9, "vs"); // vs

    LCD_SetPenColor(RGB_BLACK);
    LCD_DrawRectangle(36, 27, 81, 81); // 오목판 테두리
    for (int i = 1; i < 9; i++) { // for문을 돌면서 오목판 내부를 그림
        LCD_DrawHorLine(36, 27 + (9 * i), 81);
        LCD_DrawVerLine(36 + (9 * i), 27, 81);
    }
    LCD_SetBrushColor(RGB_BLACK);
    LCD_DrawFillRect(79,70,5,5); // 좌표(5,5)의 검은색 중앙점
    
    // 적돌
    LCD_SetTextColor(RGB_RED);
    LCD_DisplayText(9,1," (5,5)");
    Rw = Fram_Read(300); // Fram에서 적돌 승리 횟수 불러옴
    LCD_DisplayChar(9,8,Rw + 0x30); // 점수 표기
    
    // 청돌
    LCD_SetTextColor(RGB_BLUE);
    LCD_DisplayText(9,13,"(5,5) ");
    Bw = Fram_Read(301); // Fram에서 청돌 승리 횟수 불러옴
    LCD_DisplayChar(9,11,Bw + 0x30); // 점수 표기
}

/* 오목알 좌표 관련 초기화 */
void Stone_Init(void)
{
    // 착돌할 위치 좌표 초기화
    R.x = R.y = B.x = B.y = 5;

    // 착돌된 위치 기록 초기화
    index = 0;
    for (int i = 0; i < 81; i++)
        X[i].x = X[i].y = -1; // -1은 착돌되지 않음을 의미

    // 적돌, 청돌 선택 여부 초기화
    SW0_Flag = SW7_Flag = 0;
    GPIOG->ODR &= ~0x0081;	// LED 초기값: LED0,7 Off
}

uint8_t joy_flag = 0;
uint16_t JOY_Scan(void)	// input joy stick NAVI_* 
{ 
	uint16_t key;
	key = GPIOI->IDR & 0x03E0;	// any key pressed ? (5 6 7 8 9 -> 0x03E0)
	if(key == 0x03E0)		// if no key, check key off
	{  	if(joy_flag == 0)
        		return key;
      		else
		{	DelayMS(10);
        		joy_flag = 0;
        		return key;
        	}
    	}
  	else				// if key input, check continuous key
	{	if(joy_flag != 0)	// if continuous key, treat as no key input
        		return 0x03E0;
      		else			// if new key,delay for debounce
		{	joy_flag = 1;
			DelayMS(10);
 			return key;
        	}
	}
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

void BEEP(void)			/* beep for 30 ms */
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
