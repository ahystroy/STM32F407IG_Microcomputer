/* TP2 OMOK GAME 2019132024_��ȣ�� */

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

uint8_t	SW0_Flag; // ���� ����
uint8_t SW7_Flag; // û�� ����

typedef struct {
    int x;
    int y;
}Stone;
Stone R; // ���� ��ǥ
Stone B; // û�� ��ǥ
Stone X[81]; // ������ ��ǥ ���
int index; // X[index]

int Rw; // ���� �¸� Ƚ��
int Bw; // û�� �¸� Ƚ��

int main(void)
{
    _GPIO_Init(); // GPIO (LED & SW) �ʱ�ȭ
    _EXTI_Init(); // EXTI �ʱ�ȭ
    LCD_Init();	// LCD ��� �ʱ�ȭ
    Fram_Init(); // FRAM H/W �ʱ�ȭ
    Fram_Status_Config(); // FRAM S/W �ʱ�ȭ

    DisplayInitScreen();	// LCD �ʱ�ȭ��
    Stone_Init(); // ���� ��ǥ ���� �ʱ�ȭ

    while (1) {
        switch (JOY_Scan()) {
        case NAVI_UP: // y-1
            if (SW0_Flag) { // ���� ����
                BEEP();
                R.y--;
                if (R.y < 0) R.y = 0; // �ּ�:0
                // ���ڻ� ������ EXTI�ڵ鷯���� �� ������ �̹� ���������Ƿ� ����. ���� ����
                LCD_DisplayChar(9, 5, R.y + 0x30); // ����� ��ǥ ǥ��
            }
            else if (SW7_Flag) { // û�� ����
                BEEP();
                B.y--;
                if (B.y < 0) B.y = 0; // �ּ�:0
                LCD_DisplayChar(9, 16, B.y + 0x30); // ����� ��ǥ ǥ��
            }
            break;
        case NAVI_DOWN: // y+1
            if (SW0_Flag) { // ���� ����
                BEEP();
                R.y++;
                if (R.y > 9) R.y = 9; // �ִ�:9
                LCD_DisplayChar(9, 5, R.y + 0x30); // ����� ��ǥ ǥ��
            }
            else if (SW7_Flag) { // û�� ����
                BEEP();
                B.y++;
                if (B.y > 9) B.y = 9; // �ִ�:9
                LCD_DisplayChar(9, 16, B.y + 0x30); // ����� ��ǥ ǥ��
            }
            break;
        case NAVI_LEFT: // x-1
            if (SW0_Flag) { // ���� ����
                BEEP();
                R.x--;
                if (R.x < 0) R.x = 0; // �ּ�:0
                LCD_DisplayChar(9, 3, R.x + 0x30); // ����� ��ǥ ǥ��
            }
            else if (SW7_Flag) { // û�� ����
                BEEP();
                B.x--;
                if (B.x < 0) B.x = 0; // �ּ�:0
                LCD_DisplayChar(9, 14, B.x + 0x30); // ����� ��ǥ ǥ��
            }
            break;
        case NAVI_RIGHT: // x+1
            if (SW0_Flag) { // ���� ����
                BEEP();
                R.x++;
                if (R.x > 9) R.x = 9; // �ִ�:9
                LCD_DisplayChar(9, 3, R.x + 0x30); // ����� ��ǥ ǥ��
            }
            else if (SW7_Flag) { // û�� ����
                BEEP();
                B.x++;
                if (B.x > 9) B.x = 9; // �ִ�:9
                LCD_DisplayChar(9, 14, B.x + 0x30); // ����� ��ǥ ǥ��
            }
            break;
        } // switch (JOY_Scan())
        switch (KEY_Scan()) {
        case SW1_PUSH: // ���� ��
            LCD_SetTextColor(RGB_RED); // ����
            if (++Rw > 9) Rw = 0; // 9 ������ 0
            LCD_DisplayChar(9, 8, Rw + 0x30); // ���� ���� ǥ��
            Fram_Write(300, Rw); // Fram�� ���
            BEEP();
            DelayMS(150);
            BEEP();
            DelayMS(150);
            BEEP(); // 5-Buzzer
            DelayMS(150);
            BEEP();
            DelayMS(150);
            BEEP();
            DelayMS(5000); // 5���� ���α׷� �����
            DisplayInitScreen(); // ������ �ʱ�ȭ
            Stone_Init(); // ����� �ʱ�ȭ
            break;
        case SW6_PUSH: // û�� ��
            LCD_SetTextColor(RGB_BLUE); // û��
            if (++Bw > 9) Bw = 0; // 9 ������ 0
            LCD_DisplayChar(9, 11, Bw + 0x30); // û�� ���� ǥ��
            Fram_Write(301, Bw); // Fram�� ���
            BEEP();
            DelayMS(150);
            BEEP();
            DelayMS(150);
            BEEP(); // 5-Buzzer
            DelayMS(150);
            BEEP();
            DelayMS(150);
            BEEP();
            DelayMS(5000); // 5���� ���α׷� �����
            DisplayInitScreen(); // ������ �ʱ�ȭ
            Stone_Init(); // ����� �ʱ�ȭ
            break;
        } // switch (KEY_Scan())
    } // while(1)
}

/* GPIO �ʱ⼳�� */
void _GPIO_Init(void)
{
	// LED (GPIO G) ����
        RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00004001;	// GPIOG 0,7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00000081;	// GPIOG 0,7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00004001;	// GPIOG 0,7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) ���� 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xF00F0000;	// GPIOH 8,9,14,15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xF00F0000;	// GPIOH 8,9,14,15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) ���� 
        RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x00000200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 

	//Joy Stick SW (PORT I) ����
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	&= ~0x000FFC00;	// GPIOI 5~9 : Input mode (reset state)
	GPIOI->PUPDR    &= ~0x000FFC00;	// GPIOI 5~9 : Floating input (No Pull-up, pull-down) (reset state)
}	

/* EXTI �ʱ� ����  */
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
    SYSCFG->EXTICR[1] |= 0x0080;    // EXTI5�� ���� �ҽ� �Է��� GPIOI�� ����
    SYSCFG->EXTICR[2] |= 0x0007; 	// EXTI8�� ���� �ҽ� �Է��� GPIOH�� ����
    SYSCFG->EXTICR[3] |= 0x7000; 	// EXTI15�� ���� �ҽ� �Է��� GPIOH�� ����

    // 2. EXTI ���� ����
    EXTI->FTSR |= 0x008120;		// EXTI5,8,15: Falling Trigger Enable
    EXTI->IMR |= 0x008120;  	// EXTI5,8,15 ���ͷ�Ʈ mask (Interrupt Enable) ����

    // 3. NVIC setting
    NVIC->ISER[0] |= (1 << 23);  // Enable 'Global Interrupt EXTI5 and EXTI8'
    // Interrupt vector table���� 
    // EXTI5�� EXTI8�� �ش��ϴ�(EXTI9_5 interrupts) exception�� Interrupt number�� 23
    NVIC->ISER[1] |= (1 << (40 - 32));  // Enable 'Global Interrupt EXTI15'
    // Interrupt vector table���� 
    // EXTI15�� �ش��ϴ�(EXTI15_10 interrupts) exception�� Interrupt number�� 40
    // NVIC->ISER[1]�̹Ƿ� NIVC->ISER[0]�� �ش��ϴ� ��Ʈ �� ��ŭ ���ش�.
}

/* EXTI10~15 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void) // EXTI15 (SW7)
{
    if (EXTI->PR & 0x008000) 	// EXTI15 Interrupt Pending(�߻�) ����?
    {
        EXTI->PR |= 0x8000; 	// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
        SW0_Flag = 0;   // ���� ���� x
        SW7_Flag = 1;   // û�� �������� �˸��� Flag
        
        GPIOG->ODR &= ~0x0001; // LED0 Off
        GPIOG->ODR |= 0x0080; // LED7 On
        
        LCD_SetTextColor(RGB_BLUE); // û�� ���ý� ���ڻ��� BLUE
        LCD_DisplayChar(9,1,' ');
        LCD_DisplayChar(9,18,'*'); // û�� ���� ǥ��
    }
}

/* EXTI5~9 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void) // EXTI5 (JP), EXTI8 (SW0)
{
    if (EXTI->PR & 0x000020) 	// EXTI5 Interrupt Pending(�߻�) ����?
    {
        EXTI->PR |= 0x0020; 	// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
        int z = 1; // ���� ���� ���θ� �Ǵ��ϴ� ����
        if (SW0_Flag) { // (Rx,Ry)�� ���� ����
            for (int i = 0; i < index; i++) { // for���� ���鼭 �����Ϸ��� ��ġ�� �̹� �����Ǿ� �ִ��� ���� �Ǵ�
                if (X[i].x == R.x && X[i].y == R.y) { 
                    // �����Ϸ��� ��ġ�� �̹� ���� �����ִ� ���
                    DelayMS(1000); // 1����
                    BEEP();
                    DelayMS(150);
                    BEEP(); // 3-Buzzer
                    DelayMS(150);
                    BEEP();
                    z = -1; // z = -1 �̸� ���� �Ұ�
                    break;
                }
            }
            if (z) { // ���� ����
                LCD_SetBrushColor(RGB_RED); // ����
                LCD_DrawFillRect(36 + R.x * 9 - 3, 27 + R.y * 9 - 3, 7, 7); // ����
                BEEP();
                X[index++] = R; // ������ ��ġ ���
            }
        }
        else if (SW7_Flag) {  // (Bx,By)�� û�� ����
            for (int i = 0; i < index; i++) { // for���� ���鼭 �����Ϸ��� ��ġ�� �̹� �����Ǿ� �ִ��� ���� �Ǵ�
                if (X[i].x == B.x && X[i].y == B.y) { 
                    // �����Ϸ��� ��ġ�� �̹� ���� �����ִ� ���
                    DelayMS(1000); // 1����
                    BEEP();
                    DelayMS(150);
                    BEEP(); // 3-Buzzer
                    DelayMS(150);
                    BEEP();
                    z = -1; // z = -1 �̸� ���� �Ұ�
                    break;
                }
            }
            if (z) { // ���� ����
                LCD_SetBrushColor(RGB_BLUE); // û��
                LCD_DrawFillRect(36 + B.x * 9 - 3, 27 + B.y * 9 - 3, 7, 7); // ����
                BEEP();
                X[index++] = B; // ������ ��ġ ���
            }
        }
    }
    if (EXTI->PR & 0x000100) 	// EXTI8 Interrupt Pending(�߻�) ����?
    {
        EXTI->PR |= 0x0100; 	// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
        SW0_Flag = 1;   // ���� �������� �˸��� Flag
        SW7_Flag = 0;   // û�� ����x
        
        GPIOG->ODR |= 0x0001; // LED0 On
        GPIOG->ODR &= ~0x0080; // LED7 Off
        
        LCD_SetTextColor(RGB_RED); // ���� ���ý� ���ڻ��� RED
        LCD_DisplayChar(9,1,'*'); // ���� ���� ǥ��
        LCD_DisplayChar(9,18,' ');
    }
}

/* GLCD �ʱ�ȭ�� ���� */
void DisplayInitScreen(void)
{
    LCD_Clear(RGB_YELLOW);	// ȭ�� Ŭ����, ��� YELLOW
    LCD_SetFont(&Gulim8);	// ��Ʈ : ���� 8
    LCD_SetBackColor(RGB_YELLOW); // ���ڹ��� : YELLOW

    LCD_SetTextColor(RGB_BLACK); // ���ڻ� : Black
    LCD_DisplayText(0, 0, "Mecha-OMOK(AHY)");  // Title
    LCD_DisplayChar(1, 4, '0');
    LCD_DisplayChar(1, 10, '5');
    LCD_DisplayChar(1, 14, '9');
    LCD_DisplayChar(5, 3, '5');
    LCD_DisplayChar(8, 3, '9'); // �̻� ������ index
    LCD_DisplayText(9, 9, "vs"); // vs

    LCD_SetPenColor(RGB_BLACK);
    LCD_DrawRectangle(36, 27, 81, 81); // ������ �׵θ�
    for (int i = 1; i < 9; i++) { // for���� ���鼭 ������ ���θ� �׸�
        LCD_DrawHorLine(36, 27 + (9 * i), 81);
        LCD_DrawVerLine(36 + (9 * i), 27, 81);
    }
    LCD_SetBrushColor(RGB_BLACK);
    LCD_DrawFillRect(79,70,5,5); // ��ǥ(5,5)�� ������ �߾���
    
    // ����
    LCD_SetTextColor(RGB_RED);
    LCD_DisplayText(9,1," (5,5)");
    Rw = Fram_Read(300); // Fram���� ���� �¸� Ƚ�� �ҷ���
    LCD_DisplayChar(9,8,Rw + 0x30); // ���� ǥ��
    
    // û��
    LCD_SetTextColor(RGB_BLUE);
    LCD_DisplayText(9,13,"(5,5) ");
    Bw = Fram_Read(301); // Fram���� û�� �¸� Ƚ�� �ҷ���
    LCD_DisplayChar(9,11,Bw + 0x30); // ���� ǥ��
}

/* ����� ��ǥ ���� �ʱ�ȭ */
void Stone_Init(void)
{
    // ������ ��ġ ��ǥ �ʱ�ȭ
    R.x = R.y = B.x = B.y = 5;

    // ������ ��ġ ��� �ʱ�ȭ
    index = 0;
    for (int i = 0; i < 81; i++)
        X[i].x = X[i].y = -1; // -1�� �������� ������ �ǹ�

    // ����, û�� ���� ���� �ʱ�ȭ
    SW0_Flag = SW7_Flag = 0;
    GPIOG->ODR &= ~0x0081;	// LED �ʱⰪ: LED0,7 Off
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
