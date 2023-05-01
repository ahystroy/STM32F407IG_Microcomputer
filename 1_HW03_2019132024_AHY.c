#include "stm32f4xx.h"
#include "GLCD.h" // LCD ���� ȸ�翡�� ����

void _GPIO_Init(void);
uint16_t KEY_Scan(void);

void BEEP(void);
void DisplayInitScreen(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

uint8_t	SW0_Flag, SW1_Flag, SW7_Flag; 	
// Switch �Է��� Ȧ����°����? ¦����°����? �� �˱����� ���� 

int main(void)
{
	_GPIO_Init();           // GPIO (LED, SW, Buzzer) �ʱ�ȭ
	LCD_Init();             // LCD ��� �ʱ�ȭ  (GLCD.h�� ���� �Ǿ�����)
	DelayMS(100);		

	GPIOG->ODR &= ~0x00FF;	// LED �ʱⰪ: LED0~7 Off
	DisplayInitScreen();	// GLCD �ʱ�ȭ�� ����

	while (1)
	{
		switch (KEY_Scan())	// �Էµ� Switch ���� �з� 
		{
		case 0x7F00: // SW7 : Coin ����
			if (SW7_Flag == 0) {
				GPIOG->BSRRL = 0x0080;			// LED7(Coin LED) ON
				LCD_DisplayChar(1, 6, 'O');		// Coin: O ǥ��
				BEEP();							// Buzzer Beep
				SW7_Flag = 1;	// Coin�� ���Ե� ���¸� ǥ��
			}
			break;

		case 0xFE00: // SW0: Black Coffee
			if (SW7_Flag == 1) {
				GPIOG->ODR |= 0x0001;	// LED0(Black Coffee) ON
				LCD_DisplayText(2, 0, "Black-C: ");	// GLCD�� Coffee�� ǥ��
				BEEP();	// Buzzer 1ȸ

				/*Ŀ�Ǽ��� SW Ŭ���ϰ� 1���Ŀ� Ŀ�ǵ���(�ַ����̵�/����)*/
				/*1*/
				DelayMS(1000);			        // 1����
				GPIOG->ODR |= 0x0008;		// LED3(Cup) On
				LCD_DisplayChar(2, 9, 'U');	// GLCD 'U'ǥ��
				DelayMS(1000);			        // 1����
				GPIOG->ODR &= ~0x0008;	// LED3(Cup) Off
				LCD_DisplayChar(2, 9, ' ');	// GLCD 'U'����

				/*2*/
				GPIOG->ODR |= 0x0010;		// LED4(Coffee) On
				LCD_DisplayChar(2, 9, 'C');	// GLCD 'C'ǥ��
				DelayMS(2000);			        // 2����
				GPIOG->ODR &= ~0x0010;	// LED4(Coffee) Off
				LCD_DisplayChar(2, 9, ' ');	// GLCD 'C'����

				/*3*/
				GPIOG->ODR |= 0x0040;		// LED6(Water) On
				LCD_DisplayChar(2, 9, 'W');	// GLCD 'W'ǥ��
				DelayMS(2000);			        // 2����
				GPIOG->ODR &= ~0x0040;	// LED6(Water) Off
				LCD_DisplayChar(2, 9, ' ');	// GLCD 'W'����

				/*4*/
				BEEP();
				DelayMS(500);	// 0.5�� �������� Buzzer 2�� �︲
				BEEP();

				/*5*/
				DelayMS(1000);				        // 1����
				LCD_DisplayText(2, 0, "           ");	// GLCD 'Coffee��' ����
				GPIOG->ODR &= ~0x0001;		// LED0(Black Coffee) Off

				/*6*/
				DelayMS(1000);			        // 1����
				GPIOG->BSRRH = 0x0080;	// LED7(Coin LED) Off
				LCD_DisplayChar(1, 6, 'X');	// GLCD 'Coin: X' ǥ��
				
				SW7_Flag = 0;	// Coin�� ���Ե��� ���� ���¸� ǥ��
			}
			break;

		case 0xFD00: // SW1: Sugar Coffee
			if (SW7_Flag == 1) {
				GPIOG->ODR |= 0x0002;	                // LED1(Sugar Coffee) ON
				LCD_DisplayText(2, 0, "Sugar-C: ");	// GLCD�� Coffee�� ǥ��
				BEEP();	// Buzzer 1ȸ
				
				/*Ŀ�Ǽ��� SW Ŭ���ϰ� 1���Ŀ� Ŀ�ǵ���(�ַ����̵�/����)*/
				/*1*/
				DelayMS(1000);			        // 1����
				GPIOG->ODR |= 0x0008;		// LED3(Cup) On
				LCD_DisplayChar(2, 9, 'U');	// GLCD 'U'ǥ��
				DelayMS(1000);			        // 1����
				GPIOG->ODR &= ~0x0008;	// LED3(Cup) Off
				LCD_DisplayChar(2, 9, ' ');	// GLCD 'U'����

				/*2*/
				GPIOG->ODR |= 0x0010;		// LED4(Coffee) On
				LCD_DisplayChar(2, 9, 'C');	// GLCD 'C'ǥ��
				DelayMS(2000);			        // 2����
				GPIOG->ODR &= ~0x0010;	// LED4(Coffee) Off
				LCD_DisplayChar(2, 9, ' ');	// GLCD 'C'����

				/*3*/
				GPIOG->ODR |= 0x0020;		// LED5(Sugar) On
				LCD_DisplayChar(2, 9, 'S');	// GLCD 'S'ǥ��
				DelayMS(1000);			        // 1����
				GPIOG->ODR &= ~0x0020;	// LED5(Sugar) Off
				LCD_DisplayChar(2, 9, ' ');	// GLCD 'S'����

				/*4*/
				GPIOG->ODR |= 0x0040;		// LED6(Water) On
				LCD_DisplayChar(2, 9, 'W');	// GLCD 'W'ǥ��
				DelayMS(2000);			        // 2����
				GPIOG->ODR &= ~0x0040;	// LED6(Water) Off
				LCD_DisplayChar(2, 9, ' ');	// GLCD 'W'����

				/*5*/
				BEEP();
				DelayMS(500);	// 0.5�� �������� Buzzer 2�� �︲
				BEEP();

				/*6*/
				DelayMS(1000);				        // 1����
				LCD_DisplayText(2, 0, "           ");	// GLCD 'Coffee��' ����
				GPIOG->ODR &= ~0x0002;		// LED1(Sugar Coffee) Off

				/*7*/
				DelayMS(1000);			        // 1����
				GPIOG->BSRRH = 0x0080;	// LED7(Coin LED) Off
				LCD_DisplayChar(1, 6, 'X');	// GLCD 'Coin: X' ǥ��

				SW7_Flag = 0;	// Coin�� ���Ե��� ���� ���¸� ǥ��
			}
			break;
		}
	}
}

/* GPIO (GPIOG(LED), GPIOH(Switch), GPIOF(Buzzer)) �ʱ� ���� */
void _GPIO_Init(void)
{
	// LED (GPIO G) ����
	RCC->AHB1ENR |= 0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER |= 0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER &= ~0x00FF;        // GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
	GPIOG->OSPEEDR |= 0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 

	// SW (GPIO H) ���� 
	RCC->AHB1ENR |= 0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER &= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR &= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) ���� 
	RCC->AHB1ENR |= 0x00000020;      // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER |= 0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER &= ~0x0200;	        // GPIOF 9 : Push-pull  	
	GPIOF->OSPEEDR |= 0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}

/* GLCD �ʱ�ȭ�� ���� �Լ� */
void DisplayInitScreen(void)
{
	LCD_Clear(RGB_YELLOW);		// �������� : YELLOW
	LCD_SetFont(&Gulim8);			// ��Ʈ : ���� 8
	LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� : YELLOW
	LCD_SetTextColor(RGB_BLUE);		// ���ڻ� : BLUE
	LCD_DisplayText(0, 0, "Coffee Vendor");  	// title

	LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : BLACK
	LCD_DisplayText(1, 0, "Coin: ");

	LCD_SetTextColor(RGB_RED);		// ���ڻ� : RED
	LCD_DisplayChar(1, 6, 'X');
}

/* Switch�� �ԷµǾ������� ���ο� � switch�� �ԷµǾ������� ������ return�ϴ� �Լ�  */
uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if (key == 0xFF00)	// if no key, check key off
	{
		if (key_flag == 0)
			return key;
		else
		{
			DelayMS(10);
			key_flag = 0;
			return key;
		}
	}
	else	// if key input, check continuous key
	{
		if (key_flag != 0)	// if continuous key, treat as no key input
			return 0xFF00;
		else	// if new key,delay for debounce
		{
			key_flag = 1;
			DelayMS(10);
			return key;
		}
	}
}

/* Buzzer: Beep for 30 ms */
void BEEP(void)
{
	GPIOF->ODR |= 0x0200;	        // PF9 'H' Buzzer on
	DelayMS(30);		                // Delay 30 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i = 0; i < wMS; i++)
		DelayUS(1000);         // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS * 17;
	for (; Dly; Dly--);
}