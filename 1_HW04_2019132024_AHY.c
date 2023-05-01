/* HW04 ���������� ����� ���� */

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
    _GPIO_Init(); 	// GPIO (LED,SW,Buzzer) �ʱ�ȭ
    _EXTI_Init();   // EXTI (EXTI8, EXTI15) �ʱ�ȭ
    LCD_Init();	// LCD ��� �ʱ�ȭ
    DelayMS(10);

    Fram_Init();            // FRAM �ʱ�ȭ H/W �ʱ�ȭ
    Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
 
    DisplayInitScreen();    // LCD �ʱ�ȭ��
    CurFL_Init();   // FRAM���� ���� �� ������ Read�Ͽ� GLCD�� LED�� ǥ��

    BEEP(); // Buzzer ON (BEEP() 1ȸ)

    LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
    while (1) {
        uint16_t desfl; // ������ �� ����
        switch (KEY_Scan()) { // �� ���� SW �Է¿� ���� ���������� ���� (SW1~6�� GPIO)
        case SW1_PUSH: // 1��
            desfl = 1;
            Elevator_operation(desfl);
            break;
        case SW2_PUSH: // 2��
            desfl = 2;
            Elevator_operation(desfl);
            break;
        case SW3_PUSH: // 3��
            desfl = 3;
            Elevator_operation(desfl);
            break;
        case SW4_PUSH: // 4��
            desfl = 4;
            Elevator_operation(desfl);
            break;
        case SW5_PUSH: // 5��
            desfl = 5;
            Elevator_operation(desfl);
            break;
        case SW6_PUSH: // 6��
            desfl = 6;
            Elevator_operation(desfl);
            break;
        }
    }
}

/* GPIO (GPIOG(LED), GPIOH(Switch), GPIOF(Buzzer), GPIOI(Joy stick)) �ʱ� ����	*/
void _GPIO_Init(void)
{
    // LED (GPIO G) ����
    RCC->AHB1ENR |= 0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
    GPIOG->MODER |= 0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
    GPIOG->OTYPER &= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
    GPIOG->OSPEEDR |= 0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 

    // SW (GPIO H) ���� 
    RCC->AHB1ENR |= 0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
    GPIOH->MODER &= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
    GPIOH->PUPDR &= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

    // Buzzer (GPIO F) ���� 
    RCC->AHB1ENR |= 0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
    GPIOF->MODER |= 0x00040000;	// GPIOF 9 : Output mode (0b01)						
    GPIOF->OTYPER &= ~0x0200;	// GPIOF 9 : Push-pull  	
    GPIOF->OSPEEDR |= 0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
}

/* EXTI (EXTI8(GPIOH.8, SW0), EXTI15(GPIOH.15, SW7)) �ʱ� ����  */
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
    SYSCFG->EXTICR[2] |= 0x0007; 	// EXTI8�� ���� �ҽ� �Է��� GPIOH�� ����

    // 2. EXTI ���� ����
    EXTI->FTSR |= 0x000100;		// EXTI8: Falling Trigger Enable 
    EXTI->IMR |= 0x000300;  		// EXTI8,9 ���ͷ�Ʈ mask (Interrupt Enable) ����

    // 3. NVIC setting
    NVIC->ISER[0] |= (1 << 23);  // Enable 'Global Interrupt EXTI8,9'
    // Interrupt vector table���� 
    // EXTI8�� �ش��ϴ�(EXTI9_5 interrupts) exception�� Interrupt number�� 23

    /*=============================================================================*/

    /* EXTI15 setting */
    // 1. SYSCFG->EXTICR[x] setting
    SYSCFG->EXTICR[3] |= 0x7000; 	// EXTI15�� ���� �ҽ� �Է��� GPIOH�� ����

    // 2. EXTI ���� ����
    EXTI->FTSR |= 0x008000;		// EXTI15: Falling Trigger Enable 
    EXTI->IMR |= 0x008000;  		// EXTI15 ���ͷ�Ʈ mask (Interrupt Enable) ����

    // 3. NVIC setting
    NVIC->ISER[1] |= (1 << (40 - 32));  // Enable 'Global Interrupt EXTI15'
    // Interrupt vector table���� 
    // EXTI8�� �ش��ϴ�(EXTI15_10 interrupts) exception�� Interrupt number�� 40
    // NVIC->ISER[1]�̹Ƿ� NIVC->ISER[0]�� �ش��ϴ� ��Ʈ �� ��ŭ ���ش�.
}

/* EXTI10~15 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI15_10_IRQHandler(void) // EXTI15 (SW7) 7��
{
    if (EXTI->PR & 0x008000) 	// EXTI15 Interrupt Pending(�߻�) ����?
    {
        EXTI->PR |= 0x8000; 	// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
        uint16_t desfl = 7;
        if (Fram_Read(024) != desfl) // ���� ���� ���� Key�� ������ ��ȭ����
            Elevator_operation(desfl);
    }
}

/* EXTI5~9 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void) // EXTI8 (SW0) 0��
{
    if (EXTI->PR & 0x000100) 	// EXTI8 Interrupt Pending(�߻�) ����?
    {
        EXTI->PR |= 0x0100; 	// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
        uint16_t desfl = 0;
        if (Fram_Read(024) != desfl) // ���� ���� ���� Key�� ������ ��ȭ����
            Elevator_operation(desfl);
    }
}

/* GLCD �ʱ�ȭ�� ���� */
void DisplayInitScreen(void)
{
        LCD_Clear(RGB_YELLOW);		// ȭ�� Ŭ����, ����: YELLOW
        LCD_SetFont(&Gulim8);		// ��Ʈ : ���� 8
        LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� : YELLOW

        LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
        LCD_DisplayText(0,0,"MC Elevator (AHY)");  // Title

        LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : Black
        LCD_DisplayText(1,0,"Cur FL: ");  // ���� ��
        LCD_DisplayText(2,0,"Des FL: ");  // ������ ��

        LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
        LCD_DisplayChar(1,8,'0');  // �ʱ���
        LCD_DisplayChar(2,8,'-');  // �ʱ�
}

/* FRAM���� ���� �� ������ Read�Ͽ� GLCD�� LED�� ǥ�� */
void CurFL_Init(void)
{
    // FRAM 024�������� reset���� ����� ���� �� ������ read
    // FRAM ���� �ּҴ� �й� �� 3�ڸ� (2019132024)

    LCD_SetTextColor(RGB_RED); // ���ڻ�: RED
    LCD_DisplayChar(1, 8, Fram_Read(024) + 0x30); // ���� �� GLCD ǥ��

    GPIOG->ODR &= ~0x00FF;	// LED �ʱⰪ: LED0~7 Off
    LEDxOn(Fram_Read(024)); // ���� �� LED ǥ��
}

/* ���������� ���α׷� ���� */
void Elevator_operation(uint16_t desfl)
{
    if (Fram_Read(024) == desfl)
        return; // ���� ���� ���� Key�� ������ ��ȭ����
    BEEP(); // Buzzer 1ȸ
    LCD_DisplayChar(2, 8, desfl + 0x30); // Des FL ǥ��
    DelayMS(500);

    /* ���������� �̵� �� */
    if (Fram_Read(024) > desfl) // �������� ������ ������ ���� ���
        Down(desfl); // ��������
    else if (Fram_Read(024) < desfl) // �������� ������ ������ ���� ���
        Up(desfl); // �ö󰡱�

    /* ��ǥ �� ���� */
    LCD_DisplayChar(1, 8, desfl + 0x30); // Cur FL ǥ��
    LCD_DisplayChar(2, 8, '-'); // Des FL�� -�� ǥ�� 
    Fram_Write(024, desfl); // Fram�� ������ ������ ���� 
    // FRAM ���� �ּҴ� �й� �� 3�ڸ� 2019132(024)

    BEEP();
    DelayMS(300);
    BEEP(); // Buzzer 3ȸ �︲
    DelayMS(300);
    BEEP();
}

/* ���� ������ ������ ������ '������ ��' ����Ǵ� LED�� ��ȣ�� ���������� ������ */
void Down(uint16_t desfl)
{
    uint16_t curfl = Fram_Read(024); // ���� �� ������ �ҷ���
    for (; curfl != desfl; curfl--) { // for loop�� ������ ���� ������ ������ �ݺ���
        LEDxOn(curfl-1); // �� ĭ �������� LED On
        LEDxOff(curfl); // �� �� ���� LED Off
        DelayMS(500);
    }
}

/* ���� ������ ������ ������ '�ö� ��' ����Ǵ� LED�� ��ȣ�� ���������� ������ */
void Up(uint16_t desfl)
{
    uint16_t curfl = Fram_Read(024); // ���� �� ������ �ҷ���
    for (; curfl != desfl; curfl++) { // for loop�� ������ ���� ������ ������ �ݺ���
        LEDxOn(curfl+1); // �� ĭ �ö󰡼� LED On
        LEDxOff(curfl); // �� �� ���� LED Off
        DelayMS(500);
    }
}

/* GPIOG�� LED On */
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

/* GPIOG�� LED Off */
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

/* Switch�� �ԷµǾ������� ���ο� � switch�� �ԷµǾ������� ������ return�ϴ� �Լ� */ 
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
