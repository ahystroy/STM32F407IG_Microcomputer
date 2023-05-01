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
#define RE 2    // �� ���������� �� ��� ���������Ͱ� �������� �����ִ� ���

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

// �����, ��ǥ�� ���� (0���϶� ���� ������ �� �ְ���)
// ��, 0���� ���� ���� ���õ��� �ʾ����� �ǹ�
uint8_t StartFloor = 0;
uint8_t DesFloor = 0;

// ������ ������������ ���� �� ����
uint8_t LEFloor, REFloor;

// ����ġ0 Flag ���� (EXTI)
uint8_t SW0_Flag;

int main(void)
{
    _GPIO_Init(); 	// GPIO (LED,SW,Buzzer) �ʱ�ȭ
    _EXTI_Init();   // EXTI (EXTI8, EXTI15) �ʱ�ȭ
    LCD_Init();	// LCD ��� �ʱ�ȭ
    DelayMS(10);

    Fram_Init();            // FRAM �ʱ�ȭ H/W �ʱ�ȭ
    Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
    
    // �ʱ� LED ����: LED7(ON), LED0~6(OFF)
    GPIOG->ODR &= ~0xFF00;	// LED0~6 Off
    GPIOG->ODR |= 0x0080;       // LED7 On
    
    DisplayInitScreen();    // LCD �ʱ�ȭ��

    while (1) {
        /* ������ ��� (FL) */

        /* ������� ���� ���õ��� �ʾ��� �� ����� ���� ���� */
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

        /* ������� ���õǰ� ��ǥ���� ���õ��� �ʾ����� ��ǥ���� �� �������� ���� */
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

        /* ���ͷ�Ʈ EXTI8 �߻� (������) */
        if (SW0_Flag)
        {
            // �� ������ �Ϸ� �Ǿ����� ������� ��ȯ
            if (StartFloor != 0 && DesFloor != 0) {
                GPIOG->ODR |= 0x0001;   // LED0 On
                GPIOG->ODR &= ~0x0080;   // LED7 Off

                LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
                LCD_DisplayText(1, 8, "EX");    // ���� ���

                /* ����, ������ ���������� �߿��� ��� ���������Ͱ� �̵����� ���� */
                int LEoper, REoper;
                if (StartFloor - LEFloor >= 0)
                    LEoper = StartFloor - LEFloor;
                else
                    LEoper = LEFloor - StartFloor;
                if (StartFloor - REFloor >= 0)
                    REoper = StartFloor - REFloor;
                else
                    REoper = REFloor - StartFloor;
                /* <stdlib.h>�� abs()�� �̿��Ͽ� ���Ҽ��� ���� */

                if (LEoper <= REoper) {
                    // ������� ���� ���������Ϳ��� �� �����ų� �Ÿ��� ���� ���
                    // LE �̵�
                    Move(LEFloor, StartFloor, LE); // ��������� �̵�
                    Move(StartFloor, DesFloor, LE); // ��ǥ������ �̵�
                    LEFloor = DesFloor;
                    Fram_Write(1001, LEFloor); // ���� �� ���� ������������ ������ ����
                }
                else {
                    // ������� ������ ���������Ϳ��� �� ����� ���
                    // RE �̵�
                    Move(REFloor, StartFloor, RE); // ��������� �̵�
                    Move(StartFloor, DesFloor, RE); // ��ǥ������ �̵�
                    REFloor = DesFloor;
                    Fram_Write(1002, REFloor); // ���� �� ������ ������������ ������ ����
                }
                LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
                LCD_DisplayText(1, 8, "FL");    // ������ ����Ǹ� �� ���� ���� ����

                GPIOG->ODR &= ~0x007F;   // LED0~6 Off
                GPIOG->ODR |= 0x0080;   // LED7 On

                BEEP();
                DelayMS(100);
                BEEP();  // ���� 3�� �︲
                DelayMS(100);
                BEEP();

                // ������� ��ǥ���� ���� Setting�ϱ� ���� 0���� �ʱ�ȭ
                StartFloor = 0;
                DesFloor = 0;
            }
            SW0_Flag = 0; // �����忡�� ���������� �ǹ�
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

    // "EXTI �ʱ⼳���� ���� HW04���� ������ �ڵ��Դϴ�!"
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
void EXTI15_10_IRQHandler(void) // EXTI15 (SW7) �ߴ� ��� (Hold)
{
    // SW0_Flag�� 1�϶�, �� EXTI8(������) �߿��� �ߴܸ��� ��ȯ�Ǳ� ���� if�� �߰�
    if (SW0_Flag) {
        if (EXTI->PR & 0x008000) 	// EXTI15 Interrupt Pending(�߻�) ����?
        {
            EXTI->PR |= 0x8000; 	// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
            
            GPIOG->ODR &= ~0x0001;   // LED0 Off
            GPIOG->ODR |= 0x0080;   // LED7 On -> '�ߴ� ���'

            LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
            LCD_DisplayText(1, 8, "HD");    // ���� ���

            // DelayMS(500) 10ȸ �ݺ� -> 5�� �ߴ�
            for (int i = 0; i < 10; i++) {
                BEEP();     // �ߴܵ� 5�ʰ� ������ 0.5�� �������� �︲
                DelayMS(500);
            }
            GPIOG->ODR |= 0x0001;   // LED0 On -> '���� ���'
            GPIOG->ODR &= ~0x0080;   // LED7 Off

            LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
            LCD_DisplayText(1, 8, "EX");    // �ߴ� ��尡 ���� �� �ٽ� ���� ��� ���� ����
        }
    }
}

/* EXTI5~9 ���ͷ�Ʈ �ڵ鷯(ISR: Interrupt Service Routine) */
void EXTI9_5_IRQHandler(void) // EXTI8 (SW0) ���� ��� (Execute)
{
    if (EXTI->PR & 0x000100) 	// EXTI8 Interrupt Pending(�߻�) ����?
    {
        EXTI->PR |= 0x0100; 	// Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
        SW0_Flag = 1;   // ���� ��尡 ���������� �˸��� Flag
    }
}

/* GLCD �ʱ�ȭ�� ���� */
void DisplayInitScreen(void)
{
        LCD_Clear(RGB_YELLOW);		// ȭ�� Ŭ����, ����: YELLOW
        LCD_SetFont(&Gulim8);		// ��Ʈ : ���� 8
        LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� : YELLOW

        LCD_SetTextColor(RGB_BLACK);	// ���ڻ� : BLACK
        LCD_DisplayText(0,0,"MC-Elevator(AHY)");  // Title

        LCD_SetTextColor(RGB_BLUE);	// ���ڻ� : BLUE
        LCD_DisplayChar(1, 4, '6');
        LCD_DisplayChar(2, 4, '5');
        LCD_DisplayChar(3, 4, '4');
        LCD_DisplayChar(4, 4, '3');
        LCD_DisplayChar(5, 4, '2');
        LCD_DisplayChar(6, 4, '1');
        LCD_DisplayText(3, 8, "L-E");   // Left Elevator

        LCD_SetTextColor(RGB_RED);	// ���ڻ� : RED
        LCD_DisplayText(1, 8, "FL");    // �� ���� ���
        LCD_DisplayChar(4, 9, 'S');     // Stop ���� ����
        LCD_DisplayChar(5, 8, Fram_Read(1001) + 0x30); // Fram���� ���� �ҷ��ͼ� LCD�� ǥ��
        LCD_DisplayChar(5, 9, '>');     // ��FRAM 1001���� �� > FRAM 1001���� ����
        LCD_DisplayChar(5, 10, Fram_Read(1001) + 0x30); // Fram���� ���� �ҷ��ͼ� LCD�� ǥ��

        LCD_SetTextColor(RGB_GREEN);	// ���ڻ� : GREEN
        LCD_DisplayChar(1, 14, '6');
        LCD_DisplayChar(2, 14, '5');
        LCD_DisplayChar(3, 14, '4');
        LCD_DisplayChar(4, 14, '3');
        LCD_DisplayChar(5, 14, '2');
        LCD_DisplayChar(6, 14, '1');    // Right Elevator

        LCD_SetPenColor(RGB_BLUE);    // Left Elevator = BLUE bar
        LCD_SetBrushColor(RGB_BLUE);
        FillRect(Fram_Read(1001), 0); // ������ ������ ���� LCD�� ���������� bar�� ä��
        LEFloor = Fram_Read(1001); // ���� ������������ �������� FRAM���� �ҷ��� ����
        
        LCD_SetPenColor(RGB_GREEN);    // Right Elevator = GREEN bar
        LCD_SetBrushColor(RGB_GREEN);
        FillRect(Fram_Read(1002), 1); // ������ ������ ���� LCD�� ���������� bar�� ä��
        REFloor = Fram_Read(1002); // ������ ������������ �������� FRAM���� �ҷ��� ����
}

/* ���������� ����(L,R)�� �� ������ �Ű������� �޾� LCD �ʱ�ȭ�� �����ÿ� ���� */
void FillRect(uint8_t fl, uint8_t l_r) // �Ű����� => (��, ����or������)
{
    if (l_r == 0) 
        LCD_DrawFillRect(16, 79 - (13 * (fl-1)), 10, 11 + (13 * (fl-1)));
    else 
        LCD_DrawFillRect(126, 79 - (13 * (fl-1)), 10, 11 + (13 * (fl-1)));
}

/* ������� �������ִ� �Լ� */
/* �ش� �� LED On, LCD�� ����� ǥ�� */
void SetStartFL(int sf)
{
    LCD_SetTextColor(RGB_RED);
    switch (sf) {
    case SW1_PUSH: // 1��
        GPIOG->ODR |= 0x0002;   // LED1 On
        LCD_DisplayChar(5, 8, '1');  // LCD�� ����� 1�� ǥ��
        StartFloor = 1;
        break;
    case SW2_PUSH: // 2��
        GPIOG->ODR |= 0x0004;   // LED2 On
        LCD_DisplayChar(5, 8, '2');  // LCD�� ����� 2�� ǥ��
        StartFloor = 2;
        break;
    case SW3_PUSH: // 3��
        GPIOG->ODR |= 0x0008;   // LED3 On
        LCD_DisplayChar(5, 8, '3');  // LCD�� ����� 3�� ǥ��
        StartFloor = 3;
        break;
    case SW4_PUSH: // 4��
        GPIOG->ODR |= 0x0010;   // LED4 On
        LCD_DisplayChar(5, 8, '4');  // LCD�� ����� 4�� ǥ��
        StartFloor = 4;
        break;
    case SW5_PUSH: // 5��
        GPIOG->ODR |= 0x0020;   // LED5 On
        LCD_DisplayChar(5, 8, '5');  // LCD�� ����� 5�� ǥ��
        StartFloor = 5;
        break;
    case SW6_PUSH: // 6��
        GPIOG->ODR |= 0x0040;   // LED6 On
        LCD_DisplayChar(5, 8, '6');  // LCD�� ����� 6�� ǥ��
        StartFloor = 6;
        break;
    }
    BEEP();
}

/* ��ǥ���� �������ִ� �Լ� */
/* �ش� �� LED On, LCD�� ��ǥ�� ǥ�� */
void SetDesFL(int df)
{
    LCD_SetTextColor(RGB_RED);
    switch (df) {
    case SW1_PUSH: // 1��
        if (StartFloor == 1) return; // ������� ��ǥ���� �����ÿ��� return
        GPIOG->ODR |= 0x0002;   // LED1 On
        LCD_DisplayChar(5, 10, '1');  // LCD�� ����� 1�� ǥ��
        DesFloor = 1;
        break;
    case SW2_PUSH: // 2��
        if (StartFloor == 2) return; // ������� ��ǥ���� �����ÿ��� return
        GPIOG->ODR |= 0x0004;   // LED2 On
        LCD_DisplayChar(5, 10, '2');  // LCD�� ����� 2�� ǥ��
        DesFloor = 2;
        break;
    case SW3_PUSH: // 3��
        if (StartFloor == 3) return; // ������� ��ǥ���� �����ÿ��� return
        GPIOG->ODR |= 0x0008;   // LED3 On
        LCD_DisplayChar(5, 10, '3');  // LCD�� ����� 3�� ǥ��
        DesFloor = 3;
        break;
    case SW4_PUSH: // 4��
        if (StartFloor == 4) return; // ������� ��ǥ���� �����ÿ��� return
        GPIOG->ODR |= 0x0010;   // LED4 On
        LCD_DisplayChar(5, 10, '4');  // LCD�� ����� 4�� ǥ��
        DesFloor = 4;
        break;
    case SW5_PUSH: // 5��
        if (StartFloor == 5) return; // ������� ��ǥ���� �����ÿ��� return
        GPIOG->ODR |= 0x0020;   // LED5 On
        LCD_DisplayChar(5, 10, '5');  // LCD�� ����� 5�� ǥ��
        DesFloor = 5;
        break;
    case SW6_PUSH: // 6��
        if (StartFloor == 6) return; // ������� ��ǥ���� �����ÿ��� return
        GPIOG->ODR |= 0x0040;   // LED6 On
        LCD_DisplayChar(5, 10, '6');  // LCD�� ����� 6�� ǥ��
        DesFloor = 6;
        break;
    }
    BEEP();
}

/* ���������Ͱ� ��-�Ʒ��� �����ϴ� �Լ� */
/* (������, ������, ���������� ����)�� �Ű������� �޾Ƽ� ���� */
void Move(uint8_t start, uint8_t end, uint8_t e)
{
    uint8_t start_ = start;
    uint8_t end_ = end;
    if (e == LE) {
        LCD_SetTextColor(RGB_BLUE);
        LCD_DisplayText(3, 8, "L-E");   // Left Elevator
        if (start > end) { // Down
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayChar(4, 9, 'D'); // LCD ǥ��
            DelayMS(500);
            for (start_--; start_ != end_ - 1; start_--) { // for loop�� ������ ���� ������ ������ �ݺ���
                LCD_SetBrushColor(RGB_YELLOW); // ���������� �������� bar�� ����
                LCD_DrawFillRect(16, 14, 10, 13 * (6-start_)); // ���������� ������ LCD�� ǥ��
                DelayMS(500);   // 0.5�� ����
            }
        }
        else if (start < end) { // Up
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayChar(4, 9, 'U'); // LCD ǥ��
            DelayMS(500);
            for (start_++; start_ != end_ + 1; start_++) { // for loop�� ������ ���� ������ ������ �ݺ���
                LCD_SetBrushColor(RGB_BLUE); // �ö󰥶��� ���� ä��
                LCD_DrawFillRect(16, 79 - (13 * (start_-1)), 10, 11 + (13 * (start_-1))); // ���������� ������ LCD�� ǥ��
                DelayMS(500);   // 0.5�� ����
            }
        }
        else return; 
        // �������� �������� �������� return
        // Ex) ������������ ���� ���� ������� ���
    }
    else if (e == RE) {
        LCD_SetTextColor(RGB_GREEN);
        LCD_DisplayText(3, 8, "R-E");   // Right Elevator
        if (start > end) { // Down
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayChar(4, 9, 'D'); // LCD ǥ��
            DelayMS(500);
            for (start_--; start_ != end_ - 1; start_--) { // for loop�� ������ ���� ������ ������ �ݺ���
                LCD_SetBrushColor(RGB_YELLOW); // ���������� �������� bar�� ����
                LCD_DrawFillRect(126, 14, 10, 13 * (6 - start_)); // ���������� ������ LCD�� ǥ��
                DelayMS(500);   // 0.5�� ����
            }
        }
        else if (start < end) { // Up
            LCD_SetTextColor(RGB_RED);
            LCD_DisplayChar(4, 9, 'U'); // LCD ǥ��
            DelayMS(500);
            for (start_++; start_ != end_ + 1; start_++) { // for loop�� ������ ���� ������ ������ �ݺ���
                LCD_SetBrushColor(RGB_GREEN); // �ö󰥶��� ���� ä��
                LCD_DrawFillRect(126, 79 - (13 * (start_-1)), 10, 11 + (13 * (start_-1))); // ���������� ������ LCD�� ǥ��
                DelayMS(500);   // 0.5�� ����
            }
        }
        else return;
        // �������� �������� �������� return
        // Ex) ������������ ���� ���� ������� ���
    }
    LCD_SetTextColor(RGB_RED);
    LCD_DisplayChar(4, 9, 'S');    // �����̰����ϴ� ���� �����Ͽ����� Stop
    DelayMS(500);   // 0.5�� ����
}

/* Switch�� �ԷµǾ������� ���ο� � switch�� �ԷµǾ������� ������ return�ϴ� �Լ� */ 
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
