//////////////////////////////////////////////////////////////////////////
// PR2: �ݵ�ü ��������� (Semiconductor Processing Controller)
// ������: 2019132024 ��ȣ��
//
// �ֿ� ����
// - ���� ����: �� resource���� �Է°� ���� (ADC2_IN1, PA1)
//           ADC �۵��� SWSTART�� ����Ͽ� ���ͷ�Ʈ�� ó��
//
// - Switch4~7 (PH12 ~ PH15) 
//    # ����ġ ���
//    > SW4: �����Է¸�忡�� �Է°� ���� �� ����
//    > SW5: ��� ��ȯ (�Է� ��� 'I' <-> ���� ��� 'R')
//    > SW6: �� ��忡�� ���� ���μ��� ���� ����ġ
//    > SW7: �Է¸�忡�� �����Է¸�尣�� ���� (�µ�->�з�->O2->Plasma->�µ�->...)
//
// - LED0~7: ���� �������� ��� �� ���μ��� ǥ�� (PG0 ~ PG7) 
//
// - TIMER2: Plasma PWM (TIM2_CH3, PB10) 
// - TIMER3: 0.1s timer (UI Interrupt Generate) -> �� ��ġ���� �۵� �ð� count
//
// - FRAM: �� �Է°� ���� (SPI2)
//
// - GLCD: ���� �����Ȳ ��� (FSMC)
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

uint8_t getValueDivide_7(uint8_t x);    // ADC�� ��ü ������ 7����Ͽ� �Է°� ������ ���� ���� ������ �Լ�
uint8_t getValueDivide_10(void);        // ADC�� ��ü ������ 10����Ͽ� DR���� ������ �Լ�

void _ADC_Init(void);
void TIMER3_Init(void);      // 0.1s count
void TIMER2_PWM_Init(void);  // Plasma PWM

void LED(uint8_t x, uint8_t state);  // LED'x'�� 'state'�� ���� (SET=1 or RESET=0)

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

uint8_t fram_save;            // SW4: ���簪 FRAM�� ����
uint8_t mode;                 // SW5: ��� ���� (�Է� <-> ����)
uint8_t input_key;            // SW6: ���� �Է� ���
uint8_t start_key;            // SW6: ���� ���� ���
uint8_t input_x;              // SW7: �Է��� �������� ����

uint8_t TE, PR, O2, DR;             // resource value
uint8_t TE_s, PR_s, O2_s, DR_s;     // set time

// �����忡�� flag ���� (���ͷ�Ʈ ��ƾ���� flag ������ ���ϰ� �Ͽ� ������ while ������ Ż��)
uint8_t Heater_on, AirPump_on, O2_on, Plasma_on;
uint8_t count; // TIMER3 0.1s count ����

uint16_t ADC_Value;
uint16_t ADC_Value_7  = 4095/ 7;
double   ADC_Value_10 = 4095/10;

int main(void)
{
    // Init Routine
    LCD_Init();	        // LCD Init
    DelayMS(10);	// LCD ���� ������
    LCD_Display_Init();	// LCD �ʱ�ȭ�鱸�� �Լ�
    _GPIO_Init();
    _EXTI_Init();
    Fram_Init();            // FRAM H/W �ʱ�ȭ
    Fram_Status_Config();   // FRAM S/W �ʱ�ȭ
    _ADC_Init();
    TIMER3_Init();
    TIMER2_PWM_Init();
    
    // �ʱ⿡�� �Է¸�� ����
    LED(0, SET);
    LED(1, RESET);
    
    // �ʱ⿡ FRAM ���� �о� �ش� ���� ȭ�鿡 ǥ��
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
        if (start_key == 1) { // S8: �������
            
        //! (1) Wafer IN
            LCD_DisplayChar(4,16,'W'); // 'E' -> 'W'
            LED(3, SET);
            DelayMS(500); // ���� delay 0.5s
            
            
        //! (2) Heater On
            Heater_on = 1;
            LED(4, SET);
            LCD_DisplayChar(5,3,'*'); // 'TE'�տ� '*'ǥ��
            TE_s = (uint8_t)Fram_Read(1203); // ON �ð� read
            
            TIM3->CR1 |= (1<<0); // 0.1s timer on
            // ON �ð��� �� ������ 0.1�� �ֱ��� UI ���ͷ�Ʈ�� �̿��Ͽ� ȭ�鿡 0.1�ʾ� �����ϵ��� ǥ��
            while (Heater_on) {
                LCD_DisplayChar(5,12, count/10 + 0x30);
                LCD_DisplayChar(5,14, count%10 + 0x30);
            }
            // ���ͷ�Ʈ �ڵ鷯�� ���ۿ� ���� ���ѷ��� ���������鼭 LED4 OFF, '*' ����
            TIM3->CR1 &= ~(1<<0); count = 0; // timer off
            LED(4, RESET);
            LCD_DisplayChar(5,3,' ');
            
            
        //! (3) Air Pump ON
            AirPump_on = 1;
            LED(5, SET);
            LCD_DisplayChar(6,3,'*'); // 'PR'�տ� '*'ǥ��
            PR_s = (uint8_t)Fram_Read(1204); // ON �ð� read
            
            TIM3->CR1 |= (1<<0); // 0.1s timer on
            // ON �ð��� �� ������ 0.1�� �ֱ��� UI ���ͷ�Ʈ�� �̿��Ͽ� ȭ�鿡 0.1�ʾ� �����ϵ��� ǥ��
            while (AirPump_on) {
                LCD_DisplayChar(6,12, count/10 + 0x30);
                LCD_DisplayChar(6,14, count%10 + 0x30);
            }
            // ���ͷ�Ʈ �ڵ鷯�� ���ۿ� ���� ���ѷ��� ���������鼭 LED5 OFF, '*' ����
            TIM3->CR1 &= ~(1<<0); count = 0; // timer off
            LED(5, RESET);
            LCD_DisplayChar(6,3,' ');
            
            
        //! (4) O2 ON
        //! (5) Plasma ON
            O2_on = 1; Plasma_on = 1; // O2 On ���۰� ���ÿ� Plasma�� ���ÿ� ON
            LED(6, SET); LED(7, SET);
            LCD_DisplayChar(7,3,'*'); // 'O2'�տ� '*'ǥ��
            LCD_DisplayChar(8,3,'*'); // 'DR'�տ� '*'ǥ��
            
            O2_s = (uint8_t)Fram_Read(1205); // O2 ON �ð� read
            DR_s = (uint8_t)Fram_Read(1206); // DR ON �ð� read
            TIM2->CCR3 = DR_s*2000;
            
            TIM3->CR1 |= (1<<0); // 0.1s timer on
            TIM2->CR1 |= (1<<0); // PWM �߻� (PWM ���� �ð��� ������ O2��)
            // ON �ð��� �� ������ 0.1�� �ֱ��� UI ���ͷ�Ʈ�� �̿��Ͽ� ȭ�鿡 0.1�ʾ� �����ϵ��� ǥ��
            while (O2_on) {
                LCD_DisplayChar(7,12, count/10 + 0x30);
                LCD_DisplayChar(7,14, count%10 + 0x30);
            }
            // ���ͷ�Ʈ �ڵ鷯�� ���ۿ� ���� ���ѷ��� ���������鼭
            TIM3->CR1 &= ~(1<<0); count = 0; // timer off
            TIM2->CCR3 = 0; // DR = 0���� �Ͽ� PWM ��ȣ �߻� ����
            TIM2->CR1 &= ~(1<<0); // PWM �߻� off
            // LED6,7 OFF, '*' ����
            LED(6, RESET); LED(7, RESET);
            LCD_DisplayChar(7,3,' ');
            LCD_DisplayChar(8,3,' ');
            

        //! (6) Wafer Out
            DelayMS(500); // ���� delay 0.5s
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
        
        if (input_x == 1) { // �µ��� �Է�
          TE = getValueDivide_7(input_x);       // �������� ���� ���� �µ��� ȯ���Ͽ� ����
          LCD_DisplayChar(5,7, TE/10+0x30);     // LCD ǥ��
          LCD_DisplayChar(5,9, TE%10+0x30);
        }
        else if (input_x == 2) { // �з°� �Է�
          PR = getValueDivide_7(input_x);       // �������� ���� ���� �з°� ȯ���Ͽ� ����
          PR = (40-PR);  // Air Pump �����ð��� �з¿� '�ݺ��'
          LCD_DisplayChar(6,7, PR/10+0x30);     // LCD ǥ��
          LCD_DisplayChar(6,9, PR%10+0x30);
        }
        else if (input_x == 3) { // O2�� �Է�
          O2 = getValueDivide_7(input_x);       // �������� ���� ���� O2�� ȯ���Ͽ� ����
          LCD_DisplayChar(7,7, O2/10+0x30);     // LCD ǥ��
          LCD_DisplayChar(7,9, O2%10+0x30);
        }
        else if (input_x == 4) { // Plasma ���� �Է�
          DR = getValueDivide_10();
          LCD_DisplayChar(8,7, DR+0x30);
        }
        
 	//Starts conversion of regular channels
	ADC1->CR2 |= ADC_CR2_SWSTART; 	// 0x40000000 (1<<30) 
}

//! ADC(12��Ʈ, 0~4095)�� ��ü������ 7����Ͽ� �Է°��� ������ �°Բ� �� �������� ��ȯ
uint8_t getValueDivide_7(uint8_t x)
{
  if      (ADC_Value <= ADC_Value_7)                                return 0  + (3-x)*5;  // �Է��ϴ� �� ����(x)�� ���� ��ȯ�� ����
  else if (ADC_Value > ADC_Value_7*1 && ADC_Value <= ADC_Value_7*2) return 5  + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*2 && ADC_Value <= ADC_Value_7*3) return 10 + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*3 && ADC_Value <= ADC_Value_7*4) return 15 + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*4 && ADC_Value <= ADC_Value_7*5) return 20 + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*5 && ADC_Value <= ADC_Value_7*6) return 25 + (3-x)*5;
  else if (ADC_Value > ADC_Value_7*6 && ADC_Value <= ADC_Value_7*7) return 30 + (3-x)*5;
}

//! ADC(12��Ʈ, 0~4095)�� ��ü������ 10����Ͽ� �ּұ����� DR:0, �ִ뱸���� DR:9�� �ǵ��� ��ȯ
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
{	// ADC1: PA1(pin 41) ��������
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// (1<<0) ENABLE GPIOA CLK (stm32f4xx.h ����)
	GPIOA->MODER |= (3<<2*1);		// CONFIG GPIOA PIN1(PA1) TO ANALOG IN MODE
        
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	// (1<<8) ENABLE ADC1 CLK (stm32f4xx.h ����)

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
        
        // if������ ���� ON �Ǿ��ִ� ��ġ�� �� (��ġ�� �� ������ �ϳ��� ON �ǵ��� �����Ͽ���)
        if (Heater_on) {
            if (count == TE_s) { // ������ ON �ð��� �Ǹ�
                Heater_on = 0; // ���ͷ�Ʈ ��ƾ���� Ư�� �÷��� ������ ���ϰ� �� --> ���ѷ����� ���������� ��.
            }
            else count++; // ON �ð��� ���� �ȉ����� count ��� ����. (0.1s �ֱ�)
        }
        else if (AirPump_on) {
            if (count == PR_s) { // ������ ON �ð��� �Ǹ�
                AirPump_on = 0; // ���ͷ�Ʈ ��ƾ���� Ư�� �÷��� ������ ���ϰ� �� --> ���ѷ����� ���������� ��.
            }
            else count++; // ON �ð��� ���� �ȉ����� count ��� ����. (0.1s �ֱ�)
        }
        else if (O2_on) {
            if (count == O2_s) { // ������ ON �ð��� �Ǹ�
                O2_on = 0; // ���ͷ�Ʈ ��ƾ���� Ư�� �÷��� ������ ���ϰ� �� --> ���ѷ����� ���������� ��.
            }
            else count++; // ON �ð��� ���� �ȉ����� count ��� ����. (0.1s �ֱ�)
        }
        else if (Plasma_on) {
            if (count == DR_s) { // ������ ON �ð��� �Ǹ�
                Plasma_on = 0; // ���ͷ�Ʈ ��ƾ���� Ư�� �÷��� ������ ���ϰ� �� --> ���ѷ����� ���������� ��.
            }
            else count++; // ON �ð��� ���� �ȉ����� count ��� ����. (0.1s �ֱ�)
        }
}

/* 0.1s UI �߻� */
void TIMER3_Init(void)
{
	RCC->APB1ENR |= 0x02;	// RCC_APB1ENR TIMER3 Enable

	// CR1 setting ���� : 0x0000 (Reset State)

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

/* Plasma PWM ��ȣ �߻� */ 
void TIMER2_PWM_Init(void)
{   
// TIM2_CH3 : PB10 (79�� ��)
// Clock Enable : GPIOB & TIMER2
	RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
	RCC->APB1ENR 	|= (1<<0);	// TIMER2 CLOCK Enable 
    						
// PB10�� ��¼����ϰ� Alternate function(TIM2_CH3)���� ��� ���� : PWM ���
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
					// �ش���(79��)�� ���� ��ȣ���
	TIM2->CCER &= ~(1<<9);	// CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3���� �������� ���)

// Duty Ratio 
	TIM2->CCR3 = 0;	// CCR3 value

// 'Mode' Selection : Output mode, PWM 1
// CCMR2(Capture/Compare Mode Register 2) : Setting the MODE of Ch3 or Ch4
	TIM2->CCMR2 &= ~(3<<0); // CC3S(CC3 channel)= '0b00' : Output 
	TIM2->CCMR2 |= (1<<3); 	// OC3PE=1: Output Compare 3 preload Enable
	TIM2->CCMR2 |= (6<<4);	// OC3M=0b110: Output compare 3 mode: PWM 1 mode
	TIM2->CCMR2 |= (1<<7);	// OC3CE=1: Output compare 3 Clear enable
	
// Plasma On �ÿ� PWM �߻��ϱ� ���� TIM2 disable.
	TIM2->CR1 &= ~(1<<0);	// CEN: Counter TIM2 disable
}


void _GPIO_Init(void)
{
	// LED (GPIO G) ����
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) ���� 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state

	// Buzzer (GPIO F) ���� 
    	RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
 	GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed
}

/* SW4 ~ SW7's Interrupt Handler Routine */
void EXTI15_10_IRQHandler(void)
{
  // EXTI12 ~ EXTI15 Interrupt Pending(�߻�) ���� üũ
  
     //! SW4: �����Է¸�忡�� �Է°� ���� �� FRAM�� ����
	if (EXTI->PR & 0x1000)
	{
            EXTI->PR |= 0x1000; 		// Pending bit Clear
            
            ADC1->CR2 |= ADC_CR2_SWSTART;
            if (input_key) {      // FRAM�� ����
              if      (input_x == 1) Fram_Write(1203, TE);
              else if (input_x == 2) Fram_Write(1204, PR);
              else if (input_x == 3) Fram_Write(1205, O2);
              else if (input_x == 4) Fram_Write(1206, DR);
            }
	}
        
     //! SW5: ��� ��ȯ (�Է� <-> ����)
        else if (EXTI->PR & 0x2000)
	{
            EXTI->PR |= 0x2000; 		// Pending bit Clear

            if (mode) {
                if (!start_key) { //! �������� �ƴ� ���� �Է¸��� ����
                    LED(0, SET);
                    LED(1, RESET); 
                    LCD_DisplayChar(4,8,'I');
                    mode = 0; // �Է¸��
                }
            }
            else {
                if (!input_key) { //! ���� �Է¸�带 �������������� ��� ��ȯ
                    LED(0, RESET);
                    LED(1, SET);
                    LCD_DisplayChar(4,8,'R');
                    mode = 1; // ������
                }
            }
	}
        
     //! SW6: �� ��忡�� ���� ���μ��� ���� (���θ�� �Է¸�� ����)
        else if (EXTI->PR & 0x4000)
	{
            EXTI->PR |= 0x4000; 		// Pending bit Clear
            
            if (!mode) { //! �Է¸�忡�� "SW6 = �Է� ���� ���"
                if (input_key) {
                    ADC1->CR2 &= ~(1<<0); // ADC Off
                    LED(2, RESET); 
                    // LED4~7 Off (������ ���õǾ����� ������ �帧�� ���� ������� �����ϴ°� �´� �� ���� �̷��� �����Ͽ����ϴ�.)
                    GPIOG->ODR &= ~(0xF<<4);
                    LCD_DisplayChar(5,3,' '); // LCD '*'�� ����
                    LCD_DisplayChar(6,3,' ');
                    LCD_DisplayChar(7,3,' ');
                    LCD_DisplayChar(8,3,' ');
                    input_key = 0; // �Է� ����
                }
                else {
                    ADC1->CR2 |= (1<<0); // ADC On
                    ADC1->CR2 |= ADC_CR2_SWSTART;
                    LED(2, SET);
                    if (input_x != 0) {
                      LED(input_x+3, SET); // ���� �Է��� ���� LED On
                      LCD_DisplayChar(4+input_x,3,'*'); // ���� �Է��� �� �տ� '*' ǥ��
                    }
                    input_key = 1; // �Է� ����
                }
            }
            // mode = 0 : �Է¸��
            // mode = 1 : ������
            else { //! �����忡�� "SW6 = ���� ���� ���"
                if (start_key == 0) {
                    LED(2, SET);
                    start_key = 1; // ���� ����
                }
                // start_key = 0 : ������ �������� ���� ����
                // start_key = 1 : ���� ���� ���� --> ���� �߿��� �ߴ� �Ұ��ϵ��� ����
                // start_key = 2 : ������ ��ģ ����
                else if (start_key == 2) {
                    LED(2, RESET); // ������ ����ġ�� �ߴ��� Ȯ���� ��.
                    start_key = 0;
                }
            }
	}
        
     //! SW7: �Է¸�忡�� �����Է¸�尣�� ���� (�µ� -> �з� -> O2 -> Plasma -> �µ� ...)
        else if (EXTI->PR & 0x8000)
	{
            EXTI->PR |= 0x8000; 		// Pending bit Clear
            if (input_key) {
                ADC1->CR2 |= ADC_CR2_SWSTART;
                
                input_x++; // ���� �Է�
                
                // LED ��� Off �� ��, �ش� LED On
                GPIOG->ODR &= ~(0xF<<4);
                if (input_x == 5) input_x = 1;
                LED(input_x+3, SET);
                
                // �ش� �Է°��� * ǥ��
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
        SYSCFG->EXTICR[3] |= 0x7777;    // EXTI12~15�� ���� �ҽ� �Է��� GPIOH�� ����
        EXTI->FTSR |= (0xF<<12);        // EXTI12~15: Falling Trigger Enable
        EXTI->IMR  |= (0xF<<12);        // EXTI12~15: Interrupt Enable
        NVIC->ISER[1] |= (1<<(40-32));  // Enable 'Global Interrupt EXTI15_10'
}


/*!  LED'x'�� 'state'�� ����  !*/
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
	LCD_Clear(RGB_YELLOW);          // ����   : YELLOW 
        
        LCD_SetPenColor(RGB_BLUE);      // �׵θ� �� : BLUE
        LCD_DrawRectangle(14,7,134,115);
        
	LCD_SetFont(&Gulim8);		// ��Ʈ 
	LCD_SetBackColor(RGB_YELLOW);	// ���ڹ��� : YELLOW
        
	LCD_SetTextColor(RGB_BLUE);	// ���� ���ڻ�: BLUE
       	LCD_DisplayText(1,3,"AHY 2019132024");
      	LCD_DisplayText(2,3,"SPC monitor");
        
        LCD_SetTextColor(RGB_BLACK);	// ���� ���ڻ�: BLACK
        LCD_DisplayText(4,3,"Mode:  Wafer:");
        LCD_DisplayText(5,4,"TE:   s    s");
        LCD_DisplayText(6,4,"PR:   s    s");
        LCD_DisplayText(7,4,"O2:   s    s");
        LCD_DisplayText(8,4,"DR:");
        
        LCD_SetTextColor(RGB_RED);      // ���� ���ڻ�: RED
        LCD_DisplayChar(4,8,'I');       // �ʱ�  Mode = 'I'
        LCD_DisplayChar(4,16,'E');      // �ʱ� Wafer = 'E'
        LCD_DisplayChar(5,3,'*');
        
        LCD_DisplayText(5,7,"1.0");
        LCD_DisplayText(6,7,"0.5");
        LCD_DisplayText(7,7,"0.0");
        LCD_DisplayText(8,7,"1");
        
        LCD_DisplayText(5,12,"0.0");
        LCD_DisplayText(6,12,"0.0");
        LCD_DisplayText(7,12,"0.0");
}