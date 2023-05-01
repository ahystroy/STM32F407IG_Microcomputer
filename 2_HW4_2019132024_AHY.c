//////////////////////////////////////////////////////////////////////////
// HW3: 가속도 측정(SPI)을 이용한 Ball game
// 제출자: 2019132024, 안호연

// SPI 통신을 이용하여 가속도센서로부터 가속도값(X,Y,Z)을 읽어서 표시하고, 
// 키트를 기울임에 따라 변하는 가속도값을 반영하여 공이 이동하도록 표시

// SPI 통신을 이용한 가속도센서 측정
//  NSS pin:  PA8 (PA4(SPI1_CS) 대신에 사용)
//  SCK pin:  PA5 (SPI1_SCK)
//  MOSI pin: PA7 (SPI1_MOSI)
//  MISO pin: PA6 (SPI1_MISO)
// SPI mode: MASTER
// 가속도센서(LIS2HH12, Slave mode) X,Y,Z 값을 250ms마다 읽은후 LCD에 표시 
//////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "ACC.h"

#define X_MIN 6
#define X_INIT 54
#define X_MAX 102

#define Y_MIN 26
#define Y_INIT 69
#define Y_MAX 112

#define BALL_SIZE 7

void DisplayTitle(void);
void _GPIO_Init(void);
void SPI1_Init(void);
void TIMER10_Init(void);
void Display_Process(int16 *pBuf);

void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

//// void SPI1_Process(UINT16 *pBuf);  // ACC.c (ACC.h) 
//// void ACC_Init(void) // ACC.c (ACC.h)
//// void LCD_Init(void); // GLCD.c (GLCD.h)

UINT8 bControl;
uint8_t Ax_sign, Ay_sign, Az_sign; // 0 = [-] // 1 = [+]
UINT16 Ax_G, Ay_G, Az_G;

int ball_x, ball_y;


int main(void)
{
        int16 buffer[3];
    
        LCD_Init();		// LCD 구동 함수
        DelayMS(10);		// LCD구동 딜레이
        DisplayTitle();		// LCD 초기화면구동 함수
    
       	_GPIO_Init();		// LED, SW 초기화
        SPI1_Init();        	// SPI1 초기화
        ACC_Init();		// 가속도센서 초기화
        TIMER10_Init();		// 가속도센서 스캔 주기 생성: 250ms
   
        // 초기 공 위치는 중앙.
        ball_x = X_INIT;
        ball_y = Y_INIT;
        LCD_DrawRectangle(ball_x,ball_y,BALL_SIZE,BALL_SIZE); // 초기 공 위치는 중앙
        
        while(1)
        {
                if(bControl)
                {
                        bControl = FALSE;     
                        SPI1_Process(&buffer[0]);	// SPI통신을 이용하여 가속도센서 측정
                        Display_Process(&buffer[0]);	// 측정값을 LCD에 표시
                        
                        LCD_SetPenColor(RGB_WHITE);     // 하얀색 직사각형으로
                        LCD_DrawRectangle(ball_x,ball_y,BALL_SIZE,BALL_SIZE); // 이전 사각형 지움
                        if (Ax_G > 0) { // 가속도 크기가 0보다 클 때만 변동
                            if (Ax_sign) { // x양수 == 왼쪽 // G값의 범위는 -20 ~ +20
                                //if (ball_x - Ax_G/2+1 < X_MIN
                                ball_x -= (Ax_G/5)*4+1;
                                ball_x = (ball_x < X_MIN) ? X_MIN : ball_x;
                            }
                            else { // x음수 == 오른쪽
                                ball_x += (Ax_G/5)*4+1;
                                ball_x = (ball_x > X_MAX) ? X_MAX : ball_x;
                            }
                        }
                        if (Ay_G > 0) { // 가속도 크기가 0보다 클 때만 변동
                            if (Ay_sign) { // y양수 == 위쪽
                                ball_y -= (Ay_G/5)*4+1;
                                ball_y = (ball_y < Y_MIN) ? Y_MIN : ball_y;
                            }
                            else { // y음수 == 아래쪽
                                ball_y += (Ay_G/5)*4+1;
                                ball_y = (ball_y > Y_MAX) ? Y_MAX : ball_y;
                            }
                        }
                        LCD_SetPenColor(RGB_RED);     // 공 색: RED
                        LCD_DrawRectangle(ball_x,ball_y,BALL_SIZE,BALL_SIZE); // 가속도 값 계산으로 ball move
                }
        }
}

///////////////////////////////////////////////////////
// Master mode, Full-duplex, 8bit frame(MSB first), 
// fPCLK/32 Baud rate, Software slave management EN
void SPI1_Init(void)
{
	/*!< Clock Enable  *********************************************************/
        RCC->APB2ENR 	|= (1<<12);	// 0x1000, SPI1 Clock EN
        RCC->AHB1ENR 	|= (1<<0);	// 0x0001, GPIOA Clock EN		
  
        /*!< SPI1 pins configuration ************************************************/
	
        /*!< SPI1 NSS pin(PA8) configuration : GPIO 핀  */
        GPIOA->MODER 	|= (1<<(2*8));	// 0x00010000, PA8 Output mode
        GPIOA->OTYPER 	&= ~(1<<8); 	// 0x0100, push-pull(reset state)
        GPIOA->OSPEEDR 	|= (3<<(2*8));	// 0x00030000, PA8 Output speed (100MHZ) 
        GPIOA->PUPDR 	&= ~(3<<(2*8));	// 0x00030000, NO Pullup Pulldown(reset state)
    
        /*!< SPI1 SCK pin(PA5) configuration : SPI1_SCK */
        GPIOA->MODER 	|= (2<<(2*5)); 	// 0x00000800, PA5 Alternate function mode
        GPIOA->OTYPER 	&= ~(1<<5); 	// 0020, PA5 Output type push-pull (reset state)
        GPIOA->OSPEEDR 	|= (3<<(2*5));	// 0x00000C00, PA5 Output speed (100MHz)
        GPIOA->PUPDR 	|= (2<<(2*5)); 	// 0x00000800, PA5 Pull-down
        GPIOA->AFR[0] 	|= (5<<(4*5));	// 0x00500000, Connect PA5 to AF5(SPI1)
    
        /*!< SPI1 MOSI pin(PA7) configuration : SPI1_MOSI */    
        GPIOA->MODER 	|= (2<<(2*7));	// 0x00008000, PA7 Alternate function mode
        GPIOA->OTYPER	&= ~(1<<7);	// 0x0080, PA7 Output type push-pull (reset state)
        GPIOA->OSPEEDR 	|= (3<<(2*7));	// 0x0000C000, PA7 Output speed (100MHz)
        GPIOA->PUPDR 	|= (2<<(2*7)); 	// 0x00008000, PA7 Pull-down
        GPIOA->AFR[0] 	|= (5<<(4*7));	// 0x50000000, Connect PA7 to AF5(SPI1)
    
        /*!< SPI1 MISO pin(PA6) configuration : SPI1_MISO */
        GPIOA->MODER 	|= (2<<(2*6));	// 0x00002000, PA6 Alternate function mode
        GPIOA->OTYPER 	&= ~(1<<6);	// 0x0040, PA6 Output type push-pull (reset state)
        GPIOA->OSPEEDR 	|= (3<<(2*6));	// 0x00003000, PA6 Output speed (100MHz)
        GPIOA->PUPDR 	|= (2<<(2*6));	// 0x00002000, PA6 Pull-down
        GPIOA->AFR[0] 	|= (5<<(4*6));	// 0x05000000, Connect PA6 to AF5(SPI1)

       // Init SPI1 Registers 
        SPI1->CR1 |= (1<<2);	// MSTR(Master selection)=1, Master mode
        SPI1->CR1 &= ~(1<<15);	// SPI_Direction_2 Lines_FullDuplex
        SPI1->CR1 &= ~(1<<11);	// SPI_DataSize_8bit
        SPI1->CR1 |= (1<<9);  	// SSM(Software slave management)=1, 
				// NSS 핀 상태가 코딩에 의해 결정
        SPI1->CR1 |= (1<<8);   	// SSI(Internal_slave_select)=1,
				// 현재 MCU가 Master이므로 NSS 상태는 'High' 
        SPI1->CR1 &= ~(1<<7);	// LSBFirst=0, MSB transmitted first    
        SPI1->CR1 |= (4<<3);	// BR(BaudRate)=0b100, fPCLK/32 (84MHz/32 = 2.625MHz)
        SPI1->CR1 |= (1<<1);	// CPOL(Clock polarity)=1, CK is 'High' when idle
        SPI1->CR1 |= (1<<0);	// CPHA(Clock phase)=1, 두 번째 edge 에서 데이터가 샘플링
 
        SPI1->CR1 |= (1<<6);	// SPE=1, SPI1 Enable 
}

void TIMER10_Init(void)	// 가속도센서 측정 주기 생성: 250ms
{
        RCC->APB2ENR 	|= (1<<17);	// TIMER10 Clock Enable
     
        TIM10->PSC 	= 16800-1;	// Prescaler 168MHz/16800 = 10KHz (0.1ms)  
        TIM10->ARR 	= 2500-1;	// Auto reload  0.1ms * 2500 = 250ms

// TIM10->CR1 Setting skip (default)
                            
// Output Compare 설정
	// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
	TIM10->CCMR1 &= ~(3<<0); // CC1S(CC1 channel) = '0b00' : Output
	TIM10->CCMR1 &= ~(1<<2); // OC1FE=0: Output Compare 1 Fast disable 
	TIM10->CCMR1 &= ~(1<<3); // OC1PE=0: Output Compare 1 preload disable(CCR1에 언제든지 새로운 값을 loading 가능) 
	TIM10->CCMR1 |= (3<<4);  // OC1M=0b011 (Output Compare 1 Mode : toggle)
				 // OC1REF toggles when CNT = CCR1
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
	TIM10->CCER &= ~(1<<5);	// CC1P=0: CC1 channel Output Polarity (OCPolarity_High : OC1 반전없이)  

	// CC1I(CC 인터럽트) 인터럽트 발생시각 또는 신호변화(토글)시기 결정: 신호의 위상(phase) 결정
	TIM10->CCR1 = 1000;	// TIM10 CCR1

	TIM10->DIER     |= (1<<1);	// CC1IE: Enable the TIM10 CC1 interrupt
        NVIC->ISER[0] 	|= (1<<25);	// Enable TIM1_UP_TIM10 global Interrupt
        TIM10->CR1 	|= (1<<0);	// Enable Timer10 Counter    
}

void TIM1_UP_TIM10_IRQHandler(void)	// 250ms int
{
        TIM10->SR &= ~(1<<1);		// Interrupt flag Clear
	bControl = TRUE;		// 250ms마다 센서 측정 및 ball move
}

void Display_Process(int16 *pBuf)
{
//! X 축 가속도 표시
        Ax_sign = (pBuf[0] < 0) ? 0 : 1; // 부호 확인
        if (Ax_sign) { // 양수
          Ax_G = pBuf[0];
	  LCD_DisplayChar(4,22,'+'); // 양수 부호 표시
        }
        else { // 음수
          Ax_G = abs(pBuf[0]); // 절댓값
          LCD_DisplayChar(4,22,'-'); // 음수 부호 표시
        }
    	Ax_G = 20 * Ax_G / 0x4009; // 가속도 --> g 변환
	LCD_DisplayChar(4,23, Ax_G/10 +0x30); // -2.0g ~ +2.0g, 소수 첫번째자리까지 표시
	LCD_DisplayChar(4,24,'.');
	LCD_DisplayChar(4,25, Ax_G%10 +0x30);

//! Y 축 가속도 표시
        Ay_sign = (pBuf[1] < 0) ? 0 : 1; // 부호 확인
        if (Ay_sign) { // 양수
          Ay_G = pBuf[1];
	  LCD_DisplayChar(6,22,'+'); // 양수 부호 표시
        }
        else { // 음수
          Ay_G = abs(pBuf[1]); // 절댓값
          LCD_DisplayChar(6,22,'-'); // 음수 부호 표시
        }
    	Ay_G = 20 * Ay_G / 0x4009; // 가속도 --> g 변환
	LCD_DisplayChar(6,23, Ay_G/10 +0x30); // -2.0g ~ +2.0g, 소수 첫번째자리까지 표시
	LCD_DisplayChar(6,24,'.');
	LCD_DisplayChar(6,25, Ay_G%10 +0x30);
                
//! Z 축 가속도 표시
        Az_sign = (pBuf[2] < 0) ? 0 : 1; // 부호 확인
        if (Az_sign) { // 양수
          Az_G = pBuf[2];
	  LCD_DisplayChar(8,22,'+'); // 양수 부호 표시
        }
        else { // 음수
          Az_G = abs(pBuf[2]); // 절댓값
          LCD_DisplayChar(8,22,'-'); // 음수 부호 표시
        }
    	Az_G = 100 * Az_G / 0x4009; // 가속도 --> g 변환
	LCD_DisplayChar(8,23, Az_G/100 +0x30); // -2.0g ~ +2.0g, 소수 첫번째자리까지 표시
	LCD_DisplayChar(8,24,'.');
	LCD_DisplayChar(8,25, Az_G%100/10 +0x30);
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

        GPIOG->ODR &= 0x00;	// LED0~7 Off 
}

void DelayMS(unsigned short wMS)
{
        register unsigned short i;

        for (i=0; i<wMS; i++)
                DelayUS(1000);		//1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
        volatile int Dly = (int)wUS*17;
         for(; Dly; Dly--);
}

void DisplayTitle(void)
{
        LCD_Clear(RGB_WHITE);
        LCD_SetFont(&Gulim7);		// 폰트 
        LCD_SetBackColor(RGB_WHITE);
        LCD_SetTextColor(RGB_BLACK);    // 글자색
        LCD_DisplayText(1,1,"Ball game: AHY 2019132024");  // Title
        
        LCD_SetPenColor(RGB_BLUE);      // 경기장 색: BLUE
        LCD_DrawRectangle(5,25,105,95); // 경기장 그리기

	LCD_DisplayText(4,19,"Ax:");	// X AXIS
	LCD_DisplayText(6,19,"Ay:");	// Y AXIS
	LCD_DisplayText(8,19,"Az:");	// Z AXIS
        
        LCD_SetTextColor(RGB_RED);    // 글자색
}
