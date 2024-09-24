#include "stm32f446xx.h"

/* Board name: NUCLEO-F446RE

 PA.5  <--> Green LED (LD2)
 PC.13 <--> Blue user button (B1)
 
 Base Header Code by Dr. Sajid Muhaimin Choudhury, Department of EEE, BUET 22/06/2022
 
 Music Generation 7.5 base file
 
 Based on Instructor Companion of Yifeng Zhu
*/

#define SPEAKER_PORT GPIOA
#define SPEAKER_PIN  0


#define LED_PORT GPIOA
#define LED_PIN    5

#define BUTTON_PIN 13

#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */
/*
 User HSI (high-speed internal) as the processor clock
 See Page 94 on Reference Manual to see the clock tree
 HSI Clock: 16 Mhz, 1% accuracy at 25 oC
 Max Freq of AHB: 84 MHz
 Max Freq of APB2: 84 MHZ
 Max Freq of APB1: 42 MHZ
 SysTick Clock = AHB Clock / 8
*/
	
static void LED_Pin_Init(){ //declare PA5 as tim2_ch1 
	  RCC->AHB1ENR 		|= RCC_AHB1ENR_GPIOAEN;             // Enable GPIOA clock
	
	  // Set mode as Alternative Function 1
		LED_PORT->MODER  	&= ~(0x03 << (2*LED_PIN));   			// Clear bits
		LED_PORT->MODER  	|=   0x02 << (2*LED_PIN);      		// Input(00), Output(01), AlterFunc(10), Analog(11)
	
		LED_PORT->AFR[0] 	&= ~(0xF << (4*LED_PIN));         
		LED_PORT->AFR[0] 	|=   0x1 << (4*LED_PIN);          // 	AF 1 = TIM2_CH1
	
		//Set I/O output speed value as very high speed
		LED_PORT->OSPEEDR  &= ~(0x03<<(2*LED_PIN)); 				// Speed mask
		LED_PORT->OSPEEDR  |=   0x03<<(2*LED_PIN); 					// Very high speed
		//Set I/O as no pull-up pull-down 
		LED_PORT->PUPDR    &= ~(0x03<<(2*LED_PIN));    			// No PUPD(00, reset), Pullup(01), Pulldown(10), Reserved (11)
		//Set I/O as push pull 
	  //LED_PORT->OTYPER   &=  ~(1<<LED_PIN) ;           	// Push-Pull(0, reset), Open-Drain(1)
}

static void TIM2_CH1_Init(){
		//tim uptade frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1)
	  // 4000000 / 40 / 50000 = 2Hz
		// Enable the timer clock
    RCC->APB1ENR 		|= RCC_APB1ENR_TIM2EN;                  // Enable TIMER clock

		// Counting direction: 0 = up-counting, 1 = down-counting
		TIM2->CR1 &= ~TIM_CR1_DIR;   //in header file,TIM_CR1_DIR is set to 1 by default
		
    TIM2->PSC = 39;       // Prescaler = 23 
    TIM2->ARR = 50000-1;   // Auto-reload: Upcouting (0..ARR), Downcouting (ARR..0)
		TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;  // Clear ouput compare mode bits for channel 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; // OC1M = 0011 =toggle mode selected for PA5 pin
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;                     // Output 1 preload enable

		// Select output polarity: 0 = active high, 1 = active low
		TIM2->CCMR1 |= TIM_CCER_CC1NP; // select active high (complementary pol bit =1 ie polarity bit=0= active high
		
    // Enable output for ch1
		TIM2->CCER |= TIM_CCER_CC1E;                       
    
    // Main output enable (MOE): 0 = Disable, 1 = Enable
		TIM2->BDTR |= TIM_BDTR_MOE;  

    TIM2->CCR1  = 500;         // Output Compare Register for channel 1:when counter value >= ccr value:toggling occurs
		TIM2->CR1  |= TIM_CR1_CEN; // Enable counter
}

int main(void){
	
// Default system clock 4 MHz
		
	TIM2_CH1_Init();
	LED_Pin_Init();														 
															 
	while(1);
}

