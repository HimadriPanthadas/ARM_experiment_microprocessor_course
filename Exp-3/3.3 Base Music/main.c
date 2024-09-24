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
	
static void enable_HSI(){
	
	/* Enable Power Control clock */
	/* RCC->APB1ENR |= RCC_APB1LPENR_PWRLPEN; */
	
	// Regulator voltage scaling output selection: Scale 2 
	// PWR->CR |= PWR_CR_VOS_1;
	
	// Enable High Speed Internal Clock (HSI = 16 MHz)
	RCC->CR |= ((uint32_t)RCC_CR_HSION);
	while ((RCC->CR & RCC_CR_HSIRDY) == 0); // Wait until HSI ready
	
	// Store calibration value
	PWR->CR |= (uint32_t)(16 << 3);
	
	// Reset CFGR register 
	RCC->CFGR = 0x00000000;

 	// Reset HSEON, CSSON and PLLON bits 
 	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
	while ((RCC->CR & RCC_CR_PLLRDY) != 0); // Wait until PLL disabled
	
	// Programming PLLCFGR register 
	// RCC->PLLCFGR = 0x24003010; // This is the default value

	// Tip: 
	// Recommended to set VOC Input f(PLL clock input) / PLLM to 1-2MHz
	// Set VCO output between 192 and 432 MHz, 
	// f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
	// f(PLL general clock output) = f(VCO clock) / PLLP
	// f(USB OTG FS, SDIO, RNG clock output) = f(VCO clock) / PLLQ
 	
	RCC->PLLCFGR = 0;
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC); 		// PLLSRC = 0 (HSI 16 Mhz clock selected as clock source)
	RCC->PLLCFGR |= 16 << RCC_PLLCFGR_PLLN_Pos; 	// PLLM = 16, VCO input clock = 16 MHz / PLLM = 1 MHz
	RCC->PLLCFGR |= 336 << RCC_PLLCFGR_PLLN_Pos; 	// PLLN = 336, VCO output clock = 1 MHz * 336 = 336 MHz
	RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLP_Pos; 	// PLLP = 4, PLLCLK = 336 Mhz / PLLP = 84 MHz
	RCC->PLLCFGR |= 7 << RCC_PLLCFGR_PLLQ_Pos; 	// PLLQ = 7, USB Clock = 336 MHz / PLLQ = 48 MHz

	// Enable Main PLL Clock
	RCC->CR |= RCC_CR_PLLON; 
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);  // Wait until PLL ready
	
	
	// FLASH configuration block
	// enable instruction cache, enable prefetch, set latency to 2WS (3 CPU cycles)
	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;

	// Configure the HCLK, PCLK1 and PCLK2 clocks dividers
	// AHB clock division factor
	RCC->CFGR &= ~RCC_CFGR_HPRE; // 84 MHz, not divided
	// PPRE1: APB Low speed prescaler (APB1)
	RCC->CFGR &= ~RCC_CFGR_PPRE1; 
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // 42 MHz, divided by 2
	// PPRE2: APB high-speed prescaler (APB2)
	RCC->CFGR &= ~RCC_CFGR_PPRE2; // 84 MHz, not divided
	
	// Select PLL as system clock source 
	// 00: HSI oscillator selected as system clock
	// 01: HSE oscillator selected as system clock
	// 10: PLL selected as system clock
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_1;
	// while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

  	// Configure the Vector Table location add offset address 
//	VECT_TAB_OFFSET  = 0x00UL; // Vector Table base offset field. 
                                   // This value must be a multiple of 0x200. 
  	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; // Vector Table Relocation in Internal FLASH 

}


static void SPEAKER_Pin_Init(){
  // Enable the clock to GPIO Port A ----- configuring PA0 pin 	
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   

	// Set mode as Alternative Function 1
		// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
		SPEAKER_PORT->MODER  	&= ~(0x03 << (2*SPEAKER_PIN));   			// Clear bits
		SPEAKER_PORT->MODER  	|=   0x02 << (2*SPEAKER_PIN);      		// Input(00), Output(01), AlterFunc(10), Analog(11)
		
		SPEAKER_PORT->AFR[0] 	&= ~(0xF << (4*SPEAKER_PIN));     // 	Clear AF
	  SPEAKER_PORT->AFR[0] 	|=   0x2 << (4*SPEAKER_PIN);      // 	AF 2 = TIM5_CH1 : here,0010 is selected as Tim5_ch1 is 2 no. alternate func; see the table for more clarity
	
	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
		//Set I/O output speed value as very high speed
		SPEAKER_PORT->OSPEEDR  &= ~(0x03<<(2*SPEAKER_PIN)); 	  // Speed mask
		SPEAKER_PORT->OSPEEDR  |=   0x03<<(2*SPEAKER_PIN); 			// Very high speed
	
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
	//SPEAKER_PORT->OTYPER &= ~(1U<<SPEAKER_PIN);      // Push-pull
	
	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	SPEAKER_PORT->PUPDR  &= ~(3U<<(2*SPEAKER_PIN));  // No pull-up, no pull-down
	
}

static void TIM5_CH1_Init(){
	//function for musical frequency in timer5
	//tim uptade frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1)
	// 16000000 / 2 / 2000 = 50Hz
		// Enable the timer clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;                  // Enable TIMER clock

		// Counting direction: 0 = up-counting, 1 = down-counting
		TIM5->CR1 &= ~TIM_CR1_DIR;  
		
    TIM5->PSC = 1;       // Prescaler = 23 
    TIM5->ARR = 7999-1;   // Auto-reload: Upcouting (0..ARR), Downcouting (ARR..0)
		TIM5->CCMR1 &= ~TIM_CCMR1_OC1M;  // Clear ouput compare mode bits for channel 1
 
	  TIM5->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; // OC1M = 0011 
    TIM5->CCMR1 |= TIM_CCMR1_OC1PE;                     // Output 1 preload enable

		// Select output polarity: 0 = active high, 1 = active low
		TIM5->CCMR1 |= TIM_CCER_CC1NP; // select active high
		
    // Enable output for ch1
		TIM5->CCER |= TIM_CCER_CC1E;                       
    
    // Main output enable (MOE): 0 = Disable, 1 = Enable
		TIM5->BDTR |= TIM_BDTR_MOE;  

		//TIM5->CCR1  = 1135;        // Output Compare Register for channel 1 
		TIM5->CR1  |= TIM_CR1_CEN; // Enable counter
}


int main(void){
		int i;
	  uint16_t current_note = 0;
	
    static uint32_t note_freq[8] = {261, 	294, 329, 349, 392, 440, 494, 522}; //Hz
		static uint16_t song_notes[32] = {2, 2, 3, 2, 4, 4, 4, 4, 
															 1, 1, 2, 1, 3, 3, 3, 3, 
															 2, 2, 2, 2, 1, 1, 1, 1,
															 0, 0, 0, 0, 0, 0, 0, 0};  // 0=C, 1=D ....
		
		
// Default system clock 4 MHz
	
	enable_HSI(); //16 MHz
	SPEAKER_Pin_Init();
	
	TIM5_CH1_Init();
	TIM5->ARR = (16000000 / 4 /  note_freq[current_note] ) - 1;

	while(1){
		  	TIM5->ARR = (16000000UL/2/ note_freq[song_notes[current_note]] ) - 1UL;
				current_note = current_note+1;
				if (current_note > 32 ||  current_note < 0) current_note = 0; // means,song_notes matrix sesh hoye gele abar song restart
			  for(i=0;i<1000000;i++);  		// delay
	}
}

