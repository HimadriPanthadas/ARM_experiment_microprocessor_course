#include "stm32f446xx.h"
#include "stdio.h"

/* Board name: NUCLEO-F446RE

 PA.5  <--> Green LED (LD2)
 PC.13 <--> Blue user button (B1)
  
 Based on Instructor Companion of Yifeng Zhu
*/
#define LED_PIN    5
#define PUSH_BUTTON 13

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


static void configure_pin(){
  // Enable the clock to GPIO Port A	
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   
		
	/*
	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	
	*/
	
	// Set mode as Alternative Function 1
	GPIOA->MODER  	&= ~(0x03 << (2*LED_PIN));   			// Clear bits
	GPIOA->MODER  	|=   0x01 << (2*LED_PIN);      		// Input(00), Output(01), AlterFunc(10), Analog(11)
	
	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	GPIOA->OSPEEDR &= ~((unsigned int)3<<(2*LED_PIN));
	GPIOA->OSPEEDR |=   2<<((unsigned int)2*LED_PIN);  // Fast speed
	
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
 GPIOA->OTYPER &= ~((unsigned int)1<<LED_PIN);      // Push-pull
	//GPIOA->OTYPER |= ((unsigned int)1<<LED_PIN); 
	
	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOA->PUPDR  &= ~((unsigned int)3<<(2*LED_PIN));  // No pull-up, no pull-down
	
	//configure PA1 as analog pin (MODER[1:0] = 11)
	GPIOA->MODER |= 0xC;
	
	/*setup ADC1*/
	RCC->APB2ENR |= 0x00000100; /* enable ADC1 clock */
	
	ADC1->CR2 = 0; /* SW trigger */
	
	ADC1->SQR3 = 1; /* conversion sequence starts at ch 1 */
	
	ADC1->SQR1 = 0; /* conversion sequence length 1 */
	
	ADC1->CR2 |= 1; /* enable ADC1 */
	
}
static void turn_on_LED(){
	GPIOA->ODR |= ((unsigned int)1) << LED_PIN;
}

static void turn_off_LED(){
	GPIOA->ODR &= ~((unsigned int)1 << LED_PIN);
}

static void toggle_LED(){
	GPIOA->ODR ^= (1 << LED_PIN);
}

int main(void){
	int i;
	uint32_t result;
	enable_HSI();
	configure_pin();
	
  // Dead loop & program hangs here
	while(1)
		{
		
		ADC1->CR2 |= 0x40000000; /* start a conversion */ 
		
		while(!(ADC1->SR & 2)); /* wait for conv complete */ 
		result = ADC1->DR; /* read conversion result */
		
		if (result>2048) turn_on_LED();
		else turn_off_LED();		
		
		for(i=0; i<1000; i++);
	
	}
}
