#include "stm32f446xx.h"

/* Board name: NUCLEO-F446RE

 PA.5  <--> Green LED (LD2)
 PC.13 <--> Blue user button (B1)
 
 Base Header Code by Dr. Sajid Muhaimin Choudhury, Department of EEE, BUET 22/06/2022
 
 Based on Instructor Companion of Yifeng Zhu
*/
#define LED_PIN 5

#define EXTI_PIN 13

#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */
																	 
																	 

////////////////// ENABLE 16MHz CLOCK BY SADMAN SAKIB AHBAB//////////////////////////

static void sys_clk_config(){
	RCC->CR |= RCC_CR_HSION;
	while ((RCC->CR & RCC_CR_HSIRDY) == 0); // Wait until HSI ready
	
	// Store calibration value
	//PWR->CR |= (uint32_t)(16 << 3);
	
	// Reset CFGR register 
	RCC->CFGR = 0x00000000;
	
	// FLASH configuration block
	// enable instruction cache, enable prefetch, set latency to 2WS (3 CPU cycles)
	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;
	
	
	// Select HSI as system clock source 
	// 00: HSI oscillator selected as system clock
	// 01: HSE oscillator selected as system clock
	// 10: PLL_P selected as system clock
	// 10: PLL_R selected as system clock
	RCC->CFGR &= ~RCC_CFGR_SW;
	
	// Configure the HCLK, PCLK1 and PCLK2 clocks dividers
	// AHB clock division factor
	RCC->CFGR &= ~RCC_CFGR_HPRE; // 16 MHz, not divided
	// PPRE1: APB Low speed prescaler (APB1)
	RCC->CFGR &= ~RCC_CFGR_PPRE1; // 16 MHz, not divided
	// PPRE2: APB high-speed prescaler (APB2)
	RCC->CFGR &= ~RCC_CFGR_PPRE2; // 16 MHz, not divided
	
	// Configure the Vector Table location add offset address 
	// VECT_TAB_OFFSET  = 0x00UL; // Vector Table base offset field. 
                                   // This value must be a multiple of 0x200. 
  	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; // Vector Table Relocation in Internal FLASH 
}
///////////////////////////////////////////////////////////////////////////////////////////


/*
 User HSI (high-speed internal) as the processor clock
 See Page 94 on Reference Manual to see the clock tree
 HSI Clock: 16 Mhz, 1% accuracy at 25 oC
 Max Freq of AHB: 84 MHz
 Max Freq of APB2: 84 MHZ
 Max Freq of APB1: 42 MHZ
 SysTick Clock = AHB Clock / 8
*/




static void configure_LED_pin(){
  // Enable the clock to GPIO Port A	
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   
		
	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOA->MODER &= ~(3UL<<(2*LED_PIN));  
	GPIOA->MODER |=   1UL<<(2*LED_PIN);      // Output(01)
	
	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	GPIOA->OSPEEDR &= ~(3U<<(2*LED_PIN));
	GPIOA->OSPEEDR |=   2U<<(2*LED_PIN);  // Fast speed
	
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
	GPIOA->OTYPER &= ~(1U<<LED_PIN);      // Push-pull
	
	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOA->PUPDR  &= ~(3U<<(2*LED_PIN));  // No pull-up, no pull-down
	
}

static void turn_on_LED(){
	GPIOA->ODR |= 1U << LED_PIN;
}

static void turn_off_LED(){
	GPIOA->ODR &= ~(1U << LED_PIN);
}

static void toggle_LED(){
	GPIOA->ODR ^= (1 << LED_PIN);
}



void config_EXTI(void) { //PC13 is the ext intrrupt
		// GPIO Configuration
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	
	// GPIO Mode: Input(00, reset), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOC->MODER &= ~(3UL<<(2*EXTI_PIN)); //input
	
	// GPIO PUDD: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)	
	GPIOC->PUPDR &= ~(3UL<<(2*EXTI_PIN)); // no pull-up, no pull down
	
	// Connect External Line to the GPIO
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;     // SYSCFG external interrupt configuration registers
	SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC; // port C
	
	// Calling trigger selection register (RTSR)
	EXTI->RTSR |= EXTI_RTSR_TR13;  // 0 = disabled, 1 = enabled
	
	// Interrupt Mask Register (IMR)
	EXTI->IMR |= EXTI_IMR_IM13;     // 0 = marked, 1 = not masked (i.e., enabled)
	
	// EXIT Interrupt Enable
	NVIC_EnableIRQ(EXTI15_10_IRQn); 
  NVIC_SetPriority(EXTI15_10_IRQn, 0); //HIGHEST PRIORITY
}

void EXTI15_10_IRQHandler(void) {  // inside handler,specify what you wanna do when interrupt occurs
//	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	uint32_t j;
	// PR: Pending register
	if (EXTI->PR & EXTI_PR_PR13) {
		// cleared by writing a 1 to this bit
		EXTI->PR |= EXTI_PR_PR13;
		toggle_LED();
		//printf("Hi\r\n");
		for(j=0;j<100;j++);
	}
}

int main(void){
	
	sys_clk_config(); // clk = 16MHz
	configure_LED_pin();
	turn_off_LED();	
	config_EXTI();
	while(1);

}

