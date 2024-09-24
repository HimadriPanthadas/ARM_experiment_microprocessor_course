#include "stm32f446xx.h"
#include "stdio.h"

/* Board name: NUCLEO-F446RE

 PA.5  <--> Green LED (LD2)
 PC.13 <--> Blue user button (B1)
  
 Based on Instructor Companion of Yifeng Zhu
*/
#define EXTI_PIN 13

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

volatile uint32_t TimeDelay=0;
int check = 0;

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
	int flag = 0;
	// PR: Pending register
	if (EXTI->PR & EXTI_PR_PR13) {
		// cleared by writing a 1 to this bit
		EXTI->PR |= EXTI_PR_PR13;
		if(flag==0){
			check = 1;
			flag = 1;
		}
		else{
			check = 0;
			flag = 0;
		}
		for(j=0;j<100;j++);
	}
}

static void configure_SysTick(uint32_t ticks){
		SysTick->CTRL = 0;            // Disable SysTick
		
    SysTick->LOAD = ticks - 1;    // Set reload register. 

    // Set interrupt priority of SysTick to least urgency (i.e., largest priority value)
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);

    SysTick->VAL = 0;             // Reset the SysTick counter value

    // Select processor clock/8 : 1 = processor clock; 0 = external clock = processor clock/8
		SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
	
		// Enables SysTick exception request
		// 1 = counting down to zero asserts the SysTick exception request
		// 0 = counting down to zero does not assert the SysTick exception request
		SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
		
		// Enable SysTick
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	
}
void SysTick_Handler (void) { // SysTick interrupt service routine = systick interrupt hoile ki korbo ta bole dewa
	if (TimeDelay>0)
		TimeDelay--; //1ms e TimeDelay kombe 1 unit hence TimeDelay 1000 unit komte =1ms*1000=1 sec timelagbe
} 	

void MYDelay (uint32_t nTime) {
  // nTime: specifies the delay time length
  TimeDelay = nTime;      // TimeDelay must be declared as volatile
  while(TimeDelay != 0);  // Busy wait
}
int main(void){
	int i;
	uint32_t result;
	enable_HSI();
	config_EXTI();
	configure_SysTick(2000);
	while(1)
	{
		
		while(check==1){
			ADC1->CR2 |= ADC_CR2_SWSTART; /* start a conversion */ 	
			while(!(ADC1->SR & 2)); /* wait for conv complete */ 
			result = ADC1->DR; /* read conversion result */	
			MYDelay(10);
		}			
		if(~check){
			ADC1->CR2 &= ~ADC_CR2_SWSTART;
		}
		for(i=0; i<1000; i++);
	
	}
}
