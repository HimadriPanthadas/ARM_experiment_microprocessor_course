#include "stm32f446xx.h"
#include "core_cm4.h"

/* Board name: NUCLEO-F446RE

 PA.5  <--> Green LED (LD2)
 PC.13 <--> Blue user button (B1)
 
 Base Header Code by Dr. Sajid Muhaimin Choudhury, Department of EEE, BUET 22/06/2022
 
 Based on Instructor Companion of Yifeng Zhu
*/
#define LED_PIN    5

#define BUTTON_PIN 13

volatile uint32_t TimeDelay=0;

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
	
	
	sys_clk_config(); // clk = 16MHz
	configure_LED_pin();
	configure_SysTick(2000); // ARR of SysTick = 2K. 
	// systick clock is chosen as system clock/8 = 16M/8 = 2M in the
	// [SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;] line.
	// so interrupt freq = 2M/2K = 1KHz, 
	
	//so every 1ms an interrupt is generated
	
	
	// systick handler is reducing global volatile variable TimeDelay
	// in the Delay funtion, we set the TimeDealy var to rquired ms,
	// then wait in a while loop for systick handler to reduce it to zero.
	// check with stop watch

	
	
	while(1){
		toggle_LED();
		MYDelay(5000); // (in ms unit) LED kotokkhon por on hobe off hobe sheta MyDelay diye pura controlled hocche
	}

}

