#include "stm32f446xx.h"
#include "stdio.h"

/* Board name: NUCLEO-F446RE

 PA.5  <--> Green LED (LD2)
 PC.13 <--> Blue user button (B1)
  
 Base Header Code by Shafin Bin Hamid, Department of EEE, BUET 05/08/2022

 Based on Instructor Companion of Muhammad Ali Mazidi
*/
#define LED_PIN    5
void delayUs(int n);

static void configure_pin(){
  // Enable the clock to GPIO Port A	
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   
	
	//configure PA4 as analog pin (MODER[1:0] = 11)
	GPIOA->MODER |= 0x00000300; /* PA4 analog */
	
	/* setup DAC */
	RCC->APB1ENR |= 1 << 29; /* enable DAC clock */
	DAC->CR |= 1; /* enable DAC */
	
}

void delayUs(int n)
{
int i;
for (; n > 0; n--)
for (i = 0; i < 3; i++) ;
}

int main(void){
	int i;
	const static int sineWave[] = {2048,3071,3821,4095,3821,3071,2048,1024,274,0,274,1024};
	// enable_HSI();
	configure_pin();
	
  // Dead loop & program hangs here
	while(1){
		
		for (i = 0; i < sizeof(sineWave)/sizeof(int); i++)
		{
			DAC->DHR12R1 = sineWave[i]; /* write value of sinewave to DAC */
			delayUs(10);
		}
	
	}
}
