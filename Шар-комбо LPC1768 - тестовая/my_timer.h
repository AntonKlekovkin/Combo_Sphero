#include <lpc17xx.h>

#define rit_h_
#define PRESCALE (SystemCoreClock/2000000-1) //25000 PCLK clock cycles to increment TC by 1 

void initTimer0();

void initTimer0(void)
{
	/*Assuming that PLL0 has been setup with CCLK = 100Mhz and PCLK = 25Mhz.*/
	LPC_SC->PCONP |= (1<<1); //Power up TIM0. By default TIM0 and TIM1 are enabled.
	LPC_SC->PCLKSEL0 |= (1<<3); //Set PCLK for timer = CCLK/2 = 100/2 (default)
	
	LPC_TIM0->CTCR = 0x0;
	LPC_TIM0->PR = PRESCALE; //Increment LPC_TIM0->TC at every 24999+1 clock cycles
	//25000 clock cycles @25Mhz = 1 mS
	
	LPC_TIM0->MR0 = 100; //Toggle Time in mS
	LPC_TIM0->MCR |= (1<<0) | (1<<1); // Interrupt & Reset on MR0 match
	LPC_TIM0->TCR |= (1<<1); //Reset Timer0

	NVIC_EnableIRQ(TIMER0_IRQn); //Enable timer interrupt
	
	LPC_TIM0->TCR = 0x01; //Enable timer
}

void rit_init(void)
{
    LPC_SC->PCONP |= (16<<1);               //Power Control for Peripherals register: power up RIT clock
    //LPC_SC->PCLKSEL1 |= (bit26 &amp; bit27);  //Peripheral clock selection: divide clock by 8 (run RIT clock by 12MHz)
		LPC_SC->PCLKSEL1 &= ~(3 << 26); 
    LPC_SC->PCLKSEL1 |=  (1 << 26); 
    LPC_RIT->RICOUNTER = 0;               //set counter to zero
    LPC_RIT->RICOMPVAL = 100;     //interrupt tick every second (clock at 100MHz)
    LPC_RIT->RICTRL |= (1<<1);              // clear timer when counter reaches value
		LPC_RIT->RICTRL |= (2<<1);
    
     
    //enable interrupt
    NVIC_SetPriority(RIT_IRQn, 31);
    NVIC_EnableIRQ(RIT_IRQn);
	
		LPC_RIT->RICTRL |= (3<<1);              // enable timer
}
