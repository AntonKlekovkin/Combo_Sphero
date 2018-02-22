#include "mbed_assert.h"
#include "analogin_api.h"

#include "cmsis.h"
#include "pinmap.h"

#define ANALOGIN_MEDIAN_FILTER      1

#define ADC_10BIT_RANGE             0x3FF
#define ADC_12BIT_RANGE             0xFFF

static inline int div_round_up(int x, int y) {
  return (x + (y - 1)) / y;
}

static const PinMap PinMap_ADC[] = {
    {P0_23, ADC0_0, 1},
    {P0_24, ADC0_1, 1},
    {P0_25, ADC0_2, 1},
    {P0_26, ADC0_3, 1},
    {P1_30, ADC0_4, 3},
    {P1_31, ADC0_5, 3},
    {P0_2,  ADC0_7, 2},
    {P0_3,  ADC0_6, 2},
    {NC,    NC,     0}
};

#define ADC_RANGE ADC_12BIT_RANGE


//void init_adc(analogin_t *obj, PinName pin) 
void init_adc(PinName pin) 
{
		//int _adc;
		//_adc = (ADCName)pinmap_peripheral(pin, PinMap_ADC);
    //obj->adc = (ADCName)pinmap_peripheral(pin, PinMap_ADC);
    //MBED_ASSERT(obj->adc != (ADCName)NC);
    
    // ensure power is turned on
    LPC_SC->PCONP |= (1 << 12);
    
    // set PCLK of ADC to /1
    LPC_SC->PCLKSEL0 &= ~(0x3 << 24);
    LPC_SC->PCLKSEL0 |= (0x1 << 24);
    uint32_t PCLK = SystemCoreClock;
    
    // calculate minimum clock divider
    //  clkdiv = divider - 1
    uint32_t MAX_ADC_CLK = 13000000;
    uint32_t clkdiv = div_round_up(PCLK, MAX_ADC_CLK) - 1;
	
		//uint32_t clkdiv = 4;
    
    // Set the generic software-controlled ADC settings
    LPC_ADC->ADCR = (0 << 0)      // SEL: 0 = no channels selected
                  | (clkdiv << 8) // CLKDIV: PCLK max ~= 25MHz, /25 to give safe 1MHz at fastest
                  | (0 << 16)     // BURST: 0 = software control
                  | (0 << 17)     // CLKS: not applicable
                  | (1 << 21)     // PDN: 1 = operational
                  | (0 << 24)     // START: 0 = no start
                  | (0 << 27);    // EDGE: not applicable
    
    pinmap_pinout(pin, PinMap_ADC);
}

void adc_start_read(int num_channel) 
{
    // Select the appropriate channel and start conversion
    LPC_ADC->ADCR &= ~0xFF;
    LPC_ADC->ADCR |= 1 << num_channel;
    LPC_ADC->ADCR |= 1 << 24;
}

static inline uint32_t adc_read_channel2() 
{
    // Repeatedly get the sample data until DONE bit
    unsigned int data;
				
    do 
		{
      data = LPC_ADC->ADDR2;
    } 
		while ((data & ((unsigned int)1 << 31)) == 0);
    
    // Stop conversion
    LPC_ADC->ADCR &= ~(1 << 24);
    
    return (data >> 4) & ADC_RANGE; // 12 bit
}

static inline uint32_t adc_read_channel3() 
{
    // Repeatedly get the sample data until DONE bit
    unsigned int data;
				
    do 
		{
      data = LPC_ADC->ADDR3;
    } 
		while ((data & ((unsigned int)1 << 31)) == 0);
    
    // Stop conversion
    LPC_ADC->ADCR &= ~(1 << 24);
    
    return (data >> 4) & ADC_RANGE; // 12 bit
}



