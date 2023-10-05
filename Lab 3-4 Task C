#include "stm32f4xx.h"
#define __FPU_PRESENT  1
#define ARM_MATH_CM4
#include "arm_math.h"
// configuration routines 

void GPIO_config(void);// Function prototype for GPIO configuration
void DAC_config(void);// function prototype for DAC configuration
void ADC_config(void);
float moving_average(float val);

volatile float data_value = 0.0;// Value in volts
volatile float volt_value;
volatile float MA_volt_value;
volatile uint8_t dac8bit;// The value to be loaded into 8 Bit register as using the 8 bit mode
volatile uint8_t adcval = 1;
float freq = 5;

#define array_size 100

volatile float float_array[array_size];


float moving_average(float val){
  for(int i = 1; i< array_size; i++){
    float_array[i-1] = float_array[i];
  }
  float_array[array_size-1] = val;
  
  float average = 0;
  for(int i = 0; i< array_size; i ++){
    average += float_array[i];
  }
  
  average = average/array_size;
  
  return average;
}

int main()
{

        GPIO_config();
        DAC_config();
        ADC_config();
        
        //Enabling channel 1 
        DAC->CR |= 1UL;
        //Placing the value in data holding register using 8 bit rigth alligned mode 

        while(1)
        {
          ADC1->CR2 |= ADC_CR2_ADON; // enabling the ADC
          ADC1->CR2 |= (1 << 30); // starting the ADC 
          
          while((ADC1->SR & 0x2) != 0x2) // check until the EOC flag is not set
	{	}
	adcval = ADC1->DR;// Take the value from the ADC register to a variable
	
	volt_value = (((float)adcval)/255)*3;// conversion into voltage values.
	
	/*comment out next line for continous conversion mode*/
	ADC1->CR2 &= ~ADC_CR2_ADON; // Stopping in single conversion mode
	ADC1->SR = 0x00;// resetting all the flags 
          
          
        MA_volt_value = moving_average(volt_value);
        
        data_value  = MA_volt_value;
        dac8bit  =	(uint8_t)((data_value/3)*255);
        DAC->DHR8R1 = dac8bit;
        
        for (long j=0;j<4000*100;j++)// small delay
                {	}
        
        
  }
}

void GPIO_config(void)
{
        // enable the clock of GPIO PORT A
        RCC->AHB1ENR |= (0x01 << 0);
        // set the pinA4 to analog mode 
        //GPIOA->MODER   &= ~((3ul << 2*pin)); pin corresponds to the pin number being used 
        GPIOA->MODER |= (0x03 << 2*4);  // choosing the analog mode by moving 11 to bit 8 and 9.
	GPIOA->MODER |= (0x03 << 0);  // choosing the analog mode by moving 11 to bit 8 and 9.
        //GPIOA->PUPDR &= ~(0x00000003);
}

void ADC_config(void)
{
	RCC->APB2ENR = (0x01<<8);
	ADC1->CR1 &= 0x00000000;
	ADC1->CR1 |= ( 0x2 << 24 );// set resolutions to 8 bits + channel 0
	
	//ADC1->CR1 |= (0x10);
	ADC1->CR2 &= 0x00000000;
	ADC1->CR2 |= (0x1 << 10);// eoc flag
	/*include next line for continous conversion mode*/
//	ADC1->CR2 = (0x1 << 1);// continous conversion mode 
	ADC1->SMPR2 = 1;
	ADC1->SQR1 &= 0x00000000;
	ADC1->SQR1 |= (0x0 << 20); // 1 conversion 
	ADC1->SQR3 = 0x00;
}

void DAC_config(void)
{
        // Enabling the DAC clock 
        RCC->APB1ENR |= (0x01 << 29);
        // Resetting all the other bits according to the reset values specified in the reference manual
        DAC->CR &= ~(0x3FFFUL << 16);
        // Channel 1 output buffer enable  
        DAC->CR &= ~(0x1FFFUL << 1);
}
