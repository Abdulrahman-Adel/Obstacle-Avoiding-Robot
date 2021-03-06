/*
 * Ultrasonic_Src.c
 *
 * Created: 7/1/2021 2:05:24 PM
 *  Author: Sayed Gomaa
 */ 


 #include "DIO/LIB/STD_TYPES.h"
 #include "DIO/LIB/MATH.h"
  #include <avr/io.h>
 #include "DIO/DIO_Interface.h"
 #include "Ultrasonic_Interface.h"
 #define  F_CPU 1000000UL
 #include <util/delay.h>

 u16 ultarasonic_distance()
 {
	 u16 a,b,high,distance;
	 TCCR1A = 0;
	 // No modification is needed in the register TCCR1A
	 
	 TIFR1 = (1<<ICF1);  // Clear ICF (Input Capture flag)  
	 
	 DIO_SetPinValue(PORT3,PIN7,1);//here we are configuring the rising edge of the trigger 
	 _delay_us(50);
	 DIO_SetPinValue(PORT3,PIN7,0); // waiting about 50 micro second as pulse duration 
	 
	 // again we are configuring the falling after
	 //providing the suitable width duration which must be
	 //more than 10 microsecond
	 
	 //1100 0001
	 //bit0 no prescaling is required
	 //bit7 is set to set ICES1 to 1 to adjust at the rising edge
	 //bit6 to adjust noise cancellation
	 
	 TCCR1B = 0xc1;  	/* Rising edge, no prescaler , noise canceler*/
	 //waiting till the Input Capture Flag set to one
	 //at that case thats mean the the rising edge value occured
	 
	 while ((TIFR1&(1<<ICF1)) == 0);
	 a = ICR1;  		/* Take value of capture register */
	 TIFR1 = (1<<ICF1);  	/* Clear ICF flag */
	 //1000 0001
	 //bit0 no prescaling is required
	 //bit7 is cleared so ICES1 will be adjusted at the falling edge
	 //bit6 to adjust noise cancellation
	 
	 TCCR1B = 0x81;  	/* Falling edge, no prescaler ,noise canceler*/
	 while ((TIFR1&(1<<ICF1)) == 0);
	 b = ICR1;  		/* Take value of capture register */
	 TIFR1 = (1<<ICF1);  	/* Clear ICF flag */
	 
	 //waiting till the Input Capture Flag reset to zero
	 //at that case thats mean the the falling edge value captured in the
	 //input capture register
	 TCNT1=0;
	 TCCR1B = 0;  		/* Stop the timer */
	 high=b-a;
	 distance=((high*34600)/(F_CPU*2)) ;
	 
	 return distance;
 }