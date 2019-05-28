#include <avr/io.h>
#include <avr/interrupt.h>

volatile unsigned int overflow_count=0;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


void ISR(TIMER0_OVF_vect)
{
    if (overflow_count < 0xFFFF) overflow_count++;
}
