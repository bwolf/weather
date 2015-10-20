// adc.c -- analog digital converter, analog comparator routines.

#include "config.h"

#include <avr/io.h>


void disable_ad_converter(void)
{
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega88PA__)
    ADCSRA &= ~(1 << ADEN);
#else
# error "Unsupported MCU"
#endif
}

void disable_analog_comparator(void)
{
#ifdef __AVR_ATmega8__
    ACSR |= (1 << ACD); // Analog comparator disable
#elif __AVR_ATmega88PA__
    ACSR |= (1 << ACD); // Analog comparator disable
    DIDR1|= (1 << AIN1D) | (1 << AIN0D); // Disable input disable register
#else
# error "Unsupported MCU"
#endif
}

// EOF
