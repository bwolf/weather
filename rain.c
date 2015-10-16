#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>


#ifndef RAIN_DDR
# error "Missing definition RAIN_DDR."
#endif

#ifndef RAIN_DDR_NO
# error "Missing definition RAIN_DDR_NO."
#endif

#ifndef RAIN_PORT
# error "Missing definition RAIN_PORT."
#endif

#ifndef RAIN_PORT_NO
# error "Missing definition RAIN_PORT_NO."
#endif

#ifndef RAIN_INTERRUPT_FALLING_EDGE
# error "Missing definition RAIN_INTERRUPT_FALLING_EDGE."
#endif

#ifndef RAIN_INTERRUPT_ENABLE
# error "Missing definition RAIN_INTERRUPT_ENABLE."
#endif

#ifndef RAIN_INTERRUPT_VECT
# error "Missing definition RAIN_INTERRUPT_VECT."
#endif


#define RAINTIMER_VALUE_OFF 0
#define RAINTIMER_VALUE_START 1

// Delay value 500ms for timer interrupt
#define RAINTIMER_VALUE_CUTOFF 30

static volatile uint8_t raintimer = RAINTIMER_VALUE_OFF;
static volatile uint8_t raincounter_deci_meters = 0;

#define RAINTIMER_RUNNING_P() (raintimer > RAINTIMER_VALUE_OFF)
#define RAINTIMER_START() raintimer = RAINTIMER_VALUE_START
#define RAINTIMER_STOP() raintimer = RAINTIMER_VALUE_OFF
#define RAINTIMER_INC_IF_ON() if (RAINTIMER_RUNNING_P()) { ++raintimer; }
#define RAINTIMER_DEBOUNCED_P() (raintimer > RAINTIMER_VALUE_CUTOFF)



void rain_init(void)
{
    // TODO missing timer setup
    // Timer interrupt
// #if defined(__AVR_ATmega32U4__)
//     TCCR0B |= _BV(CS02) | _BV(CS00);
//     TIMSK0 |= _BV(TOIE0);
// #else
//  #warning "Unknown uC"
// #endif

    // TODO ensure that now pull-up/pull-down is required (short cicruit)
    // Configure input PIN for IRQ
    RAIN_DDR &= ~(1 << RAIN_DDR_NO);   // input
    RAIN_PORT &= ~(1 << RAIN_PORT_NO); // disable pull-up

    // Configure IRQ
    RAIN_INTERRUPT_FALLING_EDGE();
    RAIN_INTERRUPT_ENABLE();
}

void missing_code_from_rainsensor_main_c_while_loop(void)
{
    // TODO missing
}

// TODO missing timer interrupt
// #if defined(__AVR_ATmega32U4__)
// ISR(TIMER0_OVF_vect)
// #else
// # warning "Unknown uC"
// #endif
// {
//     RAINTIMER_INC_IF_ON();
// }

ISR(RAIN_INTERRUPT_VECT)
{
    if (!RAINTIMER_RUNNING_P()) {
        RAINTIMER_START();
        ++raincounter_deci_meters;
        PINC |= _BV(PINC6); // Toggle IRQ debug led
    }
}

// EOF
