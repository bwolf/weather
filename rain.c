/* rain.c -- Rain measurement with Davis rain unit.
 *
 * Internally the sensor contains a seesaw with a cup on each side.
 * The sessaw goes down if the cup fills with water (equiv. 0.2mm)
 * switching a reed relais. Thus one impulse of the seesaw equals
 * 0.2mm of rainfall.
 *
 *
 * Assumption
 *
 * Maximum rainfall per day 50 mm
 * Maximum rainfall per hour 20 mm
 *
 * 50 / 0.2 = 250 cup fills per day, thus uint8_t as counter variable
 * is enough to capture the rainfall of a whole day, granted that the
 * cup fills are evenly distributed over the day.
 *
 * 20 / 0.2 = 100 cup fills at maximum per hour. 100/3600 = 1/36, i.e.
 * one impulse each 36s, granted that the cup fills are evenly
 * distributed. Proof: 3600/36 = 100.
 *
 * The reed relais needs to be de-bounced. A timer resolution of a
 * number smaller than 36s will do.
 *
 *
 * Solution
 *
 * The reed relais triggers an external interrupt. The signal is
 * de-bounced with a large interval timer.
 *
 * To capture the interrupt count a uin16_t will be used and a timer
 * interval of < 16s is used to de-bounce the signal.
 *
 */

#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "dbgled.h"


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

#ifndef RAIN_DEBOUNCE_TIMER_OFF
# error "Missing definition RAIN_DEBOUNCE_TIMER_OFF."
#endif

#ifndef RAIN_DEBOUNCE_TIMER_START
# error "Missing definition RAIN_DEBOUNCE_TIMER_START."
#endif

#ifndef RAIN_DEBOUNCE_TIMER_OVERFLOW_INTERRUPT_ENABLE
# error "Missing definition RAIN_DEBOUNCE_TIMER_OVERFLOW_INTERRUPT_ENABLE."
#endif

#ifndef RAIN_DEBOUNCE_TIMER_OVERFLOW_INTERRUPT_VECT
# error "Missing definition RAIN_DEBOUNCE_TIMER_OVERFLOW_INTERRUPT_VECT."
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


#define RAINTIMER_VALUE_OFF   0
#define RAINTIMER_VALUE_START 1

// Delay value 500ms for timer interrupt
#define RAINTIMER_VALUE_CUTOFF 30 // TODO abstraction in config.h needed

static volatile uint8_t raintimer = RAINTIMER_VALUE_OFF;
static volatile uint8_t count_cup_fills = 0;

#define RAINTIMER_RUNNING_P()   (raintimer > RAINTIMER_VALUE_OFF)
// TODO maybe really stop/start the timer to prevent powering up the MCU!
// TODO use for stopping RAIN_TIMER_OFF()
#define RAINTIMER_START()       raintimer = RAINTIMER_VALUE_START
#define RAINTIMER_STOP()        raintimer = RAINTIMER_VALUE_OFF
#define RAINTIMER_INC_IF_ON()   if (RAINTIMER_RUNNING_P()) { ++raintimer; }
#define RAINTIMER_DEBOUNCED_P() (raintimer > RAINTIMER_VALUE_CUTOFF)



// Note: interrupts must be enabled elsewhere
void rain_init(void)
{
    // Configure a timer/counter to de-bounce the signal.
    RAIN_DEBOUNCE_TIMER_OFF();
    RAIN_DEBOUNCE_TIMER_OVERFLOW_INTERRUPT_ENABLE();
    RAIN_DEBOUNCE_TIMER_START(); // TODO

    // Configure input PIN for IRQ
    // TODO ensure that now pull-up/pull-down is required (short cicruit)
    RAIN_DDR &= ~(1 << RAIN_DDR_NO);   // input
    RAIN_PORT &= ~(1 << RAIN_PORT_NO); // disable pull-up

    // Configure IRQ
    RAIN_INTERRUPT_FALLING_EDGE();
    RAIN_INTERRUPT_ENABLE();
}

uint8_t rain_busy_p(void)
{
    return RAINTIMER_RUNNING_P();
}

// Worker function to be called periodically
void rain_periodic(void)
{
    if (RAINTIMER_RUNNING_P()) {
        if (RAINTIMER_DEBOUNCED_P()) {
            RAINTIMER_STOP();
        }
    }
}

ISR(RAIN_DEBOUNCE_TIMER_OVERFLOW_INTERRUPT_VECT)
{
    RAINTIMER_INC_IF_ON();
}

ISR(RAIN_INTERRUPT_VECT)
{
    if (!RAINTIMER_RUNNING_P()) {
        RAINTIMER_START();
        ++count_cup_fills;

        // TODO toggle led for test
    }
}

// EOF
