/* Copyright 2016 Marcus Geiger. All rights reserved. Use of this
 * source code is governed by a Apache 2.0 license that can be found
 * in the LICENSE file.
 */

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

#include "dbgled.h"


#ifndef WITHOUT_DBGLED_RED
# ifndef WITHOUT_DBGLED_RED_TOGGLE
#  warning "Compiling with red dbgled variable, using more memory"
static volatile uint8_t dbgled_red = 0;
# endif
#endif

#ifndef DBGLED_RED_DDR
# error "Missing definition"
#endif

#ifndef DBGLED_RED_DDR_PIN
# error "Missing definition"
#endif

#ifndef DBGLED_RED_PORT
# error "Missing definition"
#endif

#ifndef DBGLED_RED_PORT_PIN
# error "Missing definition"
#endif


#ifndef WITHOUT_DBGLED_RED
void dbgled_red_init(void)
{
    DBGLED_RED_DDR |= (1 << DBGLED_RED_DDR_PIN);
}

void dbgled_red_on(void)
{
#ifndef WITHOUT_DBGLED_RED_TOGGLE
    dbgled_red = 1;
#endif
    DBGLED_RED_PORT |= (1 << DBGLED_RED_PORT_PIN);
}

void dbgled_red_off(void)
{
#ifndef WITHOUT_DBGLED_RED_TOGGLE
    dbgled_red = 0;
#endif
    DBGLED_RED_PORT &= ~(1 << DBGLED_RED_PORT_PIN);
}

#ifndef WITHOUT_DBGLED_RED_TOGGLE

void dbgled_red_toggle(void)
{
    if (dbgled_red) {
        dbgled_red_off();
    } else {
        dbgled_red_on();
    }
}

#endif

void dbgled_red_pulse(uint8_t p)
{
    uint8_t n;

    for (n = 0; n < p; n++) {
        dbgled_red_on();
        _delay_ms(50);
        dbgled_red_off();
        _delay_ms(50);
    }
}

#endif


#ifndef WITHOUT_DBGLED_GREEN
# ifndef WITHOUT_DBGLED_GREEN_TOGGLE
#  warning "Compiling with green dbgled variable, using more memory"
static volatile uint8_t dbgled_green = 0;
# endif
#endif

#ifndef DBGLED_GREEN_DDR
# error "Missing definition"
#endif

#ifndef DBGLED_GREEN_DDR_PIN
# error "Missing definition"
#endif

#ifndef DBGLED_GREEN_PORT
# error "Missing definition"
#endif

#ifndef DBGLED_GREEN_PORT_PIN
# error "Missing definition"
#endif


#ifndef WITHOUT_DBGLED_GREEN

void dbgled_green_init(void)
{
    DBGLED_GREEN_DDR |= (1 << DBGLED_GREEN_DDR_PIN);
}

void dbgled_green_on(void)
{
#ifndef WITHOUT_DBGLED_GREEN_TOGGLE
    dbgled_green = 1;
#endif
    DBGLED_GREEN_PORT |= (1 << DBGLED_GREEN_PORT_PIN);
}

void dbgled_green_off(void)
{
#ifndef WITHOUT_DBGLED_GREEN_TOGGLE
    dbgled_green = 0;
#endif
    DBGLED_GREEN_PORT &= ~(1 << DBGLED_GREEN_PORT_PIN);
}

#ifndef WITHOUT_DBGLED_GREEN_TOGGLE

void dbgled_green_toggle(void)
{
    if (dbgled_green) {
        dbgled_green_off();
    } else {
        dbgled_green_on();
    }
}

#endif

void dbgled_green_pulse(uint8_t p)
{
    uint8_t n;

    for (n = 0; n < p; n++) {
        dbgled_green_on();
        _delay_ms(50);
        dbgled_green_off();
        _delay_ms(50);
    }
}

#endif

// EOF
