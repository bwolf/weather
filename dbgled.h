/* Copyright 2016 Marcus Geiger. All rights reserved. Use of this
 * source code is governed by a Apache 2.0 license that can be found
 * in the LICENSE file.
 */

#ifndef DBGLED_H
#define DBGLED_H

#ifndef WITHOUT_DBGLED_RED
void dbgled_red_init(void);
void dbgled_red_on(void);
void dbgled_red_off(void);
void dbgled_red_pulse(uint8_t p);
# ifndef WITHOUT_DBGLED_RED_TOGGLE
void dbgled_red_toggle(void);
# else // WITHOUT_DBGLED_RED_TOGGLE
#  define dbgled_red_toggle();
# endif // WITHOUT_DBGLED_RED_TOGGLE
#else // WITHOUT_DBGLED_RED
# define dbgled_red_init()
# define dbgled_red_on()
# define dbgled_red_off()
# define dbgled_red_pulse(p) (void) p
# define dbgled_red_toggle()
#endif // WITHOUT_DBGLED_RED

#ifndef WITHOUT_DBGLED_GREEN
void dbgled_green_init(void);
void dbgled_green_on(void);
void dbgled_green_off(void);
void dbgled_green_pulse(uint8_t p);
# ifndef WITHOUT_DBGLED_GREEN_TOGGLE
void dbgled_green_toggle(void);
# else // WITHOUT_DBGLED_GREEN_TOGGLE
#  define dbgled_green_toggle()
# endif // WITHOUT_DBGLED_GREEN_TOGGLE
#else  // WITHOUT_DBGLED_GREEN
# define dbgled_green_init()
# define dbgled_green_on()
# define dbgled_green_off()
# define dbgled_green_pulse(p) (void) p
# define dbgled_green_toggle()
#endif // WITHOUT_DBGLED_GREEN

#endif // DBGLED_H
