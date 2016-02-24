/* Copyright 2016 Marcus Geiger. All rights reserved. Use of this
 * source code is governed by a Apache 2.0 license that can be found
 * in the LICENSE file.
 */

#ifndef RAIN_H
#define RAIN_H

void rain_init(void);
uint8_t rain_cupfill_count(void);
uint8_t rain_busy_p(void);
void rain_periodic(void);

#endif // RAIN_H
