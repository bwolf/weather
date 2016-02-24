/* Copyright 2016 Marcus Geiger. All rights reserved. Use of this
 * source code is governed by a Apache 2.0 license that can be found
 * in the LICENSE file.
 */

#ifndef PRESSURE_H
#define PRESSURE_H

float pressure_to_altitude(int32_t p);

// NN is normal null
uint32_t pressure_to_nn(int32_t p);
uint16_t pressure_to_nn16(int32_t p);

#endif // PRESSURE_H
