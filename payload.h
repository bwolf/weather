/* Copyright 2016 Marcus Geiger. All rights reserved. Use of this
 * source code is governed by a Apache 2.0 license that can be found
 * in the LICENSE file.
 */

#ifndef PAYLOAD_H
#define PAYLOAD_H

// Requires sht11.h, bmp085.h

typedef struct payload {
    uint8_t station_id;
    uint8_t layout;
    union {
        struct {
            bmp085_t bmp085;
            sht11_t  sht11;
            uint8_t  rain_cupfills;
        } layout1;
        // struct {
        //     bmp085_t bmp085;
        //     sht11_t  sht11;
        // } layout2;
    } data;
} payload_t;

#endif // PAYLOAD_H
