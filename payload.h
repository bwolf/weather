#ifndef PAYLOAD_H
#define PAYLOAD_H

// Requires sht11.h, bmp085.h

typedef struct payload {
    bmp085_t bmp085;
    sht11_t sht11;
} payload_t;

#endif // PAYLOAD_H
