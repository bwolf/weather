#+TITLE: README
#+AUTHOR: Marcus Geiger

* Open points/Todos
** TODO ATmega88PA supports pin toggle natively
#+BEGIN_SRC c
  PINB |= (1 << PINB1); // toggle the I/O port output pin
#+END_SRC

** TODO Strange data
   Data from 28.10.2015 at 08:30.
   #+BEGIN_EXAMPLE
     BMP085   MS5611  SHT11
     75 10124 38 1013 657 8916
     75 10124 38 1013 651 8915
   #+END_EXAMPLE

** TODO Finish rain sensor code
** TODO Clean up wireless high level code 
** TODO Integrate IIR filter
   Use a warm up loop after reset to warm up the filter value.
   #+BEGIN_SRC c
     #include <stdint.h>
     #include <stdio.h>

     /**
      ,* @brief Approximation for IIR filter of \f$log2(1/\alpha)\f$.
      ,* @details This is roughly \f$ log2(1 / \alpha) \f$.
      ,*/
     #define IIR_FILTER_BITS 2

     /**
      ,* @brief   IIR filter for long values.
      ,* @details Implement a first order IIR (infinite impulse response) filter
      ,*          to approximate a K sample moving average.
      ,*
      ,*          This function implements the equation:
      ,*          \f$ y_n = \alpha * x_n + (1-\alpha) * y_{n-1 } \f$.
      ,*
      ,* @see     http://is.gd/gx3kVm.
      ,*
      ,* @param filter - a signed fixed-point value like 15.42. Choose
      ,* initial value with a reasonable start value according to the dataset.
      ,* @param   sample - the 16-bit value of the current sample.
      ,*/
     uint16_t lowpass_filter(uint32_t *filter, uint16_t sample)
     {
         uint32_t local_sample = sample << 16;

         ,*filter += (local_sample - *filter) >> IIR_FILTER_BITS;

         return (uint16_t) ((*filter+0x8000) >> 16); // Round by adding .5 and truncating.
     }

     int main(int argc, char *argv[])
     {
         uint32_t filter;
         uint16_t data[] = {
             1013, 1012, 1003, 1005, 999, 1014, 1002, 1005, 1019, 1012, 999, 1012, 1003,
             1011, 1020, 1001, 1009, 1008, 1003, 1007, 1014, 1019, 1011, 1015, 1008, 1008,
             1001, 1023, 1007, 1005, 1016, 1014, 1001, 1021, 1016, 1023, 1023, 1010, 1014,
             1006, 999, 1020, 1010, 1005, 1011, 1001, 1018, 1001, 1002, 1004, 1012, 1008
         };

         filter = 66000000;
         unsigned int n;
         for (n = 0; n < sizeof(data)/sizeof(*data); n++) {
             short sample = data[n];
             long filter_save = filter;
             short res = lowpass_filter(&filter, sample);
             printf("%ld\t%2hd\t%2hd\n", filter_save, sample, res);
         }

         return 0;
     }

#+END_SRC

** TODO Main timer higher prescaler

** TODO SHT7x Socket Connector

** TODO Use 32.768 kHz oscillator as main clock?
   Compare current consumption of internal vs. external clock in data sheet.

** TODO Verify and optimize BMP085 conversion
   #+BEGIN_EXAMPLE
     BMP085 raw (coeff/ut/up) 7106 -1261 -14633 34391 25021 17113 5498 69 -32768 -11075 2432 25208 316195
   #+END_EXAMPLE

** TODO Verify and optimize MS5611 conversion

   Raw values:
   #+BEGIN_EXAMPLE
     MS5611 raw (coeff/D1/D1) 216 61 254 44 31 116 96 146 8064280 8414176
   #+END_EXAMPLE

   Output:
   #+BEGIN_EXAMPLE
     241 1029   
   #+END_EXAMPLE

** TODO Verify and optimize SHT11 conversion
   #+BEGIN_EXAMPLE
     SHT11 raw (temp/humid) 6253 1513
   #+END_EXAMPLE

** TODO Use error handling with SHT11, BMP085, MS5611 in main


* Electrical characteristics

|               | U min | U Max | I min/µA | I max/mA |
|---------------+-------+-------+----------+----------|
| ATmega88PA 1) |   1.8 |   5.5 |      0.9 |      2.5 |
| BMP085        |   1.8 |   3.6 |      3.0 |    0.005 |
| SHT7x   2)    |   2.4 |   5.5 |      1.5 |     0.91 |
| nRF24L01+ 3)  |   1.9 |   3.3 |      0.9 |     11.3 |
| MS5611        |   1.8 |   3.6 |     0.14 |   0.0125 |
| NCR18650B     |   2.5 |   4.2 |          |          |

1) With external 32.768 kHz oscillator, which is not actually used
   to drive the system clock, but it is active during power save to
   drive the asynchronous timer to wake the MCU.
2) Needs voltage compensation, i.e. fixed power supply required
3) Needs 3.3V at max, also for input signals (i.e. MCU signals)


* COMMENT Footer
# Local Variables:
# mode: org
# End:
