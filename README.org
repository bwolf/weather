#+TITLE: README
#+AUTHOR: Marcus Geiger


* Electrical characteristics

|               | U min | U Max | I min/µA | I max/mA |
|---------------+-------+-------+----------+----------|
| ATmega88PA 1) |   1.8 |   5.5 |      0.9 |      2.5 |
| BMP085        |   1.8 |   3.6 |      3.0 |    0.005 |
| SHT7x   2)    |   2.4 |   5.5 |      1.5 |     0.91 |
| nRF24L01+ 3)  |   1.9 |   3.3 |      0.9 |     11.3 |
| NCR18650B     |   2.5 |   4.2 |          |          |

1) With external 32.768 kHz oscillator, which is not actually used
   to drive the system clock, but it is active during power save to
   drive the asynchronous timer to wake the MCU.
2) Needs voltage compensation, i.e. fixed power supply required
3) Needs 3.3V at max, also for input signals (i.e. MCU signals)




