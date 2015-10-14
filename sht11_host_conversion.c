// Test driver for the different SHT11 conversions.
// Use on host system for verification.

#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#define SHT11_VOLTAGE_COMPENSATION_D1 -39.65f

float sht11_temp_convert_sensirion(int16_t raw_temperature)
{
    const float d1 = SHT11_VOLTAGE_COMPENSATION_D1;
    const float d2 = 0.01f; // 14-bit readout
    return (d1 + d2 * (float) raw_temperature);
}

float sht11_humid_convert_sensirion(float Tcels, int16_t SOrh)
{
    // Relative Humidity
    //
    // For compensating non-linearity of the humidity sensor – see
    // Figure 13 – and for obtaining the full accuracy of the sensor
    // it is recommended to convert the humidity readout (SORH) with
    // the following formula
    const float c1 = -2.0468f;    // for 12-bit sensor readout
    const float c2 = +0.0367f;    // for 12-bit sensor readout
    const float c3 = -1.5955E-6f; // for 12-bit sensor readout

    const float RHlinear = c1 + c2 * SOrh + c3 * (SOrh * SOrh);

    // Temperature compensation of Humidity Signal
    //
    // For temperatures significantly different from 25°C (~77°F) the
    // humidity signal requires temperature compensation. The
    // temperature correction corresponds roughly to 0.12%RH/°C @
    // 50%RH. Coefficients for the temperature compensation are given
    // in Table 8.
    const float t1 = 0.01f;    // for 12-bit sensor readout
    const float t2 = 0.00008f; // for 12-bit sensor readout

    float RHtrue = (Tcels - 25.f) * (t1 + t2 * SOrh) + RHlinear;

    if (RHtrue > 100) RHtrue = 100; // Cut if the value is outside of
    if (RHtrue < 0.1) RHtrue = 0.1; // the physical possible range

    return RHtrue;
}

//----------------------------------------------------------------------------------
// calculates temperature [°C] and humidity [%RH]
// input :  humi [Ticks] (12 bit)
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp [°C]
void calc_sth11(float *p_humidity, float *p_temperature)
{
    const float C1 = -2.0468;       // for 12 Bit RH
    const float C2 = +0.0367;       // for 12 Bit RH
    const float C3 = -0.0000015955; // for 12 Bit RH
    const float T1 = +0.01;         // for 12 Bit RH
    const float T2 = +0.00008;      // for 12 Bit RH

    const float d1 = SHT11_VOLTAGE_COMPENSATION_D1;
    const float d2 = +0.01;   // for 14 Bit temperature

    float rh = *p_humidity;   // rh:      Humidity [Ticks] 12 Bit
    float t = *p_temperature; // t:       Temperature [Ticks] 14 Bit
    float rh_lin;             // rh_lin:  Humidity linear
    float rh_true;            // rh_true: Temperature compensated humidity
    float t_C;                // t_C:     Temperature [°C]

    t_C = t * d2 + d1;                    // calc. temperature[°C]from 14 bit temp.ticks
    rh_lin = C3*rh*rh + C2*rh + C1;       // calc. humidity from ticks to [%RH]
    rh_true = (t_C-25)*(T1+T2*rh)+rh_lin; // calc. temperature compensated humidity [%RH]

    if (rh_true > 100) rh_true = 100; // cut if the value is outside of
    if (rh_true < 0.1) rh_true = 0.1; // the physical possible range

    *p_temperature = t_C;
    *p_humidity = rh_true;
}

//--------------------------------------------------------------------
// calculates dew point
// input: humidity [%RH], temperature [°C] // output: dew point [°C]
float calc_dewpoint(float h, float t)
{
    float k, dew_point;
    k = (log10(h)-2)/0.4343 + (17.62*t)/(243.12+t);
    dew_point = 243.12*k/(17.62-k);
    return dew_point;
}

// this is an approximation of 100*log10(x) and does not need the math
// library. The error is less than 5% in most cases.
// compared to the real log10 function for 2<x<100.
// input: x=2<x<100
// output: 100*log10(x)
// Idea by Guido Socher
int log10_approx(uint8_t x)
{
    int l,log;
    if (x==1){
        return(0);
    }
    if (x<8){
        return(11*x+11);
    }
    //
    log=980-980/x;
    log/=10;
    if (x<31){
        l=19*x;
        l=l/10;
        log+=l-4;
    }else{
        l=67*x;
        l=l/100;
        if (x>51 && x<81){
            log+=l +42;
        }else{
            log+=l +39;
        }
    }
    if (log>200) log=200;
    return(log);
}

// calculates dew point
// input: humidity [in %RH], temperature [in C times 10]
// output: dew point [in C times 10]
int calc_dewpoint_approx(uint8_t rh,int t)
{
    // we use integer math and everything times 100
    int k, tmp;
    k = (100 * log10_approx(rh)-20000)/43;
    // we do the calculations in many steps otherwise the compiler will try
    // to optimize and that creates nonsence as the numbers
    // can get too big or too small.
    tmp=t/10;
    tmp=881*tmp;
    tmp=tmp/(243+t/10);
    k+=tmp*2;
    tmp=1762-k;
    tmp=24310/tmp;
    tmp*=k;
    // dew point temp rounded:
    if (tmp<0){
        tmp-=51;
    }else{
        tmp+=51;
    }
    return (tmp/10);
}

int main(int argc, const char *argv[])
{

    // SHT11
    // 6257 1547 2287 5062

    const int16_t raw_temp = 6257;
    const int16_t raw_humi = 1547;

    {
        printf("SHT11 single conversion\n\n");

        printf("%6s %6s\n", "Traw", "Hraw");
        printf("%6d %6d\n", raw_temp, raw_humi);

        float Tcels1 = sht11_temp_convert_sensirion(raw_temp);
        float RHtrue1 = sht11_humid_convert_sensirion(Tcels1, raw_humi);
        printf("%6.2f %6.2f Host: Sensirion convert mine\n", Tcels1, RHtrue1);

        float RHtrue2 = raw_humi;
        float Tcels2 = raw_temp;
        calc_sth11(&RHtrue2, &Tcels2);
        printf("%6.2f %6.2f Host: Sensirion convert datasheet\n", Tcels2, RHtrue2);
        printf("%6.2f %6.2f AVR:  Highlevel output\n", (float)2287 / 100.f, (float)5062 / 100.f);
    }

    printf("**************************************************\n\n"
           "SHT11 tabular conversion with fixed temperature\n"
           "and raw humidity ranging in [0..3500]\n\n");

    const float Tcels = sht11_temp_convert_sensirion(raw_temp);

    printf("Traw Hraw Tcels Tcels2 RH1    RH2    DP    DP2\n");

    int16_t SOrh;
    float RHtrue, RHtrue2, Tcels2;
    float DP;
    float DP2;
    for (SOrh = 0; SOrh <= 3500; SOrh += 50) {
        RHtrue = sht11_humid_convert_sensirion(Tcels, SOrh);

        Tcels2 = raw_temp;
        RHtrue2 = SOrh;
        calc_sth11(&RHtrue2, &Tcels2);
        DP = calc_dewpoint(RHtrue2, Tcels2);
        DP2 = calc_dewpoint_approx((int) RHtrue2, Tcels2 * 10) / 10.0;
        printf("%4hu %4hu %5.2f %5.2f  %6.2f %6.2f %6.2f %6.2f\n", raw_temp, SOrh, Tcels, Tcels2, RHtrue, RHtrue2, DP, DP2);
    }

    return 0;
}

// EOF
