#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <bitset.h>
#include <stdio.h>

#include "twi.h"
#include "serial.h"
#include "serencode.h"

#define PCA_ADDR 0x55

#define PCA9685_MODE1           0x00
#define PCA9685_MODE1_ALLCALL   0
#define PCA9685_MODE1_SUB3      1
#define PCA9685_MODE1_SUB2      2
#define PCA9685_MODE1_SUB1      3
#define PCA9685_MODE1_SLEEP     4
#define PCA9685_MODE1_AI        5
#define PCA9685_MODE1_EXTCLK    6
#define PCA9685_MODE1_RESTART   7

#define PCA9685_MODE2           0x01
#define PCA9685_MODE2_OUTNE0    0
#define PCA9685_MODE2_OUTNE1    1
#define PCA9685_MODE2_OUTDRV    2
#define PCA9685_MODE2_OCH       3
#define PCA9685_MODE2_INVRT     4

#define PCA9685_LED_ADDR(led) (4*(led) + 6)

void pca9685_init()
{
    uint8_t data[] = { PCA9685_MODE1,
        _UV(PCA9685_MODE1_AI) | _UV(PCA9685_MODE1_ALLCALL) };
    twi_write(PCA_ADDR, data, sizeof(data));
}

void set_rgb(uint8_t ch, uint16_t r, uint16_t g, uint16_t b)
{
    uint8_t data[] = { PCA9685_LED_ADDR(3*ch),
                       0x00, 0x00, (r>>4) & 0xFF, (r>>12) & 0xFF,
                       0x00, 0x00, (g>>4) & 0xFF, (g>>12) & 0xFF,
                       0x00, 0x00, (b>>4) & 0xFF, (b>>12) & 0xFF };
    twi_write(PCA_ADDR, data, sizeof(data));
}

#if 0
int main()
{
    int result;
    uint16_t r, g, b;
    
    serial_init();
    twi_init();
    sei();
    
    _SETBIT(DDRD, 4);
    
    pca9685_init();
    
    /* while(1) {
        if(cmd_ready) {
            switch(cmdbuf[0]) {
            case 0x0E:
                r = (uint16_t) cmdbuf[1] | ((uint16_t) cmdbuf[2] << 8);
                g = (uint16_t) cmdbuf[3] | ((uint16_t) cmdbuf[4] << 8);
                b = (uint16_t) cmdbuf[5] | ((uint16_t) cmdbuf[6] << 8);
                set_rgb(0, r, g, b);
                break;
            default:
                break;
            }
            cmd_ready = 0;
        }
        
        sleep_mode();
    } */
    
    uint16_t t = 0;
    
    while(1) {
        if(14*t < 28560)
            set_rgb(0, 65520, t*14, 3360);
        else
            set_rgb(0, 65520, 28560-(t-2040)*14, 3360);
        
        if(t >= 4080) t = 0;
        t++;
        
        _delay_ms(10);
    }
    
    return 0;
}
#endif
