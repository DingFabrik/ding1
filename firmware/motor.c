#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <bitset.h>
#include <stdio.h>
#include "serial.h"
#include "ssi.h"
#include "serencode.h"

#define SSI_CS0n 1

// PB1: PWM out
#define MOTOR_PWM_REG PORTB
#define MOTOR_PWM_DDR DDRB
#define MOTOR_PWM_PIN 1

// PB2: PHASE out
#define MOTOR_PHASE_REG PORTB
#define MOTOR_PHASE_DDR DDRB
#define MOTOR_PHASE_PIN 2

// PB0: FF1 in
#define MOTOR_FF1_REG PORTB
#define MOTOR_FF1_DDR DDRB
#define MOTOR_FF1_PIN 0

// PD7: FF2 in
#define MOTOR_FF2_REG PORTD
#define MOTOR_FF2_DDR DDRD
#define MOTOR_FF2_PIN 7

#define MOTOR_CLIP 230

#define true 1
#define false 0

#define MIN(x,y) ((x)<(y)?(x):(y))
#define MAX(x,y) ((x)>(y)?(x):(y))

void set_rgb(uint8_t ch, uint16_t r, uint16_t g, uint16_t b);

//uint8_t _position;
int16_t set_speed_0 = 0;
int16_t error_int_0 = 0;

#if 0
void pwm_init(void)
{
    // *Counter0*
    // Enable output (OC0A = PD6, OC0B = PD5)
    _SETBIT(DDRD, 5);
    _SETBIT(DDRD, 6);
    // Set output compare
    OCR0A = 0;
    OCR0B = 0;
    
    // Set COM1A[1:0] and COM1B[1:0] to 11 (inverting mode)
    // Note: need inverting mode to get real off state
    // Set WGM[2:0] to 011 (fast PWM mode)
    TCCR0A = _UV(COM0A1) | _UV(COM0A0) | _UV(COM0B1) |
        _UV(COM0B0) | _UV(WGM01) | _UV(WGM00);
    
    // Set clock source to CLKIO/256
    //***//TCCR0B = _UV(CS02);
    TCCR0B = _UV(CS00);
    
    // Don't Enable overflow interrupt
    // TIMSK0 = _UV(TOIE0);
}

void set_motor(uint8_t speed)
{
    OCR0A = 255 - speed;
}
#endif
void pwm_init(void)
{
  // Set output compare
  OCR1A = 0;
  // Set TOP
  ICR1 = 255;

  // Set WGM[13:10] to 1000 (phase and frequency correct PWM mode, TOP = ICR1)
  // Set COM1A[1:0] and COM1B[1:0] to 10 (set/clear OC1A/OC1B on compare match)
  // Set clock source to CLKIO, no prescaler (CS1[2:0] = 001)
  TCCR1A = _UV(COM1A1);
  TCCR1B = _UV(WGM13) | _UV(CS10);
}

void motor_init(void)
{   
    _CLRBIT(MOTOR_FF1_DDR, MOTOR_FF1_PIN);
    _CLRBIT(MOTOR_FF2_DDR, MOTOR_FF2_PIN);

    _SETBIT(MOTOR_PWM_DDR, MOTOR_PWM_PIN);
    _SETBIT(MOTOR_PHASE_DDR, MOTOR_PHASE_PIN);
    
    pwm_init();
}

void set_power(int16_t pwr)
{
    // extract direction
    if(pwr >= 0) {
        _SETBIT(MOTOR_PHASE_REG, MOTOR_PHASE_PIN);
    } else {
        _CLRBIT(MOTOR_PHASE_REG, MOTOR_PHASE_PIN);
        pwr = -pwr;
    }
    
    // clip values
    if(pwr > MOTOR_CLIP) pwr = 0;
    
    OCR1A = pwr;
}

void red_led (uint8_t state);
void green_led (uint8_t state);

int16_t get_position() {
    int16_t dat;

    _CLRBIT(PORTC, SSI_CS0n);
    dat = ssi_read();
    _SETBIT(PORTC, SSI_CS0n);
    
    return (dat >> 6);
}

volatile uint8_t holdoff;

#if defined(__AVR_ATmega8__)
ISR(TIMER2_COMP_vect)
#elif defined(__AVR_ATmega328P__)
ISR(TIMER2_COMPA_vect)
#endif
{
    holdoff--;
}

void timer2_init(void)
{
#if defined(__AVR_ATmega8__)
    OCR2 = F_CPU / (1024UL * 100UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2 = _UV(WGM21) | _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK, OCIE2);
#elif defined(__AVR_ATmega328P__)
    OCR2A = F_CPU / (1024UL * 100UL);
    // Reset counter to OCR2; clock source = t/1024
    TCCR2A = _UV(WGM21);
    TCCR2B = _UV(CS22) | _UV(CS21) | _UV(CS20);
    _SETBIT(TIMSK2, OCIE2A);
#endif
}

void reset_controller(void)
{
    error_int_0 = 0;
    set_speed_0 = 0;
}

void led_control_step(void)
{
    static uint16_t t = 0;
    
    if(14*t < 28560) {
        set_rgb(0, t*14, t*14, 3360);
        set_rgb(1, 65520, t*14, 3360);
        set_rgb(2, 65520, t*14+6, 3360);
        set_rgb(3, 65520, t*14+6, 3360);
        set_rgb(4, 65520, t*14+6, 3360);
    } else {
        set_rgb(0, 28560-(t-2040)*14, 28560-(t-2040)*14, 3360);
        set_rgb(1, 65520, 28560-(t-2040)*14, 3360);
        set_rgb(2, 65520, 28560-(t-2040)*14, 3360);
        set_rgb(3, 65520, 28560-(t-2040)*14, 3360);
        set_rgb(4, 65520, 28560-(t-2040)*14, 3360);
    }

    if(t >= 4080) t = 0;
    t++;
}

void red_led (uint8_t state)
{
    if (state) 
    {
        _SETBIT(PORTD, 3);
    }
    else 
    {
        _CLRBIT(PORTD, 3);
    }
}

void green_led (uint8_t state) 
{
    if (state) 
    {
        _SETBIT(PORTD, 4);
    }
    else 
    {
        _CLRBIT(PORTD, 4);
    }
}

#define MOTOR_CLAMP 230
#define ERROR_INT_CLAMP 30000L
int16_t sp_ctrl_step(int16_t set_speed, int16_t speed, int16_t* error_int)
{
    int16_t error = set_speed - speed;
    *error_int += error;
    
    if(*error_int > ERROR_INT_CLAMP) *error_int = ERROR_INT_CLAMP;
    else if(*error_int < -ERROR_INT_CLAMP) *error_int = -ERROR_INT_CLAMP;
    
    error += *error_int/2;
    
    if(error > MOTOR_CLAMP) error = MOTOR_CLAMP;
    else if(error < -MOTOR_CLAMP) error = -MOTOR_CLAMP;
    
    return (error*4);
}

#define MAX_POS 620
#define MIN_POS 450

#define WAIT_TIME 500  // in units of 10ms
#define TIMEOUT 500     // in units of 10ms

int main()
{
    int16_t power_0 = 0;
    int16_t speed_0;
    int16_t current_position;
    int16_t step = 1;
    // float mult;
    int8_t state=0;
    int16_t wait=0;
    
    uint16_t r;
    uint16_t g;
    uint16_t b;
    
    uint8_t test;
    
    motor_init();
    serial_init();
    // scons_init();
    ssi_init();
    timer2_init();
    twi_init();
    sei();
    
    reset_controller();

    _SETBIT(DDRD, 3);
    _SETBIT(DDRD, 4);
    
    red_led(true);
    pca9685_init();
    red_led(false);
    
    _SETBIT(DDRC, SSI_CS0n);
    _SETBIT(PORTC, SSI_CS0n);
    
    reset_controller();
    
    set_speed_0 = 1;
    set_power(1);
    
    //power_0 = 140;
    //set_power(power_0);
    
    _SETBIT(DDRD, 3);
    
    current_position = 500 - get_position();
    if(current_position < 0) current_position += 1023;
    
    while(1) {
        holdoff = 1;
        while(holdoff > 0) sleep_mode();
        
        //set_power(100);
        
        //continue;
        
        speed_0 = -current_position;
        current_position = 500 - get_position();
        if(current_position < 0) current_position += 1023;
        speed_0 += current_position;
        
        if(state == 0) {
            set_speed_0 = 2;
        } else if(state == 2) {
            set_speed_0 = -1;
        } else {
            set_speed_0 = 0;
        }
        
        wait++;
        if(state == 0) {
            if (current_position >= MAX_POS || wait > TIMEOUT) {
                state = 1; wait = 0;
                green_led(true);
            }
        } else if(state == 1) {
            if(wait > WAIT_TIME) {
                state = 2; wait = 0;
            }
        } else if(state == 2) {
            if (current_position <= MIN_POS || wait > TIMEOUT)
            {
                state = 3; wait = 0;
                green_led(false);
            }
        } else {
            if(wait > WAIT_TIME) {
                state = 0;
                wait = 0;
            }
        }
        
        if(set_speed_0 == 0) {
            power_0 = 0;
            reset_controller();
        } else {
            power_0 = sp_ctrl_step(set_speed_0, 2*speed_0, &error_int_0);
        }
        
        // CLIP TO MAXIMUM POWER
        power_0 = MIN(power_0,220);
        power_0 = MAX(power_0,-100);
        
        set_power(power_0);
        
        se_start_frame(8);
        se_puti16(current_position);
        se_puti16(power_0);
        se_puti16(MAX_POS);
        se_puti16(MIN_POS);
        
        /*if(cmd_ready) {
            switch(cmdbuf[0]) {
            case 0x0E:
                r = (uint16_t) cmdbuf[1] | ((uint16_t) cmdbuf[2] << 8);
                g = (uint16_t) cmdbuf[3] | ((uint16_t) cmdbuf[4] << 8);
                b = (uint16_t) cmdbuf[5] | ((uint16_t) cmdbuf[6] << 8);
                
                set_rgb(0, r, g, b);
                set_rgb(1, r, g, b);
                set_rgb(2, r, g, b);
                break;
            case 0x09:
                set_speed_0 = cmdbuf[1];
                break;
            default:
                break;
            }
            cmd_ready = 0;
        }*/
        
        //reset_controller();
        
//        _SETBIT(PORTD, 4);
        led_control_step();
//        _CLRBIT(PORTD, 4);
    }
    
    /* while(1) {
        _CLRBIT(PORTD, SSI_CS0n);
        dat = ssi_read();
        _SETBIT(PORTD, SSI_CS0n);
        
        printf("%d\r\n", dat >> 6);
        sleep_mode();
    } */
    
    return 0;
}
