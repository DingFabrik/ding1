/* This file is derived from twitest.c, which carries the license reproduced
 * below:
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 */

#include "twi.h"
#include <compat/twi.h>

/*
 * Maximal number of iterations to wait for a device to respond for a
 * selection.  Should be large enough to allow for a pending write to
 * complete, but low enough to properly abort an infinite loop in case
 * a slave is broken or not present at all.  With 100 kHz TWI clock,
 * transfering the start condition and SLA+R/W packet takes about 10
 * µs.  The longest write period is supposed to not exceed ~ 10 ms.
 * Thus, normal operation should not require more than 100 iterations
 * to get the device to respond to a selection.
 */
#define MAX_ITER 200

/*
 * Saved TWI status register, for error messages only.  We need to
 * save it in a variable, since the datasheet only guarantees the TWSR
 * register to have valid contents while the TWINT bit in TWCR is set.
 */
uint8_t twst;

void twi_init(void)
{
    #if F_CPU < 3600000UL
        TWBR = 10;
    #else
        TWBR = (F_CPU / 100000UL - 16) / 2;
    #endif
}

/* Read len bytes from TWI address addr into buf. */
/* Returns number of bytes read, or -1 on error. */
int twi_read(uint8_t addr, uint8_t* buf, int len)
{
    uint8_t twcr;
    int rv = 0;  /* return value */
    
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);   /* send start condition */
    while(!(TWCR & _BV(TWINT)));  /* wait for transmission */
    switch((twst = TW_STATUS)) {
        case TW_START:
        case TW_REP_START:
            break;
            
        case TW_MT_ARB_LOST:
            goto arb_lost;
            
        default:
            goto error;
    }
    
    TWDR = (addr << 1) | TW_READ;
    TWCR = _BV(TWINT) | _BV(TWEN);   /* start transmission */
    while(!(TWCR & _BV(TWINT)));     /* wait for transmission */

    switch((twst = TW_STATUS)) {
        case TW_MR_SLA_ACK:
            break;
            
        case TW_MR_SLA_NACK:
            goto quit;
            
        case TW_MR_ARB_LOST:
            goto arb_lost;
            
        default:
            goto error;
    }
    
    twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
    while(len > 0) {
        if(len == 1)
            twcr = _BV(TWINT) | _BV(TWEN);  /* send NAK this time */
        TWCR = twcr;
        while(!(TWCR & _BV(TWINT)));  /* wait for transmission */
        switch((twst = TW_STATUS)) {
            case TW_MR_DATA_NACK:
                len = 1;   /* premature end of loop */
                /* FALLTHROUGH */
            case TW_MR_DATA_ACK:
                *buf++ = TWDR;
                rv++;
                break;
            
            default:
                goto error;
        }
        --len;
    }
    
    quit:
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);  /* send stop condition */
    
    return rv;
    
    error:
    rv = -1;
    goto quit;
    
    arb_lost: /* arbitration lost: presently not handled */
    TWCR = _BV(TWINT);  /* release TWI bus */
    return -1;  /* do not send stop condition after arbitration lost */
}

int twi_write(uint8_t addr, uint8_t *buf, int len)
{
    uint8_t n = 0;
    int rv = 0;      /* return value */
    
    restart:
    if (n++ >= MAX_ITER)
        return -1;
    
    begin:
    
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
    while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
    switch ((twst = TW_STATUS))
    {
        case TW_REP_START:     /* OK, but should not happen */
        case TW_START:
            break;
        
        case TW_MT_ARB_LOST:
            goto begin;
        
        default:
            return -1;        /* error: not in start condition */
            /* NB: do /not/ send stop condition */
    }
    
    /* send SLA+W */
    TWDR = (addr << 1) | TW_WRITE;
    TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
    while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
    switch ((twst = TW_STATUS))
    {
        case TW_MT_SLA_ACK:
            break;
        
        case TW_MT_SLA_NACK:    /* nack during select: device busy writing */
            goto restart;
        
        case TW_MT_ARB_LOST:    /* re-arbitrate */
            goto begin;
        
        default:
            goto error;         /* must send stop condition */
    }
    
    for (; len > 0; len--)
    {
        TWDR = *buf++;
        TWCR = _BV(TWINT) | _BV(TWEN); /* start transmission */
        while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
        switch ((twst = TW_STATUS))
        {
            case TW_MT_DATA_NACK:
                goto error;
            case TW_MT_DATA_ACK:
                rv++;
            break;
            default:
                goto error;
        }
    }
    
    quit:
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */
    
    return rv;
    
    error:
    rv = -1;
    goto quit;
}
