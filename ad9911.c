/*
*
*    Library for Analog Devices AD9911
*    Copyright (C) 2013  Patrick Rudolph <siro@das-labor.org>
*
*    This program is free software; you can redistribute it and/or modify it under the terms 
*    of the GNU General Public License as published by the Free Software Foundation; either version 3 
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
*    without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*    See the GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License along with this program; 
*    if not, see <http://www.gnu.org/licenses/>.
*
*    This code was written for Teensy 2.0 (ATMEGA32U4). 
*/

#include <stdio.h>
#include <inttypes.h>
#include <avr/io.h>
#include "ad9911.h"

#define AD9911_RWb 0x80
#define AD9911_addressmask 0x1f
#define AD9911_MAXREG 0x18
#define AD9911_ampmask 0x3ff

/* hardcode samplerate here 500.000.000Mhz */
#define fs 500000000UL

enum AD9911_addr_e{
CSR = 0,
FR1,
FR2,
CFR,
CTW0,
CPOW0,
ACR,
LSR,
RDW,
FDW,
CTW1,
CTW2,
CTW3,
CTW4,
CTW5,
CTW6,
CTW7,
CTW8,
CTW9,
CTW10,
CTW11,
CTW12,
CTW13,
CTW14,
CTW15,
};

uint32_t AD9911_reg[AD9911_MAXREG + 1];

/* these routines are specific for ATMEGA32 */
void WriteByteSPI(unsigned char byte)
{
    SPDR = byte;                    //Load byte to Data register
    while(!(SPSR & (1<<SPIF)));     // Wait for transmission complete 
}

unsigned char ReadByteSPI(unsigned char addr)
{
    SPDR = addr;                    //Load byte to Data register
    while(!(SPSR & (1<<SPIF)));     // Wait for transmission complete 
    addr=SPDR;
    return addr;
}

/* in SPI write mode CPOL=0 CPHA=0, write msb bit first, write msb byte first */ 
void _AD9911_write( uint8_t addr, uint32_t data )
{
    uint8_t AD9911_cmd, len;
    int i;

    if( addr > AD9911_MAXREG )
        return;

    // SPI CPOL=0 CPHA=0
    SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0)|(0<<CPOL)|(0<<CPHA); 

    PORTB &= ~(1<<0); // ss low

    AD9911_cmd = addr & AD9911_addressmask;	

    if( addr > 7 || addr == 4 )
        len = 4;
    else if( addr == 7 || addr == 5 || addr == 2 )
        len = 2;  
    else if( addr == 0 )
        len = 1;
    else
        len = 3;
    
    WriteByteSPI( AD9911_cmd );
    for(i=len-1;i >= 0;i--)
    {
        WriteByteSPI( (uint8_t)((data>>(8*i)) & 0xff) );
    }
        
    PORTB |= (1<<0);    // ss high
}

/* in SPI read mode CPOL=1 CPHA=1, read msb bit first, read msb byte first */
void _AD9911_read( uint8_t addr, uint32_t *data )
{
    uint8_t AD9911_cmd, len;
    int i;

    if( addr > AD9911_MAXREG )
        return;

    // SPI CPOL=1 CPHA=1
    SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0)|(1<<CPOL)|(1<<CPHA); 
    
    PORTB &= ~(1<<0);   // ss low

    AD9911_cmd = addr & AD9911_addressmask;	
    AD9911_cmd |= AD9911_RWb;

    if( addr > 7 || addr == 4 )
        len = 4;
    else if( addr == 7 || addr == 5 || addr == 2 )
        len = 2;  
    else if( addr == 0 )
        len = 1;
    else
        len = 3;
        
    WriteByteSPI( AD9911_cmd );
    *data = 0;
    for(i=len-1;i >= 0;i--)
    {
        *data |= ReadByteSPI(0)<<(8*i);
    } 

    PORTB |= (1<<0);    // ss high
}

/* rising edge on IOUpdate pin loads the contents of the shift register */
void _AD9911_IOupdate(void)
{
    int i;
    /* start at FR1 */
    for(i=1;i<AD9911_MAXREG;i++)
        _AD9911_write(i, AD9911_reg[i]);
        
    PORTD |= 2;     // IO UPDATE

    PORTD &= ~2;
}

void _AD9911_select_channel(uint8_t channel)
{
    /* select channel to write to */
    if( channel > 3 )
        return;
    _AD9911_write(CSR, (AD9911_reg[CSR] & 0x0f) | (1<<(channel+4)) );
}

/* set constant values and disable all channels */ 
void AD9911_init()
{
    int i;
    DDRB = 0x7;     // SS, SCLK, MOSI
    DDRD = 0x3; 
    PORTD = (1<<0);     // master reset high
    for(i=0;i<0xff;i++);
    PORTD = 0;  // master reset low, IOUpdate low

    for(i=0;i<AD9911_MAXREG;i++)
        AD9911_reg[i] = 0;
    
    /* MSB first , 3-wire SPI mode, enable all channels */
    AD9911_reg[CSR] = ((uint32_t)0<<0) | ((uint32_t)1 << 1) |((uint32_t)0x0f<<4);
    _AD9911_write(CSR, AD9911_reg[CSR]);

    /* PLL 20x, vco gain high range */
    AD9911_reg[FR1] = ((uint32_t)20<<18) | ((uint32_t)1<<23);
    _AD9911_write(FR1, AD9911_reg[FR1]);
    
    /* full scale dac output, digital power down */
    AD9911_reg[CFR] = ((uint32_t)3<<8)|((uint32_t)1<<7);
    _AD9911_write(CFR, AD9911_reg[CFR]);
    
    _AD9911_select_channel( 1 ); /* all registers are written to channel 1 on IOUpdate */
}

uint32_t _AD9911_FTW_from_freq( uint32_t fo )
{
    uint64_t tmp;
    if( fo > fs/2 )
        return 0;
    tmp = (uint64_t)fo << 32;
    tmp /= fs;
    return tmp;
}

void AD9911_single_tone( uint32_t freq, uint16_t phase, uint16_t amp )
{
    _AD9911_select_channel( 1 ); /* all registers are written to channel 1 on IOUpdate */
    
    AD9911_reg[FR1] &= ~((uint32_t)1<<2);    /* disable test-tone mode */

    AD9911_reg[CFR] &= ~((uint32_t)1<<7);   /* disable digital power down */
    AD9911_reg[CTW0] = _AD9911_FTW_from_freq( freq );
    AD9911_reg[CPOW0] = (uint32_t)phase;
    AD9911_reg[ACR] = (uint32_t)amp & AD9911_ampmask;

    _AD9911_IOupdate();
}

/* untested */
void AD9911_test_tone( uint32_t freq, uint16_t freq2, uint16_t amp )
{
    _AD9911_select_channel( 1 ); /* all registers are written to channel 1 on IOUpdate */

    AD9911_reg[CFR] &= ~((uint32_t)1<<7); /* disable digital power down */
    AD9911_reg[FR1] |= ((uint32_t)1<<2);    /* enable test-tone mode */
    AD9911_reg[CTW0] = _AD9911_FTW_from_freq( freq );
    _AD9911_IOupdate();

    _AD9911_select_channel( 0 ); /* all registers are written to channel 0 on IOUpdate */

    AD9911_reg[CFR] &= ~((uint32_t)1<<7); /* disable digital power down */
    AD9911_reg[CFR] |= ((uint32_t)7<<16); /* set aux channel amp to -6dB */
    AD9911_reg[CTW0] = _AD9911_FTW_from_freq( freq2 );
    AD9911_reg[ACR] = (uint32_t)amp & AD9911_ampmask;
    _AD9911_IOupdate();
}

/* untested */
void AD9911_linear_sweep_freq( uint32_t S0, uint32_t E0 )
{
    _AD9911_select_channel( 1 ); /* all registers are written to channel 1 on IOUpdate */

    AD9911_reg[FR1] &= ~((uint32_t)1<<5); /* enable SYNC_CLK pin */

    AD9911_reg[CTW0] = (uint32_t)S0;
    AD9911_reg[CTW1] = (uint32_t)E0;
    AD9911_reg[CFR] &= ~((uint32_t)3<<22);
    AD9911_reg[CFR] |= ((uint32_t)2<<22);	/* enable frequency sweep */

    AD9911_reg[FR1] &= ~((uint32_t)3<<8);
    AD9911_reg[FR1] |= ((uint32_t)0<<8);		/* has to be 2-level modulation */

    AD9911_reg[CFR] |= ((uint32_t)1<<14)|((uint32_t)1<<15);	/* enable linear sweep, set no dwell mode (reset to 0 on reaching E0) */

    _AD9911_IOupdate();
}

