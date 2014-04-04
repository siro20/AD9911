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

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>

#include "ad9911.h"

int main(void){

    uint32_t tz;

    /* maximum frequency is 250.000.000,00 Hz */
    tz = 30000000UL;
    AD9911_init();

    /* ramp from 30Mhz to 50Mhz in 10kHz steps */
    while(1){
        AD9911_single_tone( (uint32_t)tz, 0, 0x3ff );
        tz += 10000UL;
        if(tz > 50000000UL)
            tz = 30000000UL;
    };
    return 0;
}
