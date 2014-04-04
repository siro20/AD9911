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

#ifndef _ATMEGA_AD9911_H__
#define _ATMEGA_AD9911_H__
void AD9911_single_tone( uint32_t freq, uint16_t phase, uint16_t amp );
void AD9911_init(void);
void AD9911_linear_sweep_phase( uint16_t S0, uint16_t E0 );
void AD9911_linear_sweep_freq( uint32_t S0, uint32_t E0 );
void AD9911_linear_sweep_amp( uint16_t S0, uint16_t E0);
#endif
