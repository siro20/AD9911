AD9911
======

C Library for AD9911: 500 MSPS Direct Digital Synthesizer
Target: ATMEGA32U4 (Teensy 2.0)

The AD9911 is configured for 3-wire SPI. Samplerate is 500.000.000Mhz, PLL enabled, 25Mhz crystal used.

Pin mappings:
PORTD(0) master reset
PORTD(1) IOupdate
PORTB(0) SPI SS
PORTB(3..1) SPI MOSI - MISO - CLK

verified function:
AD9911_single_tone

untested functions:
AD9911_linear_sweep_freq
AD9911_test_tone
