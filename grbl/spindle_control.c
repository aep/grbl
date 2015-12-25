/*
   spindle_control.c - spindle control methods
   Part of Grbl

   Copyright (c) 2012-2015 Sungeun K. Jeon
   Copyright (c) 2009-2011 Simen Svale Skogsrud

   Grbl is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Grbl is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


void spindle_init()
{
#ifdef VARIABLE_SPINDLE
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
#if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
#endif     
    // Configure no variable spindle and only enable pin.
#else  
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
#endif

#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
    SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
#endif
    spindle_stop();
}


void spindle_stop()
{
    // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
#ifdef VARIABLE_SPINDLE
    TCCRA_REGISTER &= ~(1<<COMB_BIT); // Disable PWM. Output voltage is zero.
#if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
#ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
#else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
#endif
#endif
#else
#ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
#else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
#endif
#endif
}


#define CLOCKS_PER_US      16
#define RC_SERVO_SHORT     CLOCKS_PER_US * 1000 / 1024
#define RC_SERVO_LONG      CLOCKS_PER_US * 2000 / 1024


void spindle_set_state(uint8_t state, float rpm)
{
    // Halt or set spindle direction and rpm.
    if (state == SPINDLE_DISABLE) {
        spindle_stop();
    } else {

#ifdef CPU_MAP_ATMEGA2560
        TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
        TCCR0B = _BV(CS12) | _BV(CS10); // 1/1024
        OCR4A = 0xFFFF; // set the top 16bit value
        uint16_t current_pwm;
#else
        TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
        TCCR0B = _BV(CS12) | _BV(CS10); // 1/1024
        uint8_t current_pwm;
#endif

#define RC_SERVO_RANGE (RC_SERVO_LONG-RC_SERVO_SHORT)



        //thread rpm as degrees
        current_pwm = floor((rpm/180.0 * RC_SERVO_RANGE) + RC_SERVO_SHORT);
        OCR_REGISTER = current_pwm;

        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
    }
}


void spindle_run(uint8_t state, float rpm)
{
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.  
    spindle_set_state(state, rpm);
}
