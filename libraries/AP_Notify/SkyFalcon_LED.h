/*
 *  AP_Notify Library. 
 * based upon a prototype library by David "Buzz" Bussenschutt.
 */

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "RGBLed.h"
#include "AP_BoardLED.h"

#define SKYFALCON_R_LED_PIN 0x20 //PC0
#define SKYFALCON_G_LED_PIN 0x21 //PC1
#define SKYFALCON_B_LED_PIN 0x22 //PC2

class SkyFalcon_LED: public RGBLed {
public:
	SkyFalcon_LED();

    bool hw_init(void);
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b);
};

