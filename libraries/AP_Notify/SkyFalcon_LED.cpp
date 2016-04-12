/*
   ToshibaLED driver
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

#include "SkyFalcon_LED.h"
#include <AP_HAL/AP_HAL.h>

#define SKYFALCON_LED_BRIGHT  1    // full brightness
#define SKYFALCON_LED_MEDIUM  1    // medium brightness
#define SKYFALCON_LED_DIM     1    // dim
#define SKYFALCON_LED_OFF     0    // off

extern const AP_HAL::HAL& hal;

SkyFalcon_LED::SkyFalcon_LED():
    RGBLed(SKYFALCON_LED_OFF, SKYFALCON_LED_BRIGHT, SKYFALCON_LED_MEDIUM, SKYFALCON_LED_DIM)
{

}
bool SkyFalcon_LED::hw_init(void){
    // setup the main LEDs as outputs
    hal.gpio->pinMode(SKYFALCON_R_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(SKYFALCON_G_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(SKYFALCON_B_LED_PIN, HAL_GPIO_OUTPUT);

    // turn all lights off
    hal.gpio->write(SKYFALCON_R_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(SKYFALCON_G_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(SKYFALCON_B_LED_PIN, HAL_GPIO_LED_OFF);
	return true;
}
bool SkyFalcon_LED::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b){

    if(r > 0){
    	hal.gpio->write(SKYFALCON_R_LED_PIN, HAL_GPIO_LED_ON);
    } else {
    	hal.gpio->write(SKYFALCON_R_LED_PIN, HAL_GPIO_LED_OFF);
    }
    if (g > 0) {
    	hal.gpio->write(SKYFALCON_G_LED_PIN, HAL_GPIO_LED_ON);
    } else {
    	hal.gpio->write(SKYFALCON_G_LED_PIN, HAL_GPIO_LED_OFF);
    }
    if (b > 0) {
    	hal.gpio->write(SKYFALCON_B_LED_PIN, HAL_GPIO_LED_ON);
    } else {
    	hal.gpio->write(SKYFALCON_B_LED_PIN, HAL_GPIO_LED_OFF);
    }

	return true;
}
