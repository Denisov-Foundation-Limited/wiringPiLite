/*
 * 23017.h:
 *	Extend wiringPi with the MCP 23017 I2C GPIO expander chip
 *	Copyright (c) 2013 Gordon Henderson
 *	Modified (c) 2023 Sergey Denisov DenisovS21@gmail.com
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

extern bool mcp23017Setup (const unsigned i2cBusId, const int i2cAddress, const int pinBase);

#ifdef __cplusplus
}
#endif
