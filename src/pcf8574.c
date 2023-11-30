/*
 * pcf8574.c:
 *	Extend wiringPi with the PCF8574 I2C GPIO expander chip
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

#include <stdio.h>
#include <pthread.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <pcf8574.h>


/*
 * myPinMode:
 *	The PCF8574 is an odd chip - the pins are effectively bi-directional,
 *	however the pins should be drven high when used as an input pin...
 *	So, we're effectively copying digitalWrite...
 *********************************************************************************
 */

static bool pcfPinMode (struct wiringPiNodeStruct *node, int pin, int mode)
{
  int bit, old ;

  bit  = 1 << ((pin - node->pinBase) & 7) ;

  old = node->data2 ;
  if (mode == OUTPUT)
    old &= (~bit) ;	// Write bit to 0
  else
    old |=   bit ;	// Write bit to 1

  if (wiringPiI2CWrite (node->fd, old) < 0) {
    return false;
  }
  node->data2 = old ;
  return true;
}



/*
 * myDigitalWrite:
 *********************************************************************************
 */

static bool pcfDigitalWrite (struct wiringPiNodeStruct *node, int pin, int value)
{
  int bit, old ;

  bit  = 1 << ((pin - node->pinBase) & 7) ;

  old = node->data2 ;
  if (value == LOW)
    old &= (~bit) ;
  else
    old |=   bit ;

  if (wiringPiI2CWrite (node->fd, old) < 0) {
    return false;
  }
  node->data2 = old ;
  return true;
}


/*
 * myDigitalRead:
 *********************************************************************************
 */

static bool pcfDigitalRead (struct wiringPiNodeStruct *node, int pin, int *value)
{
  int mask, val ;

  mask  = 1 << ((pin - node->pinBase) & 7) ;
  val = wiringPiI2CRead (node->fd) ;

  if (val < 0) {
    return false;
  }

  if ((val & mask) == 0)
    *value = LOW ;
  else 
    *value = HIGH ;

  return true;
}


/*
 * pcf8574Setup:
 *	Create a new instance of a PCF8574 I2C GPIO interface. We know it
 *	has 8 pins, so all we need to know here is the I2C address and the
 *	user-defined pin base.
 *********************************************************************************
 */

bool pcf8574Setup (const unsigned i2cBusId, const int i2cAddress, const int pinBase)
{
  int                         fd ;
  struct wiringPiNodeStruct   *node ;

  if (!wiringPiI2CSetup (i2cBusId, i2cAddress, &fd))
    return false;

  node = wiringPiNewNode (pinBase, 8) ;
  if (node == NULL) {
    return false;
  }

  node->fd           = fd ;
  node->pinMode      = pcfPinMode ;
  node->digitalRead  = pcfDigitalRead ;
  node->digitalWrite = pcfDigitalWrite ;
  node->data2        = wiringPiI2CRead (fd) ;
  if (node->data2 < 0) {
    return false;
  }

  return true;
}
