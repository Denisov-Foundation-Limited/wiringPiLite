/*
 * mcp23017.c:
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

#include <stdio.h>
#include <pthread.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <mcp23x0817.h>

#include <mcp23017.h>


/*
 * myPinMode:
 *********************************************************************************
 */

static bool mcpPinMode (struct wiringPiNodeStruct *node, int pin, int mode)
{
  int mask, old, reg ;

  pin -= node->pinBase ;

  if (pin < 8)		// Bank A
    reg  = MCP23x17_IODIRA ;
  else
  {
    reg  = MCP23x17_IODIRB ;
    pin &= 0x07 ;
  }

  mask = 1 << pin ;
  old  = wiringPiI2CReadReg8 (node->fd, reg) ;

  if (old < 0) {
    return false;
  }

  if (mode == OUTPUT)
    old &= (~mask) ;
  else
    old |=   mask ;

  if (wiringPiI2CWriteReg8 (node->fd, reg, old) < 0) {
    return false;
  }
  return true;
}

static bool mcpPinModeRead (struct wiringPiNodeStruct *node, int pin, int *mode)
{
  int mask, old, reg ;

  pin -= node->pinBase ;

  if (pin < 8)		// Bank A
    reg  = MCP23x17_IODIRA ;
  else
  {
    reg  = MCP23x17_IODIRB ;
    pin &= 0x07 ;
  }

  mask = 1 << pin ;
  old  = wiringPiI2CReadReg8 (node->fd, reg) ;

  if (old < 0) {
    return false;
  }

  if (old & mask) {
    *mode = INPUT;
  } else {
    *mode = OUTPUT;
  }

  return true;
}

/*
 * myPullUpDnControl:
 *********************************************************************************
 */

static bool mcpPullUpDnControl (struct wiringPiNodeStruct *node, int pin, int mode)
{
  int mask, old, reg ;

  pin -= node->pinBase ;

  if (pin < 8)		// Bank A
    reg  = MCP23x17_GPPUA ;
  else
  {
    reg  = MCP23x17_GPPUB ;
    pin &= 0x07 ;
  }

  mask = 1 << pin ;
  old  = wiringPiI2CReadReg8 (node->fd, reg) ;

  if (old < 0) {
    return false;
  }

  if (mode == PUD_UP)
    old |=   mask ;
  else
    old &= (~mask) ;

  if (wiringPiI2CWriteReg8 (node->fd, reg, old) < 0) {
    return false;
  }

  return true;
}

static bool mcpPullUpDnRead (struct wiringPiNodeStruct *node, int pin, int *mode)
{
  int mask, old, reg ;

  pin -= node->pinBase ;

  if (pin < 8)		// Bank A
    reg  = MCP23x17_GPPUA ;
  else
  {
    reg  = MCP23x17_GPPUB ;
    pin &= 0x07 ;
  }

  mask = 1 << pin ;
  old  = wiringPiI2CReadReg8 (node->fd, reg) ;

  if (old < 0) {
    return false;
  }

  if (old & mask) {
    *mode = PUD_UP;
  } else {
    *mode = PUD_OFF;
  }

  return true;
}


/*
 * myDigitalWrite:
 *********************************************************************************
 */

static bool mcpDigitalWrite (struct wiringPiNodeStruct *node, int pin, int value)
{
  int bit, old ;

  pin -= node->pinBase ;	// Pin now 0-15

  bit = 1 << (pin & 7) ;

  if (pin < 8)			// Bank A
  {
    old = node->data2 ;

    if (value == LOW)
      old &= (~bit) ;
    else
      old |=   bit ;

    if (wiringPiI2CWriteReg8 (node->fd, MCP23x17_GPIOA, old) < 0) {
      return false;
    }
    node->data2 = old ;
  }
  else				// Bank B
  {
    old = node->data3 ;

    if (value == LOW)
      old &= (~bit) ;
    else
      old |=   bit ;

    if (wiringPiI2CWriteReg8 (node->fd, MCP23x17_GPIOB, old) < 0) {
      return false;
    }
    node->data3 = old ;
  }
  return true;
}


/*
 * myDigitalRead:
 *********************************************************************************
 */

static bool mcpDigitalRead (struct wiringPiNodeStruct *node, int pin, int *value)
{
  int mask, val, gpio ;

  pin -= node->pinBase ;

  if (pin < 8)		// Bank A
    gpio  = MCP23x17_GPIOA ;
  else
  {
    gpio  = MCP23x17_GPIOB ;
    pin  &= 0x07 ;
  }

  mask  = 1 << pin ;
  val = wiringPiI2CReadReg8 (node->fd, gpio) ;

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
 * mcp23017Setup:
 *	Create a new instance of an MCP23017 I2C GPIO interface. We know it
 *	has 16 pins, so all we need to know here is the I2C address and the
 *	user-defined pin base.
 *********************************************************************************
 */

bool mcp23017Setup (const unsigned i2cBusId, const int i2cAddress, const int pinBase)
{
  int                         fd ;
  struct wiringPiNodeStruct   *node ;

  if (!wiringPiI2CSetup (i2cBusId, i2cAddress, &fd))
    return false ;

  if (wiringPiI2CWriteReg8 (fd, MCP23x17_IOCON, IOCON_INIT) < 0) {
    return false;
  }

  node = wiringPiNewNode (pinBase, 16) ;
  if (node == NULL) {
    return false;
  }

  node->fd              = fd ;
  node->pinMode         = mcpPinMode ;
  node->pullUpDnControl = mcpPullUpDnControl ;
  node->digitalRead     = mcpDigitalRead ;
  node->digitalWrite    = mcpDigitalWrite ;
  node->pinModeRead     = mcpPinModeRead;
  node->pullUpDnRead    = mcpPullUpDnRead;
  node->data2           = wiringPiI2CReadReg8 (fd, MCP23x17_OLATA) ;
  if (node->data2 < 0) {
    return false;
  }
  node->data3           = wiringPiI2CReadReg8 (fd, MCP23x17_OLATB) ;
  if (node->data3 < 0) {
    return false;
  }

  return true;
}
