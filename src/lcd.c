/*
 * lcd.c:
 *	Text-based LCD driver.
 *	This is designed to drive the parallel interface LCD drivers
 *	based in the Hitachi HD44780U controller and compatables.
 *
 * Copyright (c) 2012 Gordon Henderson.
 *	Modified (c) 2023 Sergey Denisov DenisovS21@gmail.com
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <wiringPi.h>

#include <lcd.h>


// HD44780U Commands

#define	LCD_CLEAR	0x01
#define	LCD_HOME	0x02
#define	LCD_ENTRY	0x04
#define	LCD_CTRL	0x08
#define	LCD_CDSHIFT	0x10
#define	LCD_FUNC	0x20
#define	LCD_CGRAM	0x40
#define	LCD_DGRAM	0x80

// Bits in the entry register

#define	LCD_ENTRY_SH		0x01
#define	LCD_ENTRY_ID		0x02

// Bits in the control register

#define	LCD_BLINK_CTRL		0x01
#define	LCD_CURSOR_CTRL		0x02
#define	LCD_DISPLAY_CTRL	0x04

// Bits in the function register

#define	LCD_FUNC_F	0x04
#define	LCD_FUNC_N	0x08
#define	LCD_FUNC_DL	0x10

#define	LCD_CDSHIFT_RL	0x04

struct lcdDataStruct
{
  int bits, rows, cols ;
  int rsPin, strbPin ;
  int dataPins [8] ;
  int cx, cy ;
} ;

struct lcdDataStruct *lcds [MAX_LCDS] ;

static int lcdControl ;

// Row offsets

static const int rowOff [4] = { 0x00, 0x40, 0x14, 0x54 } ;


/*
 * strobe:
 *	Toggle the strobe (Really the "E") pin to the device.
 *	According to the docs, data is latched on the falling edge.
 *********************************************************************************
 */

static bool strobe (const struct lcdDataStruct *lcd)
{

// Note timing changes for new version of delayMicroseconds ()

  if (!digitalWrite (lcd->strbPin, 1)) {
    return false;
  }
  delayMicroseconds (50) ;
  if (!digitalWrite (lcd->strbPin, 0)) {
    return false;
  }
  delayMicroseconds (50) ;

  return true;
}


/*
 * sentDataCmd:
 *	Send an data or command byte to the display.
 *********************************************************************************
 */

static bool sendDataCmd (const struct lcdDataStruct *lcd, unsigned char data)
{
  register unsigned char myData = data ;
  unsigned char          i, d4 ;

  if (lcd->bits == 4)
  {
    d4 = (myData >> 4) & 0x0F;
    for (i = 0 ; i < 4 ; ++i)
    {
      if (!digitalWrite (lcd->dataPins [i], (d4 & 1))) {
        return false;
      }
      d4 >>= 1 ;
    }
    if (!strobe (lcd)) {
      return false;
    }

    d4 = myData & 0x0F ;
    for (i = 0 ; i < 4 ; ++i)
    {
      if (!digitalWrite (lcd->dataPins [i], (d4 & 1))) {
        return false;
      }
      d4 >>= 1 ;
    }
  }
  else
  {
    for (i = 0 ; i < 8 ; ++i)
    {
      if (!digitalWrite (lcd->dataPins [i], (myData & 1))) {
        return false;
      }
      myData >>= 1 ;
    }
  }
  return strobe (lcd);
}


/*
 * putCommand:
 *	Send a command byte to the display
 *********************************************************************************
 */

static bool putCommand (const struct lcdDataStruct *lcd, unsigned char command)
{
  if (!digitalWrite (lcd->rsPin,   0)) {
    return false;
  }
  if (!sendDataCmd  (lcd, command)) {
    return false;
  }
  delay (2) ;
  return true;
}

static bool put4Command (const struct lcdDataStruct *lcd, unsigned char command)
{
  register unsigned char myCommand = command ;
  register unsigned char i ;

  if (!digitalWrite (lcd->rsPin,   0)) {
    return false;
  }

  for (i = 0 ; i < 4 ; ++i)
  {
    if (!digitalWrite (lcd->dataPins [i], (myCommand & 1))) {
      return false;
    }
    myCommand >>= 1 ;
  }
  return strobe (lcd);
}


/*
 *********************************************************************************
 * User Callable code below here
 *********************************************************************************
 */

/*
 * lcdHome: lcdClear:
 *	Home the cursor or clear the screen.
 *********************************************************************************
 */

bool lcdHome (const int fd)
{
  struct lcdDataStruct *lcd = lcds [fd] ;

  if (!putCommand (lcd, LCD_HOME)) {
    return false;
  }
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
  return true;
}

bool lcdClear (const int fd)
{
  struct lcdDataStruct *lcd = lcds [fd] ;

  if (!putCommand (lcd, LCD_CLEAR)) {
    return false;
  }
  if (!putCommand (lcd, LCD_HOME)) {
    return false;
  }
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
  return true;
}


/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 *	Turn the display, cursor, cursor blinking on/off
 *********************************************************************************
 */

bool lcdDisplay (const int fd, int state)
{
  struct lcdDataStruct *lcd = lcds [fd] ;

  if (state)
    lcdControl |=  LCD_DISPLAY_CTRL ;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL ;

  return putCommand (lcd, LCD_CTRL | lcdControl) ; 
}

bool lcdCursor (const int fd, int state)
{
  struct lcdDataStruct *lcd = lcds [fd] ;

  if (state)
    lcdControl |=  LCD_CURSOR_CTRL ;
  else
    lcdControl &= ~LCD_CURSOR_CTRL ;

  return putCommand (lcd, LCD_CTRL | lcdControl) ; 
}

bool lcdCursorBlink (const int fd, int state)
{
  struct lcdDataStruct *lcd = lcds [fd] ;

  if (state)
    lcdControl |=  LCD_BLINK_CTRL ;
  else
    lcdControl &= ~LCD_BLINK_CTRL ;

  return putCommand (lcd, LCD_CTRL | lcdControl) ; 
}


/*
 * lcdSendCommand:
 *	Send any arbitary command to the display
 *********************************************************************************
 */

bool lcdSendCommand (const int fd, unsigned char command)
{
  struct lcdDataStruct *lcd = lcds [fd] ;
  return putCommand (lcd, command) ;
}


/*
 * lcdPosition:
 *	Update the position of the cursor on the display.
 *	Ignore invalid locations.
 *********************************************************************************
 */

bool lcdPosition (const int fd, int x, int y)
{
  struct lcdDataStruct *lcd = lcds [fd] ;

  if ((x > lcd->cols) || (x < 0))
    return false;
  if ((y > lcd->rows) || (y < 0))
    return false;

  if (!putCommand (lcd, x + (LCD_DGRAM | rowOff [y]))) {
    return false;
  }

  lcd->cx = x ;
  lcd->cy = y ;
  return true;
}


/*
 * lcdCharDef:
 *	Defines a new character in the CGRAM
 *********************************************************************************
 */

bool lcdCharDef (const int fd, int index, unsigned char data [8])
{
  struct lcdDataStruct *lcd = lcds [fd] ;
  int i ;

  if (!putCommand (lcd, LCD_CGRAM | ((index & 7) << 3))) {
    return false;
  }

  if (!digitalWrite (lcd->rsPin, 1)) {
    return false;
  }
  for (i = 0 ; i < 8 ; ++i) {
    if (!sendDataCmd (lcd, data [i])) {
      return false;
    }
  }
  return true;
}


/*
 * lcdPutchar:
 *	Send a data byte to be displayed on the display. We implement a very
 *	simple terminal here - with line wrapping, but no scrolling. Yet.
 *********************************************************************************
 */

bool lcdPutchar (const int fd, unsigned char data)
{
  struct lcdDataStruct *lcd = lcds [fd] ;

  if (!digitalWrite (lcd->rsPin, 1)) {
    return false;
  }
  if (!sendDataCmd  (lcd, data)) {
    return false;
  }

  if (++lcd->cx == lcd->cols)
  {
    lcd->cx = 0 ;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0 ;
    
    if (!putCommand (lcd, lcd->cx + (LCD_DGRAM | rowOff [lcd->cy]))) {
      return false;
    }
  }
  return true;
}


/*
 * lcdPuts:
 *	Send a string to be displayed on the display
 *********************************************************************************
 */

bool lcdPuts (const int fd, const char *string)
{
  while (*string) {
    if (!lcdPutchar (fd, *string++)) {
      return false;
    }
  }
  return true;
}


/*
 * lcdPrintf:
 *	Printf to an LCD display
 *********************************************************************************
 */

bool lcdPrintf (const int fd, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  return lcdPuts (fd, buffer) ;
}


/*
 * lcdInit:
 *	Take a lot of parameters and initialise the LCD, and return a handle to
 *	that LCD, or -1 if any error.
 *********************************************************************************
 */

int lcdInit (const int rows, const int cols, const int bits,
	const int rs, const int strb,
	const int d0, const int d1, const int d2, const int d3, const int d4,
	const int d5, const int d6, const int d7)
{
  static int initialised = 0 ;

  unsigned char func ;
  int i ;
  int lcdFd = -1 ;
  struct lcdDataStruct *lcd ;

  if (initialised == 0)
  {
    initialised = 1 ;
    for (i = 0 ; i < MAX_LCDS ; ++i)
      lcds [i] = NULL ;
  }

// Simple sanity checks

  if (! ((bits == 4) || (bits == 8)))
    return -1 ;

  if ((rows < 0) || (rows > 20))
    return -1 ;

  if ((cols < 0) || (cols > 20))
    return -1 ;

// Create a new LCD:

  for (i = 0 ; i < MAX_LCDS ; ++i)
  {
    if (lcds [i] == NULL)
    {
      lcdFd = i ;
      break ;
    }
  }

  if (lcdFd == -1)
    return -1 ;

  lcd = (struct lcdDataStruct *)malloc (sizeof (struct lcdDataStruct)) ;
  if (lcd == NULL)
    return -1 ;

  lcd->rsPin   = rs ;
  lcd->strbPin = strb ;
  lcd->bits    = 8 ;		// For now - we'll set it properly later.
  lcd->rows    = rows ;
  lcd->cols    = cols ;
  lcd->cx      = 0 ;
  lcd->cy      = 0 ;

  lcd->dataPins [0] = d0 ;
  lcd->dataPins [1] = d1 ;
  lcd->dataPins [2] = d2 ;
  lcd->dataPins [3] = d3 ;
  lcd->dataPins [4] = d4 ;
  lcd->dataPins [5] = d5 ;
  lcd->dataPins [6] = d6 ;
  lcd->dataPins [7] = d7 ;

  lcds [lcdFd] = lcd ;

  digitalWrite (lcd->rsPin,   0) ; pinMode (lcd->rsPin,   OUTPUT) ;
  digitalWrite (lcd->strbPin, 0) ; pinMode (lcd->strbPin, OUTPUT) ;

  for (i = 0 ; i < bits ; ++i)
  {
    digitalWrite (lcd->dataPins [i], 0) ;
    pinMode      (lcd->dataPins [i], OUTPUT) ;
  }
  delay (35) ; // mS


// 4-bit mode?
//	OK. This is a PIG and it's not at all obvious from the documentation I had,
//	so I guess some others have worked through either with better documentation
//	or more trial and error... Anyway here goes:
//
//	It seems that the controller needs to see the FUNC command at least 3 times
//	consecutively - in 8-bit mode. If you're only using 8-bit mode, then it appears
//	that you can get away with one func-set, however I'd not rely on it...
//
//	So to set 4-bit mode, you need to send the commands one nibble at a time,
//	the same three times, but send the command to set it into 8-bit mode those
//	three times, then send a final 4th command to set it into 4-bit mode, and only
//	then can you flip the switch for the rest of the library to work in 4-bit
//	mode which sends the commands as 2 x 4-bit values.

  if (bits == 4)
  {
    func = LCD_FUNC | LCD_FUNC_DL ;			// Set 8-bit mode 3 times
    put4Command (lcd, func >> 4) ; delay (35) ;
    put4Command (lcd, func >> 4) ; delay (35) ;
    put4Command (lcd, func >> 4) ; delay (35) ;
    func = LCD_FUNC ;					// 4th set: 4-bit mode
    put4Command (lcd, func >> 4) ; delay (35) ;
    lcd->bits = 4 ;
  }
  else
  {
    func = LCD_FUNC | LCD_FUNC_DL ;
    putCommand  (lcd, func     ) ; delay (35) ;
    putCommand  (lcd, func     ) ; delay (35) ;
    putCommand  (lcd, func     ) ; delay (35) ;
  }

  if (lcd->rows > 1)
  {
    func |= LCD_FUNC_N ;
    putCommand (lcd, func) ; delay (35) ;
  }

// Rest of the initialisation sequence

  lcdDisplay     (lcdFd, true) ;
  lcdCursor      (lcdFd, false) ;
  lcdCursorBlink (lcdFd, false) ;
  lcdClear       (lcdFd) ;

  putCommand (lcd, LCD_ENTRY   | LCD_ENTRY_ID) ;
  putCommand (lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL) ;

  return lcdFd ;
}
