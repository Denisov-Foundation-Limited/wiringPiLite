/*
 * lcd.h:
 *	Text-based LCD driver.
 *	This is designed to drive the parallel interface LCD drivers
 *	based in the Hitachi HD44780U controller and compatables.
 *
 * Copyright (c) 2012 Gordon Henderson.
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

#define	MAX_LCDS	8

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

extern bool lcdHome        (const int fd) ;
extern bool lcdClear       (const int fd) ;
extern bool lcdDisplay     (const int fd, int state) ;
extern bool lcdCursor      (const int fd, int state) ;
extern bool lcdCursorBlink (const int fd, int state) ;
extern bool lcdSendCommand (const int fd, unsigned char command) ;
extern bool lcdPosition    (const int fd, int x, int y) ;
extern bool lcdCharDef     (const int fd, int index, unsigned char data [8]) ;
extern bool lcdPutchar     (const int fd, unsigned char data) ;
extern bool lcdPuts        (const int fd, const char *string) ;
extern bool lcdPrintf      (const int fd, const char *message, ...) ;

extern int  lcdInit (const int rows, const int cols, const int bits,
	const int rs, const int strb,
	const int d0, const int d1, const int d2, const int d3, const int d4,
	const int d5, const int d6, const int d7) ;

#ifdef __cplusplus
}
#endif
