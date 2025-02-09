#pragma once

#include "../../hal/badhal.h"
/*
    PK1,PA8,PE6,PI8 -> LCD Data transmission
    PE3             -> Register Select
    PH15            -> Enable pin
    PB4             -> Backlight control

    Parts of this are "borrowed" from Arduino's LiquidCrystal library:

    Copyright © 2006-2008 Hans-Christoph Steiner. All rights reserved. Copyright (c) 2010 Arduino LLC. All right reserved.

    This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser
    General Public License as published by the Free Software Foundation; either version 2.1 of the License, or
    (at your option) any later version.

    This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
    License for more details.

    You should have received a copy of the GNU Lesser General Public License along with this library; if not,
    write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

*/
#define HD_D0 GPIOK, 1
#define HD_D1 GPIOA, 8
#define HD_D2 GPIOE, 6
#define HD_D3 GPIOI, 8
#define HD_RS GPIOE, 3
#define HD_EN GPIOH, 15
#define HD_BC GPIOB, 4
#define HD_OUTPUT None, Medium, PushPull

#define HD_CLEARDISPLAY 0x01
#define HD_RETURNHOME 0x02
#define HD_ENTRYMODESET 0x04
#define HD_DISPLAYCONTROL 0x08
#define HD_CURSORSHIFT 0x10
#define HD_FUNCTIONSET 0x20
#define HD_SETCGRAMADDR 0x40
#define HD_SETDDRAMADDR 0x80

#define HD_ENTRYRIGHT 0x00
#define HD_ENTRYLEFT 0x02
#define HD_ENTRYSHIFTINCREMENT 0x01
#define HD_ENTRYSHIFTDECREMENT 0x00

#define HD_DISPLAYON 0x04
#define HD_DISPLAYOFF 0x00
#define HD_CURSORON 0x02
#define HD_CURSOROFF 0x00
#define HD_BLINKON 0x01
#define HD_BLINKOFF 0x00

#define HD_DISPLAYMOVE 0x08
#define HD_CURSORMOVE 0x00
#define HD_MOVERIGHT 0x04
#define HD_MOVELEFT 0x00

#define HD_8BITMODE 0x10
#define HD_4BITMODE 0x00
#define HD_2LINE 0x08
#define HD_1LINE 0x00
#define HD_5x10DOTS 0x04
#define HD_5x8DOTS 0x00

void hd44780_init();
void hd44790_puts(const char *str);