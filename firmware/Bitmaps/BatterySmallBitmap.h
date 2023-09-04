/*  Copyright (C) 2023  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Smaller battery icon used for BLE heart rate monitor.
static const uint32_t g_batterySmallBitmapWidth = 31;
static const uint32_t g_batterySmallBitmapHeight = 13;
static const uint8_t g_batterySmallBitmap[] =
{
    0b00111111, 0b11111111, 0b11111111, 0b11000000,
    0b01000000, 0b00000000, 0b00000000, 0b00100000,
    0b10000000, 0b00000000, 0b00000000, 0b00010000,
    0b10000000, 0b00000000, 0b00000000, 0b00010000,
    0b10000000, 0b00000000, 0b00000000, 0b00010000,
    0b10000000, 0b00000000, 0b00000000, 0b00010110,
    0b10000000, 0b00000000, 0b00000000, 0b00010110,
    0b10000000, 0b00000000, 0b00000000, 0b00010110,
    0b10000000, 0b00000000, 0b00000000, 0b00010000,
    0b10000000, 0b00000000, 0b00000000, 0b00010000,
    0b10000000, 0b00000000, 0b00000000, 0b00010000,
    0b01000000, 0b00000000, 0b00000000, 0b00100000,
    0b00111111, 0b11111111, 0b11111111, 0b11000000,
};

// Width of the main rectangle in the battery icon, not including the tab at the right.
static const uint32_t g_batterySmallIconRectWidth = g_batterySmallBitmapWidth - 3;
// The radius used for the battery icon rounded corners.
static const uint32_t g_batterySmallIconRadius = 3;
