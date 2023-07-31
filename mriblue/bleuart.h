/*  Copyright (C) 2022  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* This header file describes the public API that an application uses on OS X to initialize to communicate with a
   device supporting Nordic's Bluetooth Low Energy UART service.
*/
#ifndef OSXBLE_H_
#define OSXBLE_H_

#include <stdint.h>

// Integer error codes that can be returned from most of these bleuart API functions.
#define BLEUART_ERROR_NONE              0 // Success
#define BLEUART_ERROR_CONNECT           1 // Connection to device failed.
#define BLEUART_ERROR_PARAM             2 // Invalid parameter passed to API.
#define BLEUART_ERROR_MEMORY            3 // Out of memory.
#define BLEUART_ERROR_NOT_CONNECTED     4 // No device connected.
#define BLEUART_ERROR_NO_REQUEST        5 // Not waiting for a response from a request.
#define BLEUART_ERROR_TIMEOUT           6 // Timed out waiting for response.
#define BLEUART_ERROR_EMPTY             7 // The queue was empty.
#define BLEUART_ERROR_BAD_RESPONSE      8 // Unexpected response from device.


// Initialize the BLE2UART transport.
// * It should be called from a console application's main().
// * It initializes the low level transport layer and starts a separate thread to run the developer's code.  The
//   developer provides this code in their implementation of the workerMain() function.
void bleuartInitAndRun(void);

// This is the API that the developer must provide to run their code.  It will be run on a separate thread while
// the main thread is used for handling OS X events via a NSApplicationDelegate.
void workerMain(void);

int bleuartConnect(const char* pName);
int bleuartDisconnect();
int bleuartAbortConnectionAttempt();
int bleuartStartDeviceDiscovery();
int bleuartGetDiscoveredDeviceCount(size_t* pCount);
int bleuartGetDiscoveredDeviceName(size_t deviceIndex, const char** ppDeviceName);
int bleuartStopDeviceDiscovery();
int bleuartTransmitData(const void* pData, size_t dataLength);
int bleuartReceiveData(void* pBuffer, size_t bufferSize, size_t* pActualLength);

#endif // OSXBLE_H_
