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
// Logs to an in RAM g_logBuffer circular queue. Can be dumped in GDB using the dumplog macro in my .gdbinit.
#include <stdarg.h>
#include <stdint.h>
#include <nrf_assert.h>
#include "BufferLog.h"

static char              g_logBuffer[4096+1];
volatile static uint32_t g_logWrite = 0;

int logPrintF(const char* pFormat, ...)
{
    // Reserve the last byte in the buffer to always be a NULL terminator when dumping the strings later.
    const size_t qSize = sizeof(g_logBuffer) - 1;
    // First see how many bytes are needed for this printf() call.
    va_list vaList;
    va_start(vaList, pFormat);
    int bytesNeeded = vsnprintf(NULL, 0, pFormat, vaList);
    va_end(vaList);
    ASSERT ( bytesNeeded >= 0 );

    // Just return immediately if the result of the printf() call is an empty string.
    if (bytesNeeded == 0)
    {
        return 0;
    }

    // Attempt to allocate enough bytes for this data in g_logBuffer using interlocked operations so that logging can
    // happen from multiple interrupt priorities at once.
    // - Add 1 for '\0' terminator.
    bytesNeeded += 1;
    uint32_t writeIndex = 0;
    uint32_t newIndex = 0;
    int storeFailed = 1;
    do
    {
        writeIndex = __LDREXW(&g_logWrite);
        newIndex = (writeIndex + bytesNeeded) % qSize;
        storeFailed = __STREXW(newIndex, &g_logWrite);
    } while (storeFailed);

    if (newIndex > writeIndex)
    {
        // Can write to log in 1 contiguous write so do it directly from vsnprintf().
        va_start(vaList, pFormat);
        int bytesUsed = vsnprintf(&g_logBuffer[writeIndex], bytesNeeded, pFormat, vaList);
        va_end(vaList);
        ASSERT ( bytesUsed > 0 && bytesUsed == bytesNeeded-1 );
        return bytesUsed;
    }
    else
    {
        // The write has to wrap around in g_logBuffer so place it in a temporary contiguous buffer and then copy it
        // over byte by byte.
        char buffer[bytesNeeded];
        va_start(vaList, pFormat);
        int bytesUsed = vsnprintf(buffer, bytesNeeded, pFormat, vaList) + 1;
        va_end(vaList);
        ASSERT ( bytesUsed == bytesNeeded );

        char* pCurr = &buffer[0];
        while (bytesUsed > 0)
        {
            g_logBuffer[writeIndex] = *pCurr++;
            writeIndex = (writeIndex + 1) % qSize;
            bytesUsed--;
        }
        ASSERT ( writeIndex == newIndex );
        return bytesUsed;
    }
}
