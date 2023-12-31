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
#ifndef BUFFER_LOG_H_
#define BUFFER_LOG_H_

int logPrintF(const char* pFormat, ...);

#endif // BUFFER_LOG_H_
