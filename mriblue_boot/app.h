/* Copyright 2022 Adam Green (https://github.com/adamgreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* Constants used from C and Assembly Language code. Most relate to location of application code. */
#ifndef APP_H_
#define APP_H_


// Where the crash dump stores the CPU context in FLASH.
#define CRASH_DUMP_CONTEXT  0x2C000
// Where the crash dump stores the contents of SRAM at the time of the crash.
#define CRASH_DUMP_RAM      0x2D000 // (CRASH_DUMP_CONTEXT+0x1000)

// Where the application starts in FLASH.
#define APP_START           0x6D000 // (CRASH_DUMP_RAM+0x40000)

// Size of nRF52 FLASH pages in bytes.
#define FLASH_PAGE_SIZE     4096

// The start of RAM.
#define START_OF_RAM        0x20000000

// One byte past the end of RAM.
#define END_OF_RAM          0x20040000

// Top of the stack when bootloader starts and application will get whats left after bootloader uses what it needs.
#define TOP_OF_STACK        END_OF_RAM

#endif // APP_H_
