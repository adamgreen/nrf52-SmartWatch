# mriblue
![What is Yak Shaving?](https://images.prismic.io/sketchplanations/2a79fca9-374c-464f-a20e-14ae54ee8a7f_SP+726+-+Yak+shaving.png?auto=format&ixlib=react-9.0.3&h=1600.4657933042215&w=1600&q=50&dpr=2)

This was one of the yak shaving projects I got sidetracked with at the beginning of the software development for the RugRover. The name of this yak shaving project is "mriblue". It is an enhanced version of my [MRI debug monitor](https://github.com/adamgreen/mri#readme) which allows for GDB debugging of the nRF52832 Cortex-M4F microcontroller over Bluetooth Low Energy (BLE). This will allow me to sit with my feet up while programming/debugging the RugRover on the floor. No reaching down to connect and disconnect cables.

## mriblue Features
* Supports both wireless programming and debugging from the GNU Debugger, GDB:
  * It supports GDB's "load" command for programming the robot with new code.
  * It supports hardware breakpoints.
  * It supports data watchpoints.
  * It supports single stepping.
  * It supports semihosting, allowing I/O to be completed against files on my Mac.
    * This includes printf() output being sent wirelessly to the GDB console.
* It provides wireless connectivity using the built-in BLE capabilities of the nRF52 microcontroller.
* No extra debug hardware is required. It is a monitor which runs directly on the nRF52 microcontroller being debugged.
* High priority code can't be debugged but if a crash should occur in such code, a crash dump will be generated and placed in FLASH by mriblue instead. Once the dump has been saved away, the device will reset and mriblue will automatically come up in crash dump debugging mode so that GDB can use the dump to query the state of the device at the time of the crash.
* It multiplexes GDB packets and binary application data over the same BLE connection.
* It is open source.

## mriblue Monitor Commands
When using GDB you can send a few special commands to the mriblue debug monitor:
* **"monitor reset"** flags the device for reset. Follow it with a **"continue"** command to actually reboot the microcontroller. This also clears a crash dump out of the FLASH so that you can switch back to a live CPU debugging session.
* **"monitor resetbreak on|off"** enables or disables the automatic halting of the program in its Reset_Handler during bootup. This is persisted across power cycles in FLASH.
* **"monitor showfault"** will display extra information to the GDB console about the nature of a fault (using the Cortex-M fault status registers).
```console
(gdb) monitor showfault

**Hard Fault**
  Status Register: 0x40000000
    Forced
**Bus Fault**
  Status Register: 0x82
    Fault Address: 0xffffffff
    Precise Data Access
```

## mriblue vs OpenOCD
![Typical GDB Debugger Setup](../photos/20220510-01.jpg)</br>
If you have ever done any GDB debugging of a microcontroller in the past then you have probably used a setup like that shown above:
* You run a remote stub like OpenOCD on the same computer as you run GDB.
* GDB uses TCP/IP to communicate with OpenOCD.
* It is OpenOCD which understands GDB's remote debug protocol and translates it into the necessary debug probe commands.
* OpenOCD communicates with the debug probe using an external interface such as USB.
* The debug probe will then use JTAG or SWD to communicate with the debug access port of the microcontroller.

![mriblue GDB Debugger Setup](../photos/20220510-02.jpg)</br>
The setup for my mriblue project, shown above, shares some similarities with the OpenOCD setup but also differs in a few key areas:
* It too has a program (only available for macOS) which runs on the same computer as GDB:
  * It is called [mriblue](../mriblue/).
  * GDB also communicates with it over TCP/IP, on port 3333 by default.
  * ~~Unlike OpenOCD, it doesn't have any knowledge of the GDB remote debug protocol. It is just a bridge between TCP/IP and BLE. It could be used to bridge serial traffic from BLE to TCP/IP for any application except that it is explicitly written to find and connect to devices running the mriblue_boot bootloader.~~ It only knows enough about the GDB packet format to send such packets to GDB's socket, on port 3333. It sends any binary data found outside of these GDB packets to another socket, on port 3334 by default. Applications can then be written for the Mac to connect to this socket to visualize and/or log the data. The format of this data is more compact than that found within GDB packets (usually 50% smaller).
* Unlike a typical OpenOCD setup, mriblue requires no special debug hardware. The code which implements the GDB remote debug protocol and talks with the debug access port on the Cortex-M4F core is all contained in the [mriblue_boot bootloader](../mriblue_boot/) which runs directly on the nRF52832 microcontroller alongside the robot's firmware. The next couple of sections will give an overview of how it accomplishes this by sharing FLASH memory and CPU cycles via the NVIC's prioritized interrupts.

## mriblue_boot FLASH Memory Layout
![FLASH Memory Layout](../photos/20220510-03.jpg)</br>
The mriblue_boot bootloader resides in FLASH alongside the BLE Stack (Nordic SoftDevice version 132) and the robot code:
* The SoftDevice resides at the beginning of FLASH, at address 0x0000000, like usual.
* Normally the Robot code would come right after the SoftDevice to start at address 0x0001F000. This is where the mriblue_boot now resides so that the SoftDevice will forward exceptions and interrupts to it automatically without any extra configuration.
* The 68kB of FLASH following the bootloader area is reserved for storing of a crash encountered in high priority code. It is large enough to store:
  * 64kB copy of the entire SRAM at the time of the crash. This makes the stack, globals, and heap all available to the debugger when investigating the crash dump.
  * Another 4kB page of FLASH to store the context (CPU registers, MRI debug monitor state, etc) from the time of the crash.
* The robot's main code can then start at 0x0003C000. Now that the robot code and mriblue code live in their own separate areas of memory, it is possible for mriblue_boot to erase the robot code and it itself still function correcly. This means it can be used to load new code into the device as erasing pages is the first step requested when GDB handles a "load" command.

The [interrupt vector table](https://github.com/adamgreen/rugrover/blob/master/mriblue_boot/gcc_startup_nrf52.S#L422) found in the mriblue_boot bootloader is written so that by default every entry in the table will just jump to the corresponding entry in the robot code's interrupt vector table. These defaults are weak implementations so any exception or interrupts that are of interest to mriblue_boot itself will be implemented as strong functions which will override and replace these forwarding entries.

## mriblue_boot NVIC Interrupt Priorities
![Interrupt Priorities](../photos/20220510-04.jpg)</br>
The previous section showed how mriblue_boot could co-exist with the BLE SoftDevice and robot code in FLASH. Now let's look at how it can share the CPU cycles itself with the robot code and BLE stack. It uses the 8 interrupt priority levels on the nRF52 to make it possible for all these things to run on the same CPU. When MRI has halted the robot code for debugging, it can’t stop everything. It needs to keep the BLE stack running so that it can be used to keep communications open between GDB and itself:
* On Cortex-M microcontrollers, the lower the priority level number, the higher the actual priority.
* In the above diagram, you can see the priority levels that are used by the different components of the system. The more darkly shaded levels (levels 0 to 6) can't be debugged directly by mriblue as they are too high priority. The non-shaded levels (level 7 and Thread/Main) can be halted for the purpose of debugging.
* The BLE stack uses the 2 highest priority levels: level 0 and level 1. It also uses levels 4 and 5.
* mriblue_boot runs at priority level 6 so any time the BLE stack needs to do something to keep up its communication with GDB, it is free to interrupt mriblue_boot and do its work.
* Our robot code can run code at interrupt levels 2, 3, and 7.
  * Levels 2 and 3 are too high priority for MRI to debug. If the robot code running at these 2 levels should cause a fault, a crash dump will instead be written to the reserved area of the FLASH, the microcontroller reset, and then mriblue_blue starts up in a special mode which allows GDB to debug the crash dump rather than the live CPU state.
  * Level 7 ISRs in the robot code can be debugged since they will be halted automatically by the NVIC when the mriblue_boot debug monitor is entered at priority level 6.
  * It can also debug all of the non-interrupt robot code. This is anything you run from your main() function.
