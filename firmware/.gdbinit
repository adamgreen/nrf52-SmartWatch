# GDB initialization script with commands to make debugging Bangle.js2 SmartWatch eadier.
#
# Commands of Interest
#   heapsize - Displays the current heap size.
#   heapwalk - Walks the heap and dumps each chunk encountered.
#
# Issuing "help user-defined" command from within GDB will list the commands added by this script.
#
# Initially created by Adam Green - July 2023.



# Command to erase all of the FLASH and UICR on the nRF52 microcontroller.
define eraseall
    printf "Erasing all of the FLASH and UICR on nRF52 microcontroller\n"
    set mem inaccessible-by-default off
    # Enable FLASH erase.
    set var *(int*)(0x4001E504)=2
    # Issue ERASEALL command.
    set var *(int*)(0x4001E50c)=1
end

document eraseall
Erases all of the FLASH and UICR on a nRF52 microcontroller.
end




# Command to dump the current amount of space allocated to the heap.
define heapsize
    set var $heap_base=(((unsigned int)&__bss_end__+7)&~7)
    printf "heap size: %u bytes\n", ('_sbrk_r::heap' - $heap_base)
end

document heapsize
Displays the current heap size.
end




# Command to dump the heap allocations (in-use and free).
define heapwalk
    set var $chunk_curr=(((unsigned int)&__bss_end__+7)&~7)
    set var $chunk_number=1
    set var $used_bytes=(unsigned int)0
    set var $free_bytes=(unsigned int)0
    if (sizeof(struct _reent) == 96)
        # newlib-nano library in use.
        set var $free_curr=(unsigned int)__malloc_free_list
        while ($chunk_curr < '_sbrk_r::heap')
            set var $chunk_size=*(unsigned int*)$chunk_curr
            set var $chunk_next=$chunk_curr + $chunk_size
            if ($chunk_curr == $free_curr)
                set var $chunk_free=1
                set var $free_curr=*(unsigned int*)($free_curr + 4)
            else
                set var $chunk_free=0
            end
            set var $chunk_orig=$chunk_curr + 4
            set var $chunk_curr=($chunk_orig + 7) & ~7
            set var $chunk_size=$chunk_size - 8
            printf "Chunk: %u  Address: 0x%08X  Size: %u  ", $chunk_number, $chunk_curr, $chunk_size
            if ($chunk_free)
                printf "FREE CHUNK"
                set var $free_bytes+=$chunk_size
            else
                set var $used_bytes+=$chunk_size
            end
            printf "\n"
            set var $chunk_curr=$chunk_next
            set var $chunk_number=$chunk_number+1
        end
    else
        # full newlib library in use.
        while ($chunk_curr < '_sbrk_r::heap')
            set var $chunk_size=*(unsigned int*)($chunk_curr + 4)
            set var $chunk_size&=~1
            set var $chunk_next=$chunk_curr + $chunk_size
            set var $chunk_inuse=(*(unsigned int*)($chunk_next + 4)) & 1

            # A 0-byte chunk at the beginning of the heap is the initial state before any allocs occur.
            if ($chunk_size == 0)
                loop_break
            end

            # The actual data starts past the 8 byte header.
            set var $chunk_orig=$chunk_curr + 8
            # The actual data is 4 bytes smaller than the total chunk since it can use the first word of the next chunk
            # as well since that is its footer.
            set var $chunk_size=$chunk_size - 4
            printf "Chunk: %u  Address: 0x%08X  Size: %u  ", $chunk_number, $chunk_orig, $chunk_size
            if ($chunk_inuse == 0)
                printf "FREE CHUNK"
                set var $free_bytes+=$chunk_size
            else
                set var $used_bytes+=$chunk_size
            end
            printf "\n"
            set var $chunk_curr=$chunk_next
            set var $chunk_number=$chunk_number+1
        end
    end
    printf "  Used bytes: %u\n", $used_bytes
    printf "  Free bytes: %u\n", $free_bytes
end

document heapwalk
Walks the heap and dumps each chunk encountered.
end




# Settings that I just find work the best for embedded/remote target debugging:
#   No need to ask if an invalid symbol name should be postponed until something gets loaded later...it won't!
set breakpoint pending off
#   Defaults to UTF-8 and some garbage strings cause GDB to hang when it attempts to walk the string.
set target-charset ASCII
#   Just makes structures look a bit nicer.
set print pretty on
#   Can't access peripheral registers and some regions of RAM without this turned off.
set mem inaccessible-by-default off
#   Show disassembly on each stop. Makes it easier to switch to using stepi/nexti when debugging optimized code.
# set disassemble-next-line on
#   Never just set ambiguous breakpoint at multiple locations. Ask which to actually use. We have limited breakpoints
#   on embedded targets.
set multiple-symbols ask
#   Save command history on exit.
set history save on
