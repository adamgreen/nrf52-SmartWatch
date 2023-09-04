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
// Template based thread safe circular buffer that support overwrite on full queue.
#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <nrf_assert.h>

template <class T, size_t COUNT, bool OVERWRITE>
class CircularBuffer
{
    public:
        CircularBuffer()
        {
            m_indices = 0;
            ASSERT ( COUNT <= (1U << BITS_PER_INDEX) );
        }

        bool isEmpty()
        {
            return decodeCount(m_indices) == 0;
        }

        bool isFull()
        {
            return decodeCount(m_indices) == COUNT;
        }

        bool write(T& element)
        {
            uint32_t index = 0;
            int storeFailed = 1;
            do
            {
                uint32_t indices = __LDREXW(&m_indices);
                uint32_t count = decodeCount(indices);
                uint32_t writeIndex = decodeWriteIndex(indices);
                uint32_t readIndex = decodeReadIndex(indices);

                index = writeIndex;
                if (count == COUNT)
                {
                    // The buffer is full. How that is treated depends on whether OVERWRITE is set or not.
                    if (!OVERWRITE)
                    {
                        // Overwrite isn't allowed so return false.
                        __CLREX();
                        return false;
                    }
                    else
                    {
                        // Advance the read index to discard the oldest element.
                        readIndex = (readIndex + 1) % COUNT;
                        // Keeping count set to the maximum value.
                    }
                }
                else
                {
                    // There is room to add another new element to the buffer.
                    count++;
                }
                writeIndex = (writeIndex + 1) % COUNT;
                indices = encodeIndices(count, writeIndex, readIndex);
                storeFailed = __STREXW(indices, &m_indices);
            } while (storeFailed);

            m_elements[index] = element;
            return true;
        }

        bool read(T& element)
        {
            uint32_t index = 0;
            int storeFailed = 1;
            do
            {
                uint32_t indices = __LDREXW(&m_indices);
                uint32_t count = decodeCount(indices);
                uint32_t writeIndex = decodeWriteIndex(indices);
                uint32_t readIndex = decodeReadIndex(indices);

                index = readIndex;
                if (count == 0)
                {
                    // Buffer is empty so fail the read.
                    __CLREX();
                    return false;
                }
                count--;
                readIndex = (readIndex + 1) % COUNT;
                indices = encodeIndices(count, writeIndex, readIndex);
                storeFailed = __STREXW(indices, &m_indices);
            } while (storeFailed);

            element = m_elements[index];
            return true;
        }

    protected:
        const size_t BITS_PER_INDEX = 10;
        const size_t WRITE_OFFSET = BITS_PER_INDEX;
        const size_t READ_OFFSET = 2 * BITS_PER_INDEX;
        const size_t INDEX_MASK = (1 << BITS_PER_INDEX) - 1;

        uint32_t decodeCount(uint32_t indices)
        {
            return indices & INDEX_MASK;
        }
        uint32_t decodeWriteIndex(uint32_t indices)
        {
            return (indices >> WRITE_OFFSET) & INDEX_MASK;
        }
        uint32_t decodeReadIndex(uint32_t indices)
        {
            return (indices >> READ_OFFSET) & INDEX_MASK;
        }
        uint32_t encodeIndices(uint32_t count, uint32_t writeIndex, uint32_t readIndex)
        {
            count &= INDEX_MASK;
            writeIndex &= INDEX_MASK;
            readIndex &= INDEX_MASK;

            return count | (writeIndex << WRITE_OFFSET) | (readIndex << READ_OFFSET);
        }

        T                 m_elements[COUNT];
        volatile uint32_t m_indices;
};

#endif // CIRCULAR_BUFFER_H_
