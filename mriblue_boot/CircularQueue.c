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
/* Circular queue used to communicate between BLE stack and MRI. */
#include <nrf_assert.h>
#include <nrf_atomic.h>
#include "CircularQueue.h"

static uint32_t incrementIndex(uint32_t index);

void CircularQueue_Init(CircularQueue* pThis)
{
    ASSERT ( (sizeof(pThis->queue) & (sizeof(pThis->queue)-1)) == 0 );

    pThis->write = 0;
    pThis->read = 0;
    pThis->peek = 0;
    pThis->peekSize = 0;
    pThis->count = 0;
}

uint32_t CircularQueue_Write(CircularQueue* pThis, const uint8_t* pData, uint32_t dataSize)
{
    ASSERT ( pThis->count <= sizeof(pThis->queue) );

    uint32_t bytesLeft = sizeof(pThis->queue) - pThis->count;
    uint32_t bytesToWrite = (bytesLeft < dataSize) ? bytesLeft : dataSize;
    for (uint32_t i = 0 ; i < bytesToWrite ; i++)
    {
        pThis->queue[pThis->write] = *pData++;
        pThis->write = incrementIndex(pThis->write);
    }
    nrf_atomic_u32_add(&pThis->count, bytesToWrite);
    return bytesToWrite;
}

static uint32_t incrementIndex(uint32_t index)
{
    return (index + 1) & (CIRCULAR_QUEUE_SIZE-1);
}

uint32_t CircularQueue_BytesToRead(CircularQueue* pThis)
{
    return pThis->count;
}

uint32_t CircularQueue_IsFull(CircularQueue* pThis)
{
    return CircularQueue_BytesToRead(pThis) == CIRCULAR_QUEUE_SIZE;
}

uint32_t CircularQueue_IsEmpty(CircularQueue* pThis)
{
    return CircularQueue_BytesToRead(pThis) == 0;
}

uint32_t CircularQueue_Read(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize)
{
    ASSERT ( pThis->count <= sizeof(pThis->queue) );

    uint32_t bytesToRead = (dataSize > pThis->count) ? pThis->count : dataSize;
    for (uint32_t i = 0 ; i < bytesToRead ; i++)
    {
        *pData++ = pThis->queue[pThis->read];
        pThis->read = incrementIndex(pThis->read);
    }
    nrf_atomic_u32_sub(&pThis->count, bytesToRead);
    return bytesToRead;
}

uint32_t CircularQueue_Peek(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize)
{
    ASSERT ( pThis->count <= sizeof(pThis->queue) );
    ASSERT ( pThis->peekSize == 0 );

    pThis->peek = pThis->read;
    uint32_t bytesToRead = (dataSize > pThis->count) ? pThis->count : dataSize;
    for (uint32_t i = 0 ; i < bytesToRead ; i++)
    {
        *pData++ = pThis->queue[pThis->peek];
        pThis->peek = incrementIndex(pThis->peek);
    }
    pThis->peekSize = bytesToRead;
    return bytesToRead;
}

void CircularQueue_CommitPeek(CircularQueue* pThis)
{
    ASSERT ( pThis->peekSize > 0 );

    pThis->read = pThis->peek;
    nrf_atomic_u32_sub(&pThis->count, pThis->peekSize);
    pThis->peekSize = 0;
}

void CircularQueue_RollbackPeek(CircularQueue* pThis)
{
    ASSERT ( pThis->peekSize > 0 );

    pThis->peekSize = 0;
}
