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
#ifndef CICRCULAR_QUEUE_H_
#define CICRCULAR_QUEUE_H_

#include <stdint.h>


#define CIRCULAR_QUEUE_SIZE 256

typedef struct CircularQueue {
    uint32_t write;
    uint32_t read;
    uint32_t peek;
    uint32_t peekSize;
    volatile uint32_t count;
    volatile uint8_t  queue[CIRCULAR_QUEUE_SIZE];
} CircularQueue;

void CircularQueue_Init(CircularQueue* pThis);
uint32_t CircularQueue_Write(CircularQueue* pThis, const uint8_t* pData, uint32_t dataSize);
uint32_t CircularQueue_BytesToRead(CircularQueue* pThis);
uint32_t CircularQueue_IsFull(CircularQueue* pThis);
uint32_t CircularQueue_IsEmpty(CircularQueue* pThis);
uint32_t CircularQueue_Read(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize);
uint32_t CircularQueue_Peek(CircularQueue* pThis, uint8_t* pData, uint32_t dataSize);
void CircularQueue_CommitPeek(CircularQueue* pThis);
void CircularQueue_RollbackPeek(CircularQueue* pThis);

#endif // CICRCULAR_QUEUE_H_
