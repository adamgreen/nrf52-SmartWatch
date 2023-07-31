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
/* Acts as a bridge between local TCP/IP and remote mriblue devices.
   An application like GDB can open a socket to which this server is listening
   and be connected to a remote serial device over Bluetooth Low Energy.
*/
#include <ctype.h>
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "bleuart.h"



static void displayUsage(void)
{
    printf("\n"
           "Usage: mriblue [--port gdbPortNumber] [--log logFilename] [--verbose]\n"
           "Where:\n"
           "  --port is an optional parameter to specify TCP/IP port for GDB connections.\n"
           "         Defaults to port 3333.\n"
           "  --log is an optional parameter to specify name of file to which all TCP/IP\n"
           "        traffic should be logged. By default no logging is performed.\n"
           "  --verbose is an optional parameter to enable logging of TCP/IP traffic to\n"
           "            stdout.\n");
}



// Flags to be used in CommandLineParams::flags field.
#define FLAG_VERBOSE (1 << 0)

// Boolean values.
#define TRUE 1
#define FALSE 0


// Command line flags are parsed into this structure.
typedef struct CommandLineParams
{
    const char* pLogFilename;
    uint32_t    flags;
    uint16_t    portNumber;
} CommandLineParams;

// State of parsing out data destined for GDB from other data.
typedef struct PacketState
{
    int inGdbPacket;
    int foundDollarSign;
    int bytesLeft;
} PacketState;


static CommandLineParams g_params;
static FILE*             g_pLogFile;

// The Control+C handler and workerMain both need to access these globals.
static volatile int      g_controlCdetected;
static volatile int      g_gdbListenSocket = -1;
static volatile int      g_dataListenSocket = -1;
static volatile int      g_isBleConnected = 0;

static PacketState       g_packetState = { .inGdbPacket = FALSE, .foundDollarSign = FALSE, .bytesLeft = 0 };


static int parseCommandLine(CommandLineParams* pParams, int argc, char** ppArgs);
static int parsePortNumber(CommandLineParams* pParams, int argc, char** ppArgs);
static int parseLogFilename(CommandLineParams* pParams, int argc, char** ppArgs);
static void controlCHandler(int arg);
static int initListenSocket(uint16_t portNumber);
static ssize_t splitSend(int gdbSocket, int dataSocket, const void *pBuffer, size_t length);
static size_t findEndOfGdbData(const uint8_t* pStart, size_t length);
static size_t findStartOfGdbData(const uint8_t* pStart, size_t length);
static ssize_t sendOutOfBand(int socket, const uint8_t* pData, size_t length);
static int socketHasDataToRead(int socket);
static void logData(const char* pDirection, uint8_t* pData, int dataLength);
static const char* escapeData(const uint8_t* pBuffer, size_t bufferSize);
static char hexDigit(uint8_t digit);



int main(int argc, char *argv[])
{
    if (0 != parseCommandLine(&g_params, argc, argv))
    {
        displayUsage();
        return 1;
    }

    /*
       Initialize the Core Bluetooth stack on this the main thread and start the worker robot thread to run the
       code found in workerMain() below.
    */
    bleuartInitAndRun();
    return 0;
}

static int parseCommandLine(CommandLineParams* pParams, int argc, char** ppArgs)
{
    int result = 0;

    memset(pParams, 0, sizeof(*pParams));
    pParams->portNumber = 3333;

    // Skip executable name.
    ppArgs++;
    argc--;
    while (argc && result == 0)
    {
        const char* pArg = *ppArgs;
        if (0 == strcmp(pArg, "--port"))
        {
            result = parsePortNumber(pParams, --argc, ++ppArgs);
        }
        else if (0 == strcmp(pArg, "--log"))
        {
            result = parseLogFilename(pParams, --argc, ++ppArgs);
        }
        else if (0 == strcmp(pArg, "--verbose"))
        {
            pParams->flags |= FLAG_VERBOSE;
        }
        else
        {
            fprintf(stderr, "error: '%s' isn't a valid command line flag.\n", pArg);
            return -1;
        }

        ppArgs++;
        argc--;
    }

    return result;
}

static int parsePortNumber(CommandLineParams* pParams, int argc, char** ppArgs)
{
    if (argc < 1)
    {
        fprintf(stderr, "error: --port requires gdbPortNumber parameter.\n");
        return -1;
    }
    uint32_t portNumber = strtoul(*ppArgs, NULL, 0);
    if (portNumber > 0xFFFF)
    {
        fprintf(stderr, "error: gdbPortNumber of %u isn't valid.\n", portNumber);
        return -1;
    }
    pParams->portNumber = portNumber;
    return 0;
}

static int parseLogFilename(CommandLineParams* pParams, int argc, char** ppArgs)
{
    if (argc < 1)
    {
        fprintf(stderr, "error: --log requires logFilename parameter.\n");
        return -1;
    }
    pParams->pLogFilename = *ppArgs;
    return 0;
}



void workerMain(void)
{
    int result = -1;
    int gdbSocket = -1;
    int dataSocket = -1;

    signal(SIGINT, controlCHandler);
    signal(SIGPIPE, SIG_IGN);

    if (g_params.pLogFilename)
    {
        g_pLogFile = fopen(g_params.pLogFilename, "w");
        if (!g_pLogFile)
        {
            fprintf(stderr, "error: Failed to open '%s' logfile. %s\n", g_params.pLogFilename, strerror(errno));
            goto Error;
        }
    }

    g_gdbListenSocket = initListenSocket(g_params.portNumber);
    if (g_gdbListenSocket == -1)
    {
        printf("error: Failed to initialize socket on port %u. %s\n", g_params.portNumber, strerror(errno));
        goto Error;
    }

    g_dataListenSocket = initListenSocket(g_params.portNumber+1);
    if (g_dataListenSocket == -1)
    {
        printf("error: Failed to initialize socket on port %u. %s\n", g_params.portNumber+1, strerror(errno));
        goto Error;
    }

    while (!g_controlCdetected)
    {
        size_t             bytesRead;
        uint8_t            buffer[4096];

        if (!g_isBleConnected)
        {
            printf("Attempting to connect to MRIBLUE device...\n");
            result = bleuartConnect(NULL);
            if (result)
            {
                fprintf(stderr, "error: Failed to connect to remote MRIBLUE device.\n");
                goto Error;
            }
            printf("MRIBLUE device connected!\n");
            g_isBleConnected = 1;
        }
        if (gdbSocket == -1)
        {
            struct sockaddr_in remoteAddress;
            socklen_t          remoteAddressSize = sizeof(remoteAddress);

            printf("Waiting for GDB to connect on port %u...\n", g_params.portNumber);
            gdbSocket = accept(g_gdbListenSocket, (struct sockaddr*)&remoteAddress, &remoteAddressSize);
            if (gdbSocket == -1 && errno != ECONNABORTED)
            {
                fprintf(stderr, "error: Failed to accept TCP/IP connection. %s\n", strerror(errno));
                goto Error;
            }
            if (gdbSocket != -1)
            {
                printf("GDB connected!\n");
            }
        }

        while (gdbSocket != -1 && g_isBleConnected && !g_controlCdetected)
        {
            /* Forward data received from BLEUART to socket. */
            result = bleuartReceiveData(buffer, sizeof(buffer), &bytesRead);
            if (result == BLEUART_ERROR_NOT_CONNECTED)
            {
                printf("BLE connection lost!\n");
                g_isBleConnected = 0;
                break;
            }
            if (bytesRead > 0)
            {
                logData("tcp<-ble", buffer, (int)bytesRead);
                result = splitSend(gdbSocket, dataSocket, buffer, bytesRead);
                if (result == -1)
                {
                    printf("TCP/IP connection to GDB lost!\n");
                    close(gdbSocket);
                    gdbSocket = -1;
                    break;
                }
                else if (result == -2)
                {
                    printf("TCP/IP connection to Data Application lost!\n");
                    close(dataSocket);
                    dataSocket = -1;
                    break;
                }
            }

            /* Forward data received from socket to BLEUART. */
            if (socketHasDataToRead(gdbSocket))
            {
                result = recv(gdbSocket, buffer, sizeof(buffer), 0);
                if (result < 1)
                {
                    printf("TCP/IP connection lost!\n");
                    close(gdbSocket);
                    gdbSocket = -1;
                    break;
                }
                logData("tcp->ble", buffer, result);
                result = bleuartTransmitData(buffer, result);
                if (result == BLEUART_ERROR_NOT_CONNECTED)
                {
                    printf("BLE connection lost!\n");
                    g_isBleConnected = 0;
                    break;
                }
                else if (result != BLEUART_ERROR_NONE)
                {
                    printf("BLE transmit returned error: %d\n", result);
                }
            }

            if (dataSocket == -1 && socketHasDataToRead(g_dataListenSocket))
            {
                struct sockaddr_in remoteAddress;
                socklen_t          remoteAddressSize = sizeof(remoteAddress);

                printf("Waiting for Data Application to connect on port %u...\n", g_params.portNumber+1);
                dataSocket = accept(g_dataListenSocket, (struct sockaddr*)&remoteAddress, &remoteAddressSize);
                if (dataSocket == -1 && errno != ECONNABORTED)
                {
                    fprintf(stderr, "error: Failed to accept TCP/IP connection. %s\n", strerror(errno));
                    goto Error;
                }
                if (dataSocket != -1)
                {
                    printf("Data Application connected!\n");
                }
            }

            /* This is a little more time than it takes to transmit one byte at 115200. */
            usleep(100);
        }
    }

Error:
    printf("Shutting down\n");
    if (gdbSocket != -1)
        close (gdbSocket);
    if (g_dataListenSocket != -1)
        close(g_dataListenSocket);
    if (g_gdbListenSocket != -1)
        close(g_gdbListenSocket);
    if (g_pLogFile)
        fclose(g_pLogFile);
    bleuartDisconnect();
}

static void controlCHandler(int arg)
{
    g_controlCdetected = 1;
    if (g_gdbListenSocket != -1)
    {
        int listenSocket = g_gdbListenSocket;
        g_gdbListenSocket = -1;
        close(listenSocket);
    }
    if (g_dataListenSocket != -1)
    {
        int listenSocket = g_dataListenSocket;
        g_dataListenSocket = -1;
        close(listenSocket);
    }
    if (!g_isBleConnected)
    {
        bleuartAbortConnectionAttempt();
    }
}

static int initListenSocket(uint16_t portNumber)
{
    int optionValue = 1;
    int listenSocket = -1;
    int result = -1;
    struct sockaddr_in bindAddress;

    listenSocket = socket(PF_INET, SOCK_STREAM, 0);
    if (listenSocket == -1)
        goto Error;
    setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &optionValue, sizeof(optionValue));

    memset(&bindAddress, 0, sizeof(bindAddress));
    bindAddress.sin_family = AF_INET;
    bindAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    bindAddress.sin_port = htons(portNumber);
    result = bind(listenSocket, (struct sockaddr *)&bindAddress, sizeof(bindAddress));
    if (result == -1)
        goto Error;

    result = listen(listenSocket, 0);
    if (result == -1)
        goto Error;

    return listenSocket;

Error:
    if (listenSocket != -1)
        close(listenSocket);
    return -1;
}

static ssize_t splitSend(int gdbSocket, int dataSocket, const void *pBuffer, size_t length)
{
    const uint8_t* pCurr = (const uint8_t*)pBuffer;
    size_t bytesLeft = length;

    while (bytesLeft > 0)
    {
        size_t len = 0;

        if (g_packetState.inGdbPacket)
        {
            len = findEndOfGdbData(pCurr, bytesLeft);
            if (len > 0)
            {
                ssize_t result = send(gdbSocket, pCurr, len, 0);
                if (result == -1)
                {
                    return -1;
                }
            }
        }
        else
        {
            len = findStartOfGdbData(pCurr, bytesLeft);
            if (len > 0 && dataSocket != -1)
            {
                ssize_t result = sendOutOfBand(dataSocket, pCurr, len);
                if (result == -1)
                {
                    return -2;
                }
            }
        }
        pCurr += len;
        bytesLeft -= len;
    }

    return length;
}

static size_t findEndOfGdbData(const uint8_t* pStart, size_t length)
{
    const uint8_t* pCurr = pStart;

    while (length-- > 0)
    {
        char ch = *pCurr;

        if (!g_packetState.foundDollarSign)
        {
            if (ch == '$')
            {
                g_packetState.bytesLeft = 0;
                g_packetState.foundDollarSign = TRUE;
            }
            else if (ch != '+' && ch != '-')
            {
                g_packetState.bytesLeft = 0;
                g_packetState.inGdbPacket = FALSE;
                break;
            }
        }
        else if (g_packetState.bytesLeft == 0 && ch == '#')
        {
            g_packetState.bytesLeft = 2;
        }
        else if (g_packetState.bytesLeft > 0)
        {
            g_packetState.bytesLeft--;
            if (g_packetState.bytesLeft == 0)
            {
                g_packetState.foundDollarSign = FALSE;
            }
        }

        pCurr++;
    }

    return pCurr - pStart;
}

static size_t findStartOfGdbData(const uint8_t* pStart, size_t length)
{
    const uint8_t* pCurr = pStart;

    while (length-- > 0)
    {
        char ch = *pCurr;

        if (ch == '+' || ch == '-')
        {
            g_packetState.bytesLeft = 0;
            g_packetState.inGdbPacket = TRUE;
            g_packetState.foundDollarSign = FALSE;
            break;
        }
        else if (ch == '$')
        {
            g_packetState.bytesLeft = 0;
            g_packetState.inGdbPacket = TRUE;
            g_packetState.foundDollarSign = TRUE;
            break;
        }

        pCurr++;
    }

    return pCurr - pStart;
}

static ssize_t sendOutOfBand(int socket, const uint8_t* pSrc, size_t length)
{
    // Unescape the out of band data which starts with a '}' prefix.
    static int lastByteWasPrefix = FALSE;
    static uint8_t buffer[64*1024];
    uint8_t* pDest = buffer;
    while (length-- && (pDest - buffer) < sizeof(buffer))
    {
        uint8_t byte = *pSrc++;
        if (!lastByteWasPrefix && byte == '}')
        {
            lastByteWasPrefix = TRUE;
        }
        else if (lastByteWasPrefix)
        {
            *pDest++ = byte ^ 0x20;
            lastByteWasPrefix = FALSE;
        }
        else
        {
            *pDest++ = byte;
            lastByteWasPrefix = FALSE;
        }
    }
    if (pDest-buffer >= sizeof(buffer))
    {
        printf("**MRIBLUE OVERFLOW**\n");
        return -2;
    }

    ssize_t result = send(socket, buffer, pDest-buffer, 0);
    if (result == -1)
    {
        return -1;
    }

    return 0;
}

static int socketHasDataToRead(int socket)
{
    int            result = -1;
    struct timeval zeroTimeout = {0, 0};
    fd_set         readSet;

    FD_ZERO(&readSet);
    FD_SET(socket, &readSet);
    result = select(socket + 1, &readSet, NULL, NULL, &zeroTimeout);
    if (result == -1)
        return 0;
    return result;
}

static void logData(const char* pDirection, uint8_t* pData, int dataLength)
{
    if ((g_params.flags & FLAG_VERBOSE) == 0 && g_pLogFile == NULL)
    {
        // No logging is enabled so just return.
        return;
    }

    const char* pEscapedData = escapeData(pData, dataLength);
    const char* formatString = "%s: %s\n";
    if (g_params.flags & FLAG_VERBOSE)
    {
        printf(formatString, pDirection, pEscapedData);
    }
    if (g_pLogFile)
    {
        fprintf(g_pLogFile, formatString, pDirection, pEscapedData);
    }
}

static const char* escapeData(const uint8_t* pBuffer, size_t bufferSize)
{
    static char escapedBuffer[1024*1024];
    char* pDest = escapedBuffer;
    size_t bytesLeft = sizeof(escapedBuffer) - 1;

    while (bufferSize-- > 0)
    {
        uint8_t ch = *pBuffer;
        if (isprint(ch))
        {
            if (bytesLeft < 1)
            {
                break;
            }
            *pDest++ = ch;
        }
        else
        {
            if (bytesLeft < 4)
            {
                break;
            }
            pDest[0] = '\\';
            pDest[1] = 'x';
            pDest[2] = hexDigit(ch >> 4);
            pDest[3] = hexDigit(ch & 0xF);
            pDest += 4;
        }

        pBuffer++;
    }
    *pDest++ = '\0';

    return escapedBuffer;
}

static char hexDigit(uint8_t digit)
{
    char digits[] = "0123456789ABCDEF";
    if (digit > 15)
    {
        return '?';
    }
    return digits[digit];
}
