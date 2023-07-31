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
/* Implementation of BLEUART transport for OS X using Core Bluetooth.
   It runs a NSApplication on the main thread and runs the developer's code on a worker thread workerMain().  This
   code is to be used with console applications on OS X.
*/
#import <Cocoa/Cocoa.h>
#import <CoreBluetooth/CoreBluetooth.h>
#import <pthread.h>
#import <sys/time.h>
#import "bleuart.h"


// Forward Declarations.
static void* workerThread(void* pArg);


// This is the service UUID advertised by MRIBLUE devices.
#define MRIBLUE_ADVERTISE "6E40ADA4-B5A3-F393-E0A9-E50E24DCCA9E"

// This is the MRIBLUE service UUID.
#define BLEUART_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

// The controller can register for receive data notifications on this characteristic.
#define BLEUART_RECEIVE_DATA_NOTIFY_CHARACTERISTIC "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
// Characteristic used to send transmit data to BLEUART.
#define BLEUART_TRANSMIT_DATA_WRITE_CHARACTERISTIC "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// Size of receive data queue.  The queue will overwrite the oldest item once this size is hit.
#define BLEUART_RECEIVE_QUEUE_SIZE 1024



// This class implements a fixed sized circular queue which supports push overflow.
@interface CircularQueue : NSObject
{
    uint8_t*        pQueue;
    size_t          alloc;
    size_t          count;
    size_t          push;
    size_t          pop;
    pthread_mutex_t mutex;
}

- (id) initWithSize:(size_t) itemCount;
- (void) pushByte:(uint8_t) byte;
- (void) push:(const uint8_t*)pData length:(size_t)length;
- (int)  pop:(uint8_t*)pBuffer size:(size_t)size actualLength:(size_t*)pActual;
- (int)  popByte:(uint8_t*)pByte;
@end



@implementation CircularQueue
// Initialize the circular queue.  Allocate room for the specified number of bytes in the queue.
- (id) initWithSize:(size_t) itemCount
{
    int mutexInit = -1;

    self = [super init];
    if (!self)
        goto Error;

    pQueue = malloc(itemCount * sizeof(*pQueue));
    if (!pQueue)
        goto Error;
    mutexInit = pthread_mutex_init(&mutex, NULL);
    if (mutexInit)
        goto Error;
    alloc = itemCount;

    return self;
Error:
    if (mutexInit == 0)
        pthread_mutex_destroy(&mutex);
    free(pQueue);
    pQueue = NULL;
    return nil;
}

// Free pthread synchronization objects when this object is finally freed.
- (void) dealloc
{
    pthread_mutex_destroy(&mutex);
    free(pQueue);
    pQueue = NULL;
    [super dealloc];
}

- (void) pushByte:(uint8_t)byte
{
    pthread_mutex_lock(&mutex);
    {
        pQueue[push] = byte;
        push = (push + 1) % alloc;
        if (count == alloc)
        {
            // Queue was already full so drop oldest item by advancing the pop index.
            pop = (pop + 1) % alloc;
        }
        else
        {
            count++;
        }
    }
    pthread_mutex_unlock(&mutex);
}

- (void) push:(const uint8_t*)pData length:(size_t)length
{
    while (length--)
    {
        [self pushByte:*pData];
        pData++;
    }
}

- (int)  popByte:(uint8_t*)pByte
{
    int ret = BLEUART_ERROR_EMPTY;

    pthread_mutex_lock(&mutex);
    {
        if (count > 0)
        {
            *pByte = pQueue[pop];
            pop = (pop + 1) % alloc;
            count--;
            ret = BLEUART_ERROR_NONE;
        }
    }
    pthread_mutex_unlock(&mutex);

    return ret;
}

- (int)  pop:(uint8_t*)pBuffer size:(size_t)size actualLength:(size_t*)pActual
{
    size_t bytesRead = 0;

    while (size && [self popByte:pBuffer] == BLEUART_ERROR_NONE)
    {
        size--;
        pBuffer++;
        bytesRead++;
    }

    *pActual = bytesRead;
    return (bytesRead == 0) ? BLEUART_ERROR_EMPTY : BLEUART_ERROR_NONE;
}
@end



// This is the delegate where most of the work on the main thread occurs.
@interface BleUartAppDelete : NSObject <NSApplicationDelegate, CBCentralManagerDelegate, CBPeripheralDelegate>
{
    CBCentralManager*   manager;
    NSMutableArray*     discoveredDevices;
    CBPeripheral*       peripheral;
    CBCharacteristic*   transmitDataWriteCharacteristic;

    int                 error;
    int32_t             characteristicsToFind;
    BOOL                autoConnect;
    BOOL                isBlePowerOn;
    BOOL                scanOnBlePowerOn;

    pthread_mutex_t     connectMutex;
    pthread_cond_t      connectCondition;
    pthread_t           thread;

    NSObject*           napDisable;

    CircularQueue*      receiveQueue;
}

- (id) initForApp:(NSApplication*) app;
- (int) error;
- (void) clearPeripheral;
- (void) handleDeviceConnect:(id) deviceName;
- (void) foundCharacteristic;
- (void) signalConnectionError;
- (void) waitForConnectToComplete;
- (void) handleDeviceDisconnect:(id) dummy;
- (void) waitForDisconnectToComplete;
- (void) handleDeviceDiscoveryStart:(id) dummy;
- (void) handleDeviceDiscoveryStop:(id) dummy;
- (NSUInteger) getDiscoveredDeviceCount;
- (NSString*) getDiscoveredDeviceAtIndex:(NSUInteger) index;
- (void) handleTransmitRequest:(id) request;
- (int) popReceiveData:(uint8_t*) pBuffer size:(size_t) size actualLength:(size_t*) pActual;
- (void) handleQuitRequest:(id) dummy;
- (void) startScan;
- (void) stopScan;
@end



@implementation BleUartAppDelete
// Initialize this delegate.
// Create necessary synchronization objects for managing worker thread's access to connection and response state.
// Also adds itself as the delegate to the main NSApplication object.
- (id) initForApp:(NSApplication*) app;
{
    int connectMutexResult = -1;
    int connectConditionResult = -1;

    self = [super init];
    if (!self)
        return nil;

    discoveredDevices = [[NSMutableArray alloc] init];
    if (!discoveredDevices)
        goto Error;
    receiveQueue = [[CircularQueue alloc] initWithSize:BLEUART_RECEIVE_QUEUE_SIZE];
    if (!receiveQueue)
        goto Error;

    connectMutexResult = pthread_mutex_init(&connectMutex, NULL);
    if (connectMutexResult)
        goto Error;
    connectConditionResult = pthread_cond_init(&connectCondition, NULL);
    if (connectConditionResult)
        goto Error;

    [app setDelegate:self];
    return self;

Error:
    if (connectConditionResult == 0)
        pthread_cond_destroy(&connectCondition);
    if (connectMutexResult == 0)
        pthread_mutex_destroy(&connectMutex);
    [receiveQueue release];
    [discoveredDevices release];
    return nil;
}


// Invoked when application finishes launching.
// Initialize the Core Bluetooth manager object and also starts up the worker thread.  This worker thread will end up
// running the code in the developer's workerMain() implementation.
- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    manager = [[CBCentralManager alloc] initWithDelegate:self queue:nil];
    pthread_create(&thread, NULL, workerThread, self);

    // Disable App Nap as it causes all kinds of problems for this application.
    NSActivityOptions options = NSActivityUserInitiatedAllowingIdleSystemSleep;
    NSString *reason = @"No napping on this background job!";
    napDisable = [[NSProcessInfo processInfo] beginActivityWithOptions:options reason:reason];
    [napDisable retain];
}

// Invoked just before application will shutdown.
- (void)applicationWillTerminate:(NSNotification *)aNotification
{
    // Stop any BLE discovery process that might have been taking place.
    [self stopScan];

    // Disconnect from the device if necessary.
    if(peripheral)
    {
        [manager cancelPeripheralConnection:peripheral];
        [self clearPeripheral];
    }

    // Free up resources here rather than dealloc which doesn't appear to be called during NSApplication shutdown.
    [receiveQueue release];
    receiveQueue = nil;
    [discoveredDevices release];
    discoveredDevices = nil;

    [manager release];
    manager = nil;

    pthread_cond_destroy(&connectCondition);
    pthread_mutex_destroy(&connectMutex);

    [[NSProcessInfo processInfo] endActivity:napDisable];
    [napDisable release];
}

// Request CBCentralManager to stop scanning for BLEUART devices.
- (void) stopScan
{
    [manager stopScan];
}

// Clear BLE peripheral member.
- (void) clearPeripheral
{
    if (!peripheral)
        return;

    pthread_mutex_lock(&connectMutex);
        [peripheral setDelegate:nil];
        [peripheral release];
        peripheral = nil;
    pthread_mutex_unlock(&connectMutex);
    pthread_cond_signal(&connectCondition);
}

// Handle device connection request posted to the main thread by the worker thread.
- (void) handleDeviceConnect:(id) deviceName
{
    error = BLEUART_ERROR_NONE;
    characteristicsToFind = -1;
    if (discoveredDevices.count > 0)
    {
        // A discovery scan has already been completed so use the list of discovered devices.
        CBPeripheral* device = nil;
        if (deviceName == nil)
        {
            // Just use the first item in the discovered device list.
            device = [discoveredDevices objectAtIndex:0];
        }
        else
        {
            // Find the specified device in the list of discovered devices.
            for (NSUInteger deviceIndex = 0 ; deviceIndex < discoveredDevices.count ; deviceIndex++)
            {
                device = [discoveredDevices objectAtIndex:deviceIndex];
                if ([(NSString*)deviceName compare:device.name] == NSOrderedSame)
                    break;
            }
        }
        // Make sure that the specified deviceName is valid.
        if (!device)
        {
            error = BLEUART_ERROR_PARAM;
            return;
        }

        // Connect to specified device.
        NSLog(@"Connecting to %@", device.name);
        [self stopScan];
        autoConnect = FALSE;
        characteristicsToFind = 2;
        peripheral = device;
        [peripheral retain];
        [manager connectPeripheral:peripheral options:nil];
    }
    else if (deviceName == nil)
    {
        autoConnect = TRUE;
        characteristicsToFind = 2;
        [self startScan];
    }
    else
    {
        // Can't specify a deviceName without first discovering near devices.
        error = BLEUART_ERROR_PARAM;
        return;
    }
}

// Request CBCentralManager to scan for BLEUART devices via the service that it broadcasts.
- (void) startScan
{
    if (!isBlePowerOn)
    {
        // Postpone the scan start until later when BLE power on is detected.
        scanOnBlePowerOn = TRUE;
        return;
    }
    else
    {
        [manager scanForPeripheralsWithServices:[NSArray arrayWithObject:[CBUUID UUIDWithString:@MRIBLUE_ADVERTISE]] options:nil];
    }
}

// Invoked when the central discovers BLEUART device while scanning.
- (void) centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)aPeripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI
{
    // Add to discoveredDevices array if not already present in that list.
    @synchronized(discoveredDevices)
    {
        if (![discoveredDevices containsObject:aPeripheral])
            [discoveredDevices addObject:aPeripheral];
    }

    // If the user wants to connect to first discovered device then issue connection request now.
    if (autoConnect)
    {
        // Connect to first device found.
        [self stopScan];
        autoConnect = FALSE;
        peripheral = aPeripheral;
        [peripheral retain];
        [manager connectPeripheral:peripheral options:nil];
    }
}

// Invoked whenever a connection is succesfully created with a BLEUART device.
// Start discovering available BLE services on the device.
- (void) centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)aPeripheral
{
    [aPeripheral setDelegate:self];
    [aPeripheral discoverServices:[NSArray arrayWithObject:[CBUUID UUIDWithString:@BLEUART_SERVICE]]];
}

// Invoked whenever an existing connection with the peripheral is torn down.
- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)aPeripheral error:(NSError *)err
{
    [self clearPeripheral];
}

// Invoked whenever the central manager fails to create a connection with the peripheral.
- (void)centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)aPeripheral error:(NSError *)err
{
    NSLog(@"didFailToConnectPeripheral");
    NSLog(@"err = %@", err);
    [self clearPeripheral];
    [self signalConnectionError];
}

// Error was encountered while attempting to connect to device.
// Record this error and unblock worker thread which is waiting for the connection to complete.
- (void) signalConnectionError
{
    pthread_mutex_lock(&connectMutex);
        characteristicsToFind = -1;
        error = BLEUART_ERROR_CONNECT;
    pthread_mutex_unlock(&connectMutex);
    pthread_cond_signal(&connectCondition);
}

// Invoked upon completion of a -[discoverServices:] request.
// Discover available characteristics on interested services.
- (void) peripheral:(CBPeripheral *)aPeripheral didDiscoverServices:(NSError *)error
{
    for (CBService *aService in aPeripheral.services)
    {
        /* BLEUART specific services */
        if ([aService.UUID isEqual:[CBUUID UUIDWithString:@BLEUART_SERVICE]])
        {
            [aPeripheral discoverCharacteristics:nil forService:aService];
        }
    }
}

// Invoked upon completion of a -[discoverCharacteristics:forService:] request.
// Perform appropriate operations on interested characteristics.
- (void) peripheral:(CBPeripheral *)aPeripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error
{
    /* BLEUART service. */
    if ([service.UUID isEqual:[CBUUID UUIDWithString:@BLEUART_SERVICE]])
    {
        for (CBCharacteristic *aChar in service.characteristics)
        {
            /* Set notification on received data. */
            if ([aChar.UUID isEqual:[CBUUID UUIDWithString:@BLEUART_RECEIVE_DATA_NOTIFY_CHARACTERISTIC]])
            {
                [peripheral setNotifyValue:YES forCharacteristic:aChar];
                [self foundCharacteristic];
            }
            /* Remember transmit data characteristic. */
            if ([aChar.UUID isEqual:[CBUUID UUIDWithString:@BLEUART_TRANSMIT_DATA_WRITE_CHARACTERISTIC]])
            {
                transmitDataWriteCharacteristic = aChar;
                [self foundCharacteristic];
            }
        }
    }
}

// Found one of the two characteristics required for communicating with the BLEUART device.
// The worker thread will be waiting for both of these characteristics to be found so there is code to unblock it.
- (void) foundCharacteristic
{
    pthread_mutex_lock(&connectMutex);
        characteristicsToFind--;
    pthread_mutex_unlock(&connectMutex);
    pthread_cond_signal(&connectCondition);
}

// The worker thread calls this selector to wait for the connection to the device to complete.
- (void) waitForConnectToComplete
{
    pthread_mutex_lock(&connectMutex);
        while (characteristicsToFind > 0)
            pthread_cond_wait(&connectCondition, &connectMutex);
    pthread_mutex_unlock(&connectMutex);
}

// The worker thread calls this selector to determine if the main thread has encountered an error.
- (int) error
{
    return error;
}

// Handle device disconnection request posted to the main thread by the worker thread.
- (void) handleDeviceDisconnect:(id) dummy
{
    error = BLEUART_ERROR_NONE;

    if(!peripheral)
        return;
    [manager cancelPeripheralConnection:peripheral];
}

// The worker thread calls this selector to wait for the disconnection from the device to complete.
- (void) waitForDisconnectToComplete
{
    pthread_mutex_lock(&connectMutex);
        while (peripheral)
            pthread_cond_wait(&connectCondition, &connectMutex);
    pthread_mutex_unlock(&connectMutex);
}

// Handle device discovery start request posted to the main thread by the worker thread.
- (void) handleDeviceDiscoveryStart:(id) dummy
{
    error = BLEUART_ERROR_NONE;
    autoConnect = FALSE;
    [self startScan];
}

// Handle device discovery stop request posted to the main thread by the worker thread.
- (void) handleDeviceDiscoveryStop:(id) dummy
{
    [self stopScan];
}

// The worker thread calls this selector to determine how many devices have been discovered so far.
- (NSUInteger) getDiscoveredDeviceCount
{
    NSUInteger count = 0;
    @synchronized(discoveredDevices)
    {
        count = [discoveredDevices count];
    }
    return count;
}

// The worker thread calls this selector to obtain the name for one of the discovered devices.
- (NSString*) getDiscoveredDeviceAtIndex:(NSUInteger) index
{
    CBPeripheral* p = nil;
    @synchronized(discoveredDevices)
    {
        p = [discoveredDevices objectAtIndex:index];
    }
    return p.name;
}

// Handle BLEUART transmit request posted to the main thread by the worker thread.
- (void) handleTransmitRequest:(id) object
{
    if (!peripheral || !transmitDataWriteCharacteristic)
    {
        // Don't have a successful connection so error out.
        error = BLEUART_ERROR_NOT_CONNECTED;
        return;
    }
    error = BLEUART_ERROR_NONE;

    // Send request to BLEUART via Core Bluetooth.
    NSData* cmdData = (NSData*)object;
    [peripheral writeValue:cmdData forCharacteristic:transmitDataWriteCharacteristic type:CBCharacteristicWriteWithoutResponse];
}

// Invoked upon completion of a -[readValueForCharacteristic:] request or on the reception of a notification/indication.
- (void) peripheral:(CBPeripheral *)aPeripheral didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)err
{
    if (err)
        NSLog(@"Read encountered error (%@)", err);

    // Received data from BLEUART device.
    if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:@BLEUART_RECEIVE_DATA_NOTIFY_CHARACTERISTIC]])
    {
        [receiveQueue push:characteristic.value.bytes length:[characteristic.value length]];
    }
    else
    {
        NSLog(@"Unexpected characteristic %@", characteristic.UUID);
        NSLog(@"characteristic = %@", characteristic.value.bytes);
    }
}

// Handle application shutdown request posted to the main thread by the worker thread.
- (void) handleQuitRequest:(id) dummy
{
    [NSApp terminate:self];
}

// The worker thread calls this selector to make a deep copy of any received data.
- (int) popReceiveData:(uint8_t*) pBuffer size:(size_t) size actualLength:(size_t*) pActual
{
    if (!peripheral)
    {
        // Don't have a successful connection so error out.
        return BLEUART_ERROR_NOT_CONNECTED;
    }
    return [receiveQueue pop:pBuffer size:size actualLength:pActual];
}

// Invoked whenever the central manager's state is updated.
- (void) centralManagerDidUpdateState:(CBCentralManager *)central
{
    NSString * state = nil;

    // Display an error to user if there is no BLE hardware and then force an exit.
    switch ([manager state])
    {
        case CBManagerStateUnsupported:
            state = @"The platform/hardware doesn't support Bluetooth Low Energy.";
            break;
        case CBManagerStateUnauthorized:
            state = @"The app is not authorized to use Bluetooth Low Energy.";
            break;
        case CBManagerStatePoweredOff:
            isBlePowerOn = FALSE;
            state = @"Bluetooth is currently powered off.";
            break;
        case CBManagerStatePoweredOn:
            isBlePowerOn = TRUE;
            if (scanOnBlePowerOn)
            {
                scanOnBlePowerOn = FALSE;
                [self startScan];
            }
            return;
        case CBManagerStateUnknown:
        default:
            return;
    }

    NSLog(@"Central manager state: %@", state);
    [NSApp terminate:self];
}
@end



// *** Implementation of lower level C APIs that make use of above Objective-C classes. ***
static BleUartAppDelete* g_appDelegate;

// Initialize the BLE2UART transport.
void bleuartInitAndRun(void)
{
    [NSApplication sharedApplication];
    g_appDelegate = [[BleUartAppDelete alloc] initForApp:NSApp];
    [NSApp run];
    [g_appDelegate release];
    return;
}

// Worker thread root function.
// Calls developer's workerMain() function and upon return sends the Quit request to the main application thread.
static void* workerThread(void* pArg)
{
    workerMain();
    [g_appDelegate performSelectorOnMainThread:@selector(handleQuitRequest:) withObject:nil waitUntilDone:YES];
    return NULL;
}

int bleuartConnect(const char* pName)
{
    NSString* nameObject = nil;

    if (pName)
        nameObject = [NSString stringWithUTF8String:pName];
    [g_appDelegate performSelectorOnMainThread:@selector(handleDeviceConnect:) withObject:nameObject waitUntilDone:YES];
    [g_appDelegate waitForConnectToComplete];
    [nameObject release];

    return [g_appDelegate error];
}

int bleuartDisconnect()
{
    [g_appDelegate performSelectorOnMainThread:@selector(handleDeviceDisconnect:) withObject:nil waitUntilDone:YES];
    [g_appDelegate waitForDisconnectToComplete];
    sleep(1);

    return [g_appDelegate error];
}

int bleuartAbortConnectionAttempt()
{
    [g_appDelegate performSelectorOnMainThread:@selector(signalConnectionError) withObject:nil waitUntilDone:YES];
    return BLEUART_ERROR_NONE;
}

int bleuartStartDeviceDiscovery()
{
    [g_appDelegate performSelectorOnMainThread:@selector(handleDeviceDiscoveryStart:) withObject:nil waitUntilDone:YES];
    return [g_appDelegate error];
}

int bleuartGetDiscoveredDeviceCount(size_t* pCount)
{
    NSUInteger count = [g_appDelegate getDiscoveredDeviceCount];
    *pCount = (size_t)count;
    return [g_appDelegate error];
}

int bleuartGetDiscoveredDeviceName(size_t deviceIndex, const char** ppDeviceName)
{
    NSString* pName = [g_appDelegate getDiscoveredDeviceAtIndex:deviceIndex];
    *ppDeviceName = pName.UTF8String;
    return [g_appDelegate error];
}

int bleuartStopDeviceDiscovery()
{
    [g_appDelegate performSelectorOnMainThread:@selector(handleDeviceDiscoveryStop:) withObject:nil waitUntilDone:YES];
    return [g_appDelegate error];
}

static int transmitChunk(const void* pData, size_t dataLength)
{
    NSData* p = [NSData dataWithBytesNoCopy:(void*)pData length:dataLength freeWhenDone:NO];
    if (!p)
        return BLEUART_ERROR_MEMORY;

    [g_appDelegate performSelectorOnMainThread:@selector(handleTransmitRequest:) withObject:p waitUntilDone:YES];
    [p release];
    return [g_appDelegate error];
}

int bleuartTransmitData(const void* pData, size_t dataLength)
{
    int result = BLEUART_ERROR_NONE;
    uint8_t* pCurr = (uint8_t*)pData;
    size_t bytesLeft = dataLength;

    while (bytesLeft > 0)
    {
        size_t bytesToTransmit = bytesLeft;
        if (bytesToTransmit > 20)
        {
            bytesToTransmit = 20;
        }
        result = transmitChunk(pCurr, bytesToTransmit);
        if (result)
        {
            return result;
        }
        pCurr += bytesToTransmit;
        bytesLeft -= bytesToTransmit;
    }

    return result;
}

int bleuartReceiveData(void* pBuffer, size_t bufferSize, size_t* pActualLength)
{
    return [g_appDelegate popReceiveData:pBuffer size:bufferSize actualLength:pActualLength];
}
