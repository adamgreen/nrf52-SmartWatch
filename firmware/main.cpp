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
// Beginnings of firmware for Bangle.js2 SmartWatch.
#include <math.h>
#include <stdarg.h>
#include <nrf_assert.h>
#include <nrf_delay.h>
#include <nrf_drv_gpiote.h>
#include <nrf_atomic.h>
#include <ble.h>
#include <ble_conn_state.h>
#include <ble_db_discovery.h>
#include <ble_hrs_c.h>
#include <ble_bas_c.h>
#include <app_timer.h>
#include <boards.h>
#include <fds.h>
#include <fstorage.h>
#include <peer_manager.h>
#include <nrf_ble_gatt.h>
#include <softdevice_handler.h>
#include <core/mri.h>
#include "SAADCScanner/SAADCScanner.h"
#include "ColorMemLCD/ColorMemLCD.h"
#include "BufferLog/BufferLog.h"
#include "Bitmaps/HeartBitmap.h"
#include "Bitmaps/BluetoothLargeBitmap.h"
#include "Bitmaps/BluetoothSmallBitmap.h"
#include "Bitmaps/BatteryBitmap.h"
#include "Bitmaps/BatterySmallBitmap.h"
#include "HeartAnimation/HeartAnimation.h"
#include "CircularBuffer/CircularBuffer.h"

// Bangle.js2 analog battery voltage is connected to this pin through a 1/4 resistor divider.
static const uint8_t BATTERY_VOLTAGE_PIN = 3;

// Bangle.js2 connections to the transreflective LCD display.
static const uint8_t LCD_SCLK_PIN = 26;
static const uint8_t LCD_SI_PIN = 27;
static const uint8_t LCD_SCS_PIN = 5;
static const uint8_t LCD_EXTCOMIN_PIN = 6;
static const uint8_t LCD_DISP_PIN = 7;
static const uint8_t LCD_BACKLIGHT_PIN = 8;

// Bangle.js2 connection to the single button on the right side of the watch.
static const uint8_t BUTTON_PIN = 17;


// UNDONE: These are my mappings for running the code on the nRF52840 DK.
#ifdef UNDONE
// Bangle.js2 analog battery voltage is connected to this pin through a 1/4 resistor divider.
static const uint8_t BATTERY_VOLTAGE_PIN = 3;
// Bangle.js2 connections to the transreflective LCD display.
static const uint8_t LCD_SCLK_PIN = 26;
static const uint8_t LCD_SI_PIN = 27;
static const uint8_t LCD_SCS_PIN = 28;
static const uint8_t LCD_EXTCOMIN_PIN = 29;
static const uint8_t LCD_DISP_PIN = 30;
static const uint8_t LCD_BACKLIGHT_PIN = 31;
// Bangle.js2 connection to the single button on the right side of the watch.
static const uint8_t BUTTON_PIN = 11;
#endif // UNDONE


// The Bangle.js2 has required passives for enabling the lower power DC to DC converter.
#define WATCH_HAS_DCDC_PASSIVES true


// The SPIM peripheral instance to use for communicating with the LCD. Note that SPIM instances share the same
// resources as TWI (I2C).
#define LCD_SPI_INSTANCE    0

// The PWM peripheral instance to use for pulsing the LCD_EXTCOMIN LCD signal input.
#define LCD_PWM_INSTANCE    0


// ******************************************************************************************************
// BLE based Heart Rate Monitor Client code based on similar sample in Nordic's nRF5 SDK.
// ******************************************************************************************************
// The number of central and peripheral links to be supported are configured in sdk_config.h using the
// NRF_BLE_CENTRAL_LINK_COUNT and NRF_BLE_PERIPHERAL_LINK_COUNT macros.


// Value of the RTC1 PRESCALER register.
#define APP_TIMER_PRESCALER             0
// Size of timer operation queues.
#define APP_TIMER_OP_QUEUE_SIZE         3

// Sample the watch battery once a minute.
#define BATTERY_READ_TICKS              APP_TIMER_TICKS(60000u, APP_TIMER_PRESCALER)

// Debounce the right side watch button by ignoring subsequent presses for 500ms.
#define BUTTON_DEBOUNCE_TICKS           APP_TIMER_TICKS(500U, APP_TIMER_PRESCALER)


// Security GAP parameters.
// Perform bonding.
#define SEC_PARAM_BOND                  1
// Man In The Middle protection required (applicable when display module is detected).
#define SEC_PARAM_MITM                  0
// LE Secure Connections not enabled.
#define SEC_PARAM_LESC                  0
// Keypress notifications not enabled.
#define SEC_PARAM_KEYPRESS              0
// Display I/O capabilities.
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE
// Out Of Band data not available.
#define SEC_PARAM_OOB                   0
/// Minimum encryption key size.
#define SEC_PARAM_MIN_KEY_SIZE          7
// Maximum encryption key size.
#define SEC_PARAM_MAX_KEY_SIZE          16

// Parameters for configuring how this application scans for nearby peripherals.
// Scan interval between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s). 0xA0 is equal to 100 milliseconds.
// The central device will switch between advertising channels at this period.
#define SCAN_INTERVAL           0x00A0
// Scan window between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s). 0x50 is equal to 50 milliseconds.
// The central device will actively scan a single advertising channel for this amount of time.
// Must be <= SCAN_INTERVAL. No scanning is performed for (SCAN_INTERVAL - SCAN_WINDOW) amount of time.
#define SCAN_WINDOW             0x0050

// Timings for how often peripheral and central should communicate over the link. The shorter the interval, the lower
// the link latency but at the cost of higher power usage.
// Apple's Accessory Design Guidelines places some extra constraints on these parameters:
// * Peripheral Latency of up to 30 connection intervals.
// * Supervision Timeout from 2 seconds to 6 seconds.
// * Interval Min of at least 15 ms.
// * Interval Min is a multiple of 15 ms.
// * One of the following:
//   * Interval Max at least 15 ms greater than Interval Min.
//    * Interval Max and Interval Min both set to 15 ms.
// * Interval Max * (Peripheral Latency + 1) of 2 seconds or less.
// * Supervision Timeout greater than Interval Max * (Peripheral Latency + 1) * 3.
// Minimum acceptable connection interval in ms (7.5 ms is smallest allowed). Connection interval uses 1.25 ms units.
#define MIN_CONNECTION_INTERVAL     MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
// Maximum acceptable connection interval in ms. Connection interval uses 1.25 ms units.
#define MAX_CONNECTION_INTERVAL     MSEC_TO_UNITS(30, UNIT_1_25_MS)
// The number of the connection interval events that the peripheral can ignore before response is required.
#define SLAVE_LATENCY               0
// Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
#define SUPERVISION_TIMEOUT         MSEC_TO_UNITS(4000, UNIT_10_MS)


// Structure used to pass variable length data buffers into parseAdvertisementData().
typedef struct
{
    uint8_t* pData;
    uint16_t dataLength;
} SizedData;


// Analog to digital converter object used to read watch battery voltage.
static SAADCScanner         g_adc(NRF_SAADC_RESOLUTION_12BIT, false, _PRIO_APP_LOWEST);

// Battery voltage is read using this ADC channel.
SAADCScanner::Channel*      g_pBatteryVoltageChannel = NULL;

// LCD display.
static const nrf_drv_spi_t  g_lcdSpiInstance = NRF_DRV_SPI_INSTANCE(LCD_SPI_INSTANCE);
static nrf_drv_pwm_t        g_lcdPwmInstance = NRF_DRV_PWM_INSTANCE(LCD_PWM_INSTANCE);
ColorMemLCD                 g_lcd(&g_lcdSpiInstance, &g_lcdPwmInstance,
                                  LCD_SCLK_PIN, LCD_SI_PIN, LCD_SCS_PIN,
                                  LCD_EXTCOMIN_PIN, LCD_DISP_PIN, LCD_BACKLIGHT_PIN);

// Heart animation object.
static HeartAnimation       g_heartAnimation(0.2f, 0.5f, 0.05f);

// This counter is incremented each time a UI component is update and the screen should be updated.
static volatile uint32_t    g_uiUpdates = 1;

// Globals used to communicate state between Heart Rate Monitor (hrm) and main loop.
static volatile uint16_t    g_hrmConnectionHandle = BLE_CONN_HANDLE_INVALID;
static volatile bool        g_isHrmConnected = false;
static volatile uint32_t    g_heartRate = 0;
static CircularBuffer<uint16_t, 5, true> g_rrIntervals;

// Latest watch battery voltage.
static volatile float       g_watchBatteryVoltage = 0.0f;

// Latest heart rate monitor voltage level (%).
static volatile uint8_t     g_hrmBatteryLevel = 0;

// Global BLE modules.
static ble_db_discovery_t   g_bleDbDiscovery;
static ble_hrs_c_t          g_bleHeartRateClient;
static ble_bas_c_t          g_bleBatteryClient;
static ble_gap_scan_params_t g_bleScanParameters;
static nrf_ble_gatt_t       g_bleGatt;

// Globals to track state within application's BLE stack.
static bool                 g_isWhitelistDisabled;
static bool                 g_isFlashAccessInProgress;

// If first call to ble_db_discovery_start() fails when connection is first made, then retry it later.
static bool                 g_retryDbDiscovery;
static uint16_t             g_bleConnectionToRetryDbDiscovery = BLE_CONN_HANDLE_INVALID;

// App Timer Instances.
// Timer used to sample the watch battery on a regular basis.
APP_TIMER_DEF(g_batteryReadTimer);
// Timer used to debounce watch button presses.
APP_TIMER_DEF(g_buttonDebounceTimer);
// Timer used to animate things like the heart throb.
APP_TIMER_DEF(g_animationTimer);

// Connection timing parameters requested for BLE connection.
static const ble_gap_conn_params_t g_bleConnectionParameters =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};

// When scanning for a peripheral to which a connection should be made, you can use one of 3 methods to find a match
// based on the data advertised by peripherals in the vicinity of this BLE central device:
// 1. The name of the Heart Rate Monitor to which a connection should be made.
//     Empty string if don't want to search by name.
static const char g_targetPeripheralName[] = "";
// 2. A specific device address.
//    Set g_useTargetPeripheralAddress to true if you want to use the address in g_targetPeripheralAddress.
static bool                 g_useTargetPeripheralAddress = false;
static const ble_gap_addr_t g_targetPeripheralAddress =
{
    // Possible values for addr_type:
    //   BLE_GAP_ADDR_TYPE_PUBLIC,
    //   BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
    //   BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE,
    //   BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE.
    .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
    .addr      = {0x8D, 0xFE, 0x23, 0x86, 0x77, 0xD9}
};
// 3. The UUID of the BLE Heart Rate Service.
#define TARGET_UUID         BLE_UUID_HEART_RATE_SERVICE





// UNDONE: Don't really want to go to sleep on Watch since I haven't made sure that I can wake it up yet.
#ifdef UNDONE
/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t errorCode = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(errorCode);

    // Prepare wakeup buttons.
    errorCode = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(errorCode);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    errorCode = sd_power_system_off();
    APP_ERROR_CHECK(errorCode);
}
#endif // UNDONE





// Forward Function Declarations
static void initLCD();
static void initTimers();
static void handleBatteryTimer(void* pvContext);
static void handleButtonDebounceTimer(void* pvContext);
static void handleAnimationTimer(void* pvContext);
static void initBleStack();
static void initBLE(void);
static void dispatchBleEvents(ble_evt_t* pBleEvent);
static void handleBleEvent(ble_evt_t* pBleEvent);
static bool findPeerAddressInAdvertisementData(const ble_gap_evt_adv_report_t* pAdvertisementReport,
                                               const ble_gap_addr_t* pAddressOut);
static bool findNameInAdvertisementData(const ble_gap_evt_adv_report_t* pAdvertisementReport, const char* pNameToFind);
static uint32_t parseAdvertisementData(uint8_t type, SizedData* pAdvertisementData, SizedData* pTypeData);
static bool findUuidInAdvertisementData(const ble_gap_evt_adv_report_t* pAdvertisementReport, const uint16_t uuidToFind);
static uint16_t extractUUID16(const uint8_t* pSrc);
static void dispatchSysEvents(uint32_t sysEvent);
static void handleSysEvents(uint32_t sysEvent);
static void initPeerManager(bool eraseBonds);
static void handlePeerManagerEvents(pm_evt_t const* pGattEvent);
static void initGATT();
static void handleGattEvent(nrf_ble_gatt_t* pGATT, nrf_ble_gatt_evt_t* pGattEvent);
static void initDbDiscoveryModule();
static void handleDbDiscoveryEvent(ble_db_discovery_evt_t* pGattEvent);
static void initHeartRateService();
static void handleHeartRateServiceEvent(ble_hrs_c_t* pHeartRate, ble_hrs_c_evt_t* pHeartRateEvent);
static void initBatteryService();
static void handleBatteryServiceEvent(ble_bas_c_t * pBatteryService, ble_bas_c_evt_t * pBatteryServiceEvent);
static void initBatteryVoltageReading();
static float readBatteryVoltage();
static void initButtonInterrupt();
static void handleButtonPress(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void stopBLE();
static void startBLE();
static void startTimers();
static void stopTimers();
static void startScanningForHeartRateMonitor();
static void loadWhitelist();
static void getPeerList(pm_peer_id_t* pPeers, uint32_t* peerCount);
static void sleepUntilNextEvent();
static void clearPendingFpuExceptions();
static void checkForErroneousPendingInterrupts();
static void updateLCD();


int main(void)
{
    logPrintF("Starting up...\n");

    initLCD();
    initTimers();
    initBleStack();
    initBatteryVoltageReading();
    initButtonInterrupt();
    startTimers();

    // Start scanning for peripherals and initiate connection with devices that advertise Heart Rate UUID.
    logPrintF("Heart Rate Example.\n");
    startScanningForHeartRateMonitor();

    // Main UI Loop.
    for (;;)
    {
        sleepUntilNextEvent();
        updateLCD();
    }
}

static void initLCD()
{
    g_lcd.init();
    g_lcd.turnOn();
    g_lcd.setTextSize(2);
    bool result = g_heartAnimation.init();
    ASSERT ( result );
}

static void initTimers()
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    // Create the timer used to read the watch battery voltage on a regular basis.
    ret_code_t errorCode = app_timer_create(&g_batteryReadTimer, APP_TIMER_MODE_REPEATED, handleBatteryTimer);
    APP_ERROR_CHECK(errorCode);

    // Create the timer used to debounce watch button presses by ignoring subsequent presses for some time before
    // re-enabling button press interrupts.
    errorCode = app_timer_create(&g_buttonDebounceTimer, APP_TIMER_MODE_SINGLE_SHOT, handleButtonDebounceTimer);
    APP_ERROR_CHECK(errorCode);

    // Create the timer used to display frames at the correct time.
    errorCode = app_timer_create(&g_animationTimer, APP_TIMER_MODE_SINGLE_SHOT, handleAnimationTimer);
    APP_ERROR_CHECK(errorCode);
}

static void handleBatteryTimer(void* pvContext)
{
    g_watchBatteryVoltage = readBatteryVoltage();
    nrf_atomic_u32_add(&g_uiUpdates, 1);
    logPrintF("Vbatt=%.1f\n", g_watchBatteryVoltage + 0.05f);
}

static void handleButtonDebounceTimer(void* pvContext)
{
    // Re-enable the GPIOTE interrupt for button presses.
    nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
}

static void handleAnimationTimer(void* pvContext)
{
    // Let main loop know that there is something on the display to be updated.
    nrf_atomic_u32_add(&g_uiUpdates, 1);
}
static void initBleStack()
{
    initBLE();

    // UNDONE: Need way do this on the watch itself.
    bool eraseBonds = false;
    initPeerManager(eraseBonds);
    if (eraseBonds == true)
    {
        logPrintF("Bonds erased!\n");
    }

    initGATT();
    initDbDiscoveryModule();
    initHeartRateService();
    initBatteryService();
}

static void initBLE()
{
    uint32_t errorCode;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t bleEnableParams;
    errorCode = softdevice_enable_get_default_config(NRF_BLE_CENTRAL_LINK_COUNT,
                                                     NRF_BLE_PERIPHERAL_LINK_COUNT,
                                                     &bleEnableParams);
    APP_ERROR_CHECK(errorCode);

    // Verify that the start of RAM in nr52.ld leaves enough room for the softdevice when configured for the desired
    // number of central and peripheral links.
    CHECK_RAM_START_ADDR(NRF_BLE_CENTRAL_LINK_COUNT, NRF_BLE_PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    bleEnableParams.gatt_enable_params.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
#endif

    errorCode = softdevice_enable(&bleEnableParams);
    APP_ERROR_CHECK(errorCode);

    // Register with the SoftDevice handler module for BLE events.
    errorCode = softdevice_ble_evt_handler_set(dispatchBleEvents);
    APP_ERROR_CHECK(errorCode);

    // Register with the SoftDevice handler module for System events.
    errorCode = softdevice_sys_evt_handler_set(dispatchSysEvents);
    APP_ERROR_CHECK(errorCode);

    // For now it is ok to have a longer latency out of sleep mode so enable low power sleep mode.
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    APP_ERROR_CHECK(errorCode);

    // Enable the DC to DC converter if possible as it reduced current consumption. Requires external components that
    // the Bangle.js2 has installed.
    if (WATCH_HAS_DCDC_PASSIVES)
    {
        errorCode = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(errorCode);
    }
}

static void dispatchBleEvents(ble_evt_t* pBleEvent)
{
    // Call ble_conn_state_on_ble_evt() before other functions that depend on ble_conn_state, like Peer Manager.
    ble_conn_state_on_ble_evt(pBleEvent);
    pm_on_ble_evt(pBleEvent);
    ble_db_discovery_on_ble_evt(&g_bleDbDiscovery, pBleEvent);
    ble_hrs_c_on_ble_evt(&g_bleHeartRateClient, pBleEvent);
    ble_bas_c_on_ble_evt(&g_bleBatteryClient, pBleEvent);
    nrf_ble_gatt_on_ble_evt(&g_bleGatt, pBleEvent);
    handleBleEvent(pBleEvent);
}

static void handleBleEvent(ble_evt_t* pBleEvent)
{
    uint32_t                errorCode;
    const ble_gap_evt_t   * pBleGapEvent = &pBleEvent->evt.gap_evt;

    switch (pBleEvent->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            logPrintF("Connected.\n");
            g_hrmConnectionHandle = pBleEvent->evt.gap_evt.conn_handle;

            // Discover services supported by this peripheral.
            g_bleConnectionToRetryDbDiscovery = pBleEvent->evt.gap_evt.conn_handle;
            g_retryDbDiscovery = false;
            errorCode = ble_db_discovery_start(&g_bleDbDiscovery, g_bleConnectionToRetryDbDiscovery);
            if (errorCode == NRF_ERROR_BUSY)
            {
                logPrintF("ble_db_discovery_start() returned busy, will retry later.\n");
                g_retryDbDiscovery = true;
            }
            else
            {
                APP_ERROR_CHECK(errorCode);
            }

#ifdef UNDONE
            errorCode = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(errorCode);
#endif // UNDONE

            // If can still connect to more peripherals, then continue scanning for them.
            if (ble_conn_state_n_centrals() < NRF_BLE_CENTRAL_LINK_COUNT)
            {
                startScanningForHeartRateMonitor();
            }
        } break;

        case BLE_GAP_EVT_ADV_REPORT:
        {
            // UNDONE: The adv_report also has RSSI that can be used.
            //         Hopefully we can store the peer_addr along with RSSI, name, etc to display to user and
            //         call sd_ble_gap_connect() later.
            // Look at the advertised data from this peripheral to see if it one of the devices that we want to connect
            // to based on name, address, or supported UUIDs.
            bool isSupportedPeripheral = false;
            if (g_useTargetPeripheralAddress)
            {
                if (findPeerAddressInAdvertisementData(&pBleGapEvent->params.adv_report, &g_targetPeripheralAddress))
                {
                    logPrintF("Address match. Send connect request.\n");
                    isSupportedPeripheral = true;
                }
            }
            else if (strlen(g_targetPeripheralName) != 0)
            {
                if (findNameInAdvertisementData(&pBleGapEvent->params.adv_report, g_targetPeripheralName))
                {
                    isSupportedPeripheral = true;
                    logPrintF("Name match. Send connect request.\n");
                }
            }
            else
            {
                if (findUuidInAdvertisementData(&pBleGapEvent->params.adv_report, TARGET_UUID))
                {
                    isSupportedPeripheral = true;
                    logPrintF("UUID match. Send connect request.\n");
                }
            }

            // If we found a supported peripheral then try to connect to it.
            if (isSupportedPeripheral)
            {
                // Stop scanning.
                (void) sd_ble_gap_scan_stop();

                #if (NRF_SD_BLE_API_VERSION == 2)
                    g_bleScanParameters.selective = 0;
                #endif
                #if (NRF_SD_BLE_API_VERSION == 3)
                    g_bleScanParameters.use_whitelist = 0;
                #endif

                // Initiate connection.
                errorCode = sd_ble_gap_connect(&pBleGapEvent->params.adv_report.peer_addr,
                                              &g_bleScanParameters,
                                              &g_bleConnectionParameters);

                g_isWhitelistDisabled = false;

                if (errorCode != NRF_SUCCESS)
                {
                    logPrintF("Connection Request Failed, reason %ld.\n", errorCode);
                }
            }
        } break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_DISCONNECTED:
        {
            logPrintF("Disconnected, reason 0x%x.\n",
                         pBleEvent->evt.gap_evt.params.disconnected.reason);
            g_isHrmConnected = false;
            nrf_atomic_u32_add(&g_uiUpdates, 1);
            g_hrmConnectionHandle = BLE_CONN_HANDLE_INVALID;
#ifdef UNDONE
            errorCode = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(errorCode);
#endif // UNDONE

            // Reset DB discovery structure.
            memset(&g_bleDbDiscovery, 0 , sizeof (g_bleDbDiscovery));

            // Can start scanning for a new device to connect to now that this one has disconnected.
            // Only start scanning if the LCD is on though.
            if (g_lcd.isOn() && ble_conn_state_n_centrals() < NRF_BLE_CENTRAL_LINK_COUNT)
            {
                startScanningForHeartRateMonitor();
            }
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            if (pBleGapEvent->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                logPrintF("Scan timed out. Trying again...\n");
                startScanningForHeartRateMonitor();
            }
            else if (pBleGapEvent->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                logPrintF("Connection Request timed out.\n");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            errorCode = sd_ble_gap_conn_param_update(pBleGapEvent->conn_handle,
                                                    &pBleGapEvent->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(errorCode);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            logPrintF("GATT Client Timeout.\n");
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            logPrintF("GATT Server Timeout.\n");
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            break;

        default:
            break;
    }
}

static bool findPeerAddressInAdvertisementData(const ble_gap_evt_adv_report_t* pAdvertisementReport,
                                               const ble_gap_addr_t* pAddressToFind)
{
    if (pAddressToFind->addr_type == pAdvertisementReport->peer_addr.addr_type)
    {
        if (memcmp(pAddressToFind->addr, pAdvertisementReport->peer_addr.addr, sizeof(pAdvertisementReport->peer_addr.addr)) == 0)
        {
            return true;
        }
    }
    return false;
}

static bool findNameInAdvertisementData(const ble_gap_evt_adv_report_t* pAdvertisementReport, const char* pNameToFind)
{
    SizedData   deviceName;

    // Initialize advertisement report for parsing
    SizedData advertisementData = { .pData = (uint8_t *)pAdvertisementReport->data,
                                    .dataLength = pAdvertisementReport->dlen };

    //search for advertising names
    ret_code_t errorCode = parseAdvertisementData(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                                  &advertisementData,
                                                  &deviceName);
    if (errorCode == NRF_SUCCESS)
    {
        if (memcmp(pNameToFind, deviceName.pData, deviceName.dataLength)== 0)
        {
            return true;
        }
    }
    else
    {
        // Look for the short local name if it was not found as complete
        errorCode = parseAdvertisementData(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                           &advertisementData,
                                           &deviceName);
        if (errorCode != NRF_SUCCESS)
        {
            return false;
        }
        if (memcmp(g_targetPeripheralName, deviceName.pData, deviceName.dataLength )== 0)
        {
            return true;
        }
    }
    return false;
}

static uint32_t parseAdvertisementData(uint8_t type, SizedData* pAdvertisementData, SizedData* pTypeData)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = pAdvertisementData->pData;

    while (index < pAdvertisementData->dataLength)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            pTypeData->pData   = &p_data[index + 2];
            pTypeData->dataLength = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

static bool findUuidInAdvertisementData(const ble_gap_evt_adv_report_t* pAdvertisementReport, const uint16_t uuidToFind)
{
    uint32_t    errorCode;
    SizedData   advertisementData;
    SizedData   typeData;

    // Initialize advertisement report for parsing.
    advertisementData.pData = (uint8_t *)pAdvertisementReport->data;
    advertisementData.dataLength = pAdvertisementReport->dlen;

    errorCode = parseAdvertisementData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                                &advertisementData,
                                &typeData);

    if (errorCode != NRF_SUCCESS)
    {
        // Look for the services in 'complete' if it was not found in 'more available'.
        errorCode = parseAdvertisementData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                     &advertisementData,
                                     &typeData);

        if (errorCode != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit.
            return false;
        }
    }

    // Verify if any UUID match the given UUID.
    for (uint32_t i = 0; i < (typeData.dataLength / sizeof(uint16_t)); i++)
    {
        uint16_t extractedUUID = extractUUID16(&typeData.pData[i * sizeof(uint16_t)]);
        if (extractedUUID == uuidToFind)
        {
            return true;
        }
    }
    return false;
}

static uint16_t extractUUID16(const uint8_t* pSrc)
{
    return (pSrc[1] << 8) | pSrc[0];
}

static void dispatchSysEvents(uint32_t sysEvent)
{
    fs_sys_event_handler(sysEvent);
    handleSysEvents(sysEvent);
}

static void handleSysEvents(uint32_t sysEvent)
{
    switch (sysEvent)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */
        case NRF_EVT_FLASH_OPERATION_ERROR:

            if (g_isFlashAccessInProgress)
            {
                g_isFlashAccessInProgress = false;
                startScanningForHeartRateMonitor();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void initPeerManager(bool eraseBonds)
{
    ble_gap_sec_params_t secParams;
    ret_code_t errorCode;

    errorCode = pm_init();
    APP_ERROR_CHECK(errorCode);

    if (eraseBonds)
    {
        errorCode = pm_peers_delete();
        APP_ERROR_CHECK(errorCode);
    }

    // Security parameters to be used for all security procedures.
    memset(&secParams, 0, sizeof(ble_gap_sec_params_t));
    secParams.bond              = SEC_PARAM_BOND;
    secParams.mitm              = SEC_PARAM_MITM;
    secParams.lesc              = SEC_PARAM_LESC;
    secParams.keypress          = SEC_PARAM_KEYPRESS;
    secParams.io_caps           = SEC_PARAM_IO_CAPABILITIES;
    secParams.oob               = SEC_PARAM_OOB;
    secParams.min_key_size      = SEC_PARAM_MIN_KEY_SIZE;
    secParams.max_key_size      = SEC_PARAM_MAX_KEY_SIZE;
    secParams.kdist_own.enc     = 1;
    secParams.kdist_own.id      = 1;
    secParams.kdist_peer.enc    = 1;
    secParams.kdist_peer.id     = 1;

    errorCode = pm_sec_params_set(&secParams);
    APP_ERROR_CHECK(errorCode);

    errorCode = pm_register(handlePeerManagerEvents);
    APP_ERROR_CHECK(errorCode);
}

static void handlePeerManagerEvents(pm_evt_t const* pGattEvent)
{
    ret_code_t errorCode;

    switch (pGattEvent->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            logPrintF("Connected to a previously bonded device.\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            logPrintF("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\n",
                         ble_conn_state_role(pGattEvent->conn_handle),
                         pGattEvent->conn_handle,
                         pGattEvent->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(pGattEvent->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            errorCode = fds_gc();
            if (errorCode == FDS_ERR_BUSY || errorCode == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(errorCode);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            startScanningForHeartRateMonitor();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(pGattEvent->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(pGattEvent->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(pGattEvent->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(pGattEvent->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

static void initGATT()
{
    ret_code_t errorCode = nrf_ble_gatt_init(&g_bleGatt, handleGattEvent);
    APP_ERROR_CHECK(errorCode);
}

static void handleGattEvent(nrf_ble_gatt_t* pGATT, nrf_ble_gatt_evt_t* pGattEvent)
{
    logPrintF("GATT MTU on link %d changed to %d\n",
                 pGattEvent->conn_handle,
                 pGattEvent->att_mtu_effective);
    if (g_retryDbDiscovery)
    {
        logPrintF("Retrying DB discovery.\n");

        // Discover peer's services.
        g_retryDbDiscovery = false;
        ret_code_t errorCode = ble_db_discovery_start(&g_bleDbDiscovery, g_bleConnectionToRetryDbDiscovery);
        if (errorCode == NRF_ERROR_BUSY)
        {
            logPrintF("ble_db_discovery_start() returned busy, will retry later.\n");
            g_retryDbDiscovery = true;
        }
        else
        {
            APP_ERROR_CHECK(errorCode);
        }
    }
}

static void initDbDiscoveryModule()
{
    uint32_t errorCode = ble_db_discovery_init(handleDbDiscoveryEvent);
    APP_ERROR_CHECK(errorCode);
}

static void handleDbDiscoveryEvent(ble_db_discovery_evt_t* pGattEvent)
{
    ble_hrs_on_db_disc_evt(&g_bleHeartRateClient, pGattEvent);
    ble_bas_on_db_disc_evt(&g_bleBatteryClient, pGattEvent);
}

static void initHeartRateService()
{
    ble_hrs_c_init_t hrsInitObject = { .evt_handler = handleHeartRateServiceEvent };

    uint32_t errorCode = ble_hrs_c_init(&g_bleHeartRateClient, &hrsInitObject);
    APP_ERROR_CHECK(errorCode);
}

static void handleHeartRateServiceEvent(ble_hrs_c_t* pHeartRate, ble_hrs_c_evt_t* pHeartRateEvent)
{
    uint32_t errorCode;

    switch (pHeartRateEvent->evt_type)
    {
        case BLE_HRS_C_EVT_DISCOVERY_COMPLETE:
            errorCode = ble_hrs_c_handles_assign(pHeartRate ,
                                                pHeartRateEvent->conn_handle,
                                                &pHeartRateEvent->params.peer_db);
            APP_ERROR_CHECK(errorCode);

            // Initiate bonding.
            errorCode = pm_conn_secure(pHeartRateEvent->conn_handle, false);
            if (errorCode != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(errorCode);
            }

            // Heart rate service discovered. Enable notification of Heart Rate Measurement.
            errorCode = ble_hrs_c_hrm_notif_enable(pHeartRate);
            APP_ERROR_CHECK(errorCode);

            logPrintF("Heart rate service discovered.\n");
            break;

        case BLE_HRS_C_EVT_HRM_NOTIFICATION:
        {
            // Save away this most recent heart rate measurement.
            g_isHrmConnected = true;
            g_heartRate = pHeartRateEvent->params.hrm.hr_value;
            nrf_atomic_u32_add(&g_uiUpdates, 1);
            logPrintF("Heart Rate = %d.\n", pHeartRateEvent->params.hrm.hr_value);
            for (int i = 0; i < pHeartRateEvent->params.hrm.rr_intervals_cnt; i++)
            {
                uint16_t currInterval = pHeartRateEvent->params.hrm.rr_intervals[i];
                logPrintF("rr_interval = %d.\n", currInterval);

                // Ignore any readings that are so different from current BPM that they cause weird animation hiccups.
                int32_t bpmFromInterval = 60000 / (int32_t)currInterval;
                if (g_heartRate > 0 && abs((int32_t)g_heartRate - bpmFromInterval) < 20)
                {
                    g_rrIntervals.write(pHeartRateEvent->params.hrm.rr_intervals[i]);
                }
            }
            break;
        }

        default:
            break;
    }
}

static void initBatteryService()
{
    ble_bas_c_init_t basInitObject = { .evt_handler = handleBatteryServiceEvent };

    uint32_t errorCode = ble_bas_c_init(&g_bleBatteryClient, &basInitObject);
    APP_ERROR_CHECK(errorCode);
}

static void handleBatteryServiceEvent(ble_bas_c_t* pBatteryService, ble_bas_c_evt_t* pBatteryServiceEvent)
{
    uint32_t errorCode;

    switch (pBatteryServiceEvent->evt_type)
    {
        case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
        {
            errorCode = ble_bas_c_handles_assign(pBatteryService,
                                                pBatteryServiceEvent->conn_handle,
                                                &pBatteryServiceEvent->params.bas_db);
            APP_ERROR_CHECK(errorCode);

            // Initiate bonding.
            errorCode = pm_conn_secure(pBatteryServiceEvent->conn_handle, false);
            if (errorCode != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(errorCode);
            }

            // Batttery service discovered. Enable notification of Battery Level.
            logPrintF("Battery Service discovered. Reading battery level.\n");

            errorCode = ble_bas_c_bl_read(pBatteryService);
            APP_ERROR_CHECK(errorCode);

            logPrintF("Enabling Battery Level Notification.\n");
            errorCode = ble_bas_c_bl_notif_enable(pBatteryService);
            APP_ERROR_CHECK(errorCode);

        } break;

        case BLE_BAS_C_EVT_BATT_NOTIFICATION:
            logPrintF("Battery Level received %d %%.\n", pBatteryServiceEvent->params.battery_level);
            g_hrmBatteryLevel = pBatteryServiceEvent->params.battery_level;
            nrf_atomic_u32_add(&g_uiUpdates, 1);
            break;

        case BLE_BAS_C_EVT_BATT_READ_RESP:
            logPrintF("Battery Level Read as %d %%.\n", pBatteryServiceEvent->params.battery_level);
            g_hrmBatteryLevel = pBatteryServiceEvent->params.battery_level;
            nrf_atomic_u32_add(&g_uiUpdates, 1);
            break;

        default:
            break;
    }
}

static void initBatteryVoltageReading()
{
    // Initialize the ADC object which scans all of the configured ADC channels manually.
    bool result = g_adc.init();
    ASSERT ( result );
    g_pBatteryVoltageChannel = g_adc.addChannel(BATTERY_VOLTAGE_PIN,
                                                NRF_SAADC_RESISTOR_DISABLED,
                                                NRF_SAADC_GAIN1_4,
                                                NRF_SAADC_REFERENCE_VDD4,
                                                NRF_SAADC_ACQTIME_3US,
                                                SAADC_LOWER_LIMIT_DISABLED,
                                                SAADC_UPPER_LIMIT_DISABLED,
                                                NULL);
    ASSERT ( g_pBatteryVoltageChannel != NULL );
    g_adc.startScanning();
}

static float readBatteryVoltage()
{
    // Read the battery voltage.
    ASSERT ( g_pBatteryVoltageChannel->getSampleCount() > 0 );
    SAADCScanner::Channel::Reading reading = g_pBatteryVoltageChannel->read();
    float batteryVoltage = 3.3f * 4.0f * reading.mean / 4095.0f;

    // Kick off the next ADC sampling cycle so that the sample is ready by the time it is needed.
    g_adc.startScanning();

    return batteryVoltage;
}

static void initButtonInterrupt()
{
    // Configure button on right side of watch to generate interrupt when it is pressed.
    nrf_drv_gpiote_in_config_t gpioteConfig =
    {
        .sense = NRF_GPIOTE_POLARITY_HITOLO,
        .pull = NRF_GPIO_PIN_PULLUP,
        .is_watcher = false,
        .hi_accuracy = false
    };
    uint32_t errorCode = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(errorCode);

    errorCode = nrf_drv_gpiote_in_init(BUTTON_PIN, &gpioteConfig, handleButtonPress);
    APP_ERROR_CHECK(errorCode);

    nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
}

static void handleButtonPress(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // Disable the button sensing and re-enable it from a timer later.
    nrf_drv_gpiote_in_event_disable(BUTTON_PIN);
    ret_code_t errorCode = app_timer_start(g_buttonDebounceTimer, BUTTON_DEBOUNCE_TICKS, NULL);
    APP_ERROR_CHECK(errorCode);

    if (g_lcd.isOn())
    {
        g_lcd.turnOff();
        stopTimers();
        stopBLE();
    }
    else
    {
        g_lcd.turnOn();
        nrf_atomic_u32_add(&g_uiUpdates, 1);
        startBLE();
        startTimers();
    }
}

static void stopBLE()
{
    // User has asked for Watch to be turned off so stop BLE activity (connection or scanning).
    if (g_hrmConnectionHandle != BLE_CONN_HANDLE_INVALID)
    {
        // Are currently connected to the heart rate monitor so disconnect from it.
        // Normally the disconnect message would later cause the next scan to start but this has been disabled if
        // the LCD is turned off.
        (void) sd_ble_gap_disconnect(g_hrmConnectionHandle,  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    }
    else
    {
        // Hadn't found heart rate monitor yet so just stop scanning for it.
        (void) sd_ble_gap_scan_stop();
    }
}

static void startBLE()
{
    // User has asked for Watch to be turned back on.
    startScanningForHeartRateMonitor();
}

static void startTimers()
{
    // Perform initial watch battery voltage read now and then start timer to perform it in the future.
    handleBatteryTimer(NULL);
    ret_code_t errorCode = app_timer_start(g_batteryReadTimer, BATTERY_READ_TICKS, NULL);
    APP_ERROR_CHECK(errorCode);
}

static void stopTimers()
{
    ret_code_t errorCode = app_timer_stop(g_batteryReadTimer);
    APP_ERROR_CHECK(errorCode);
}

static void startScanningForHeartRateMonitor()
{
    // If there are any pending writes to FLASH, defer scanning until it completes.
    uint32_t isFlashBusy;
    (void) fs_queued_op_count_get(&isFlashBusy);
    if (isFlashBusy != 0)
    {
        g_isFlashAccessInProgress = true;
        return;
    }

    // Whitelist buffers.
    ble_gap_addr_t whitelistAddresses[8];
    ble_gap_irk_t  whitelistIRKs[8];
    memset(whitelistAddresses, 0x00, sizeof(whitelistAddresses));
    memset(whitelistIRKs,  0x00, sizeof(whitelistIRKs));

    uint32_t addressCount = (sizeof(whitelistAddresses) / sizeof(ble_gap_addr_t));
    uint32_t irkCount  = (sizeof(whitelistIRKs)  / sizeof(ble_gap_irk_t));

    #if (NRF_SD_BLE_API_VERSION == 2)

        ble_gap_addr_t* p_whitelist_addrs[8];
        ble_gap_irk_t*  p_whitelist_irks[8];

        for (uint32_t i = 0; i < 8; i++)
        {
            p_whitelist_addrs[i] = &whitelistAddresses[i];
            p_whitelist_irks[i]  = &whitelistIRKs[i];
        }

        ble_gap_whitelist_t whitelist =
        {
            .pp_addrs = p_whitelist_addrs,
            .pp_irks  = p_whitelist_irks,
        };

    #endif

    // Reload the whitelist and whitelist all peers.
    loadWhitelist();

    // Get the whitelist previously set using pm_whitelist_set().
    ret_code_t errorCode = pm_whitelist_get(whitelistAddresses, &addressCount,
                                            whitelistIRKs,  &irkCount);

    g_bleScanParameters.active   = 0;
    g_bleScanParameters.interval = SCAN_INTERVAL;
    g_bleScanParameters.window   = SCAN_WINDOW;

    if (((addressCount == 0) && (irkCount == 0)) ||
        (g_isWhitelistDisabled))
    {
        // Don't use whitelist.
        #if (NRF_SD_BLE_API_VERSION == 2)
            g_bleScanParameters.selective   = 0;
            g_bleScanParameters.p_whitelist = NULL;
        #endif
        #if (NRF_SD_BLE_API_VERSION == 3)
            g_bleScanParameters.use_whitelist  = 0;
            g_bleScanParameters.adv_dir_report = 0;
        #endif
        g_bleScanParameters.timeout  = 0x0000; // No timeout.
    }
    else
    {
        // Use whitelist.
        #if (NRF_SD_BLE_API_VERSION == 2)
            whitelist.addr_count     = addressCount;
            whitelist.irk_count      = irkCount;
            g_bleScanParameters.selective   = 1;
            g_bleScanParameters.p_whitelist = &whitelist;
        #endif
        #if (NRF_SD_BLE_API_VERSION == 3)
            g_bleScanParameters.use_whitelist  = 1;
            g_bleScanParameters.adv_dir_report = 0;
        #endif
        g_bleScanParameters.timeout  = 0x001E; // 30 seconds.
    }

    logPrintF("Starting scan.\n");

    errorCode = sd_ble_gap_scan_start(&g_bleScanParameters);
    APP_ERROR_CHECK(errorCode);

#ifdef UNDONE
    errorCode = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(errorCode);
#endif // UNDONE
}

static void loadWhitelist()
{
    pm_peer_id_t peers[8];
    memset(peers, PM_PEER_ID_INVALID, sizeof(peers));
    uint32_t peerCount = (sizeof(peers) / sizeof(pm_peer_id_t));

    // Load all peers from flash and whitelist them.
    getPeerList(peers, &peerCount);

    ret_code_t errorCode = pm_whitelist_set(peers, peerCount);
    APP_ERROR_CHECK(errorCode);

    // Setup the device identities list.
    // Some SoftDevices do not support this feature.
    errorCode = pm_device_identities_list_set(peers, peerCount);
    if (errorCode != NRF_ERROR_NOT_SUPPORTED)
    {
        APP_ERROR_CHECK(errorCode);
    }
}

static void getPeerList(pm_peer_id_t* pPeers, uint32_t* peerCount)
{
    uint32_t peersToCopy = (*peerCount < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ? *peerCount : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    *peerCount = 0;
    pm_peer_id_t peerID = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while ((peerID != PM_PEER_ID_INVALID) && (peersToCopy--))
    {
        pPeers[(*peerCount)++] = peerID;
        peerID = pm_next_peer_id_get(peerID);
    }
}

static void sleepUntilNextEvent()
{
    clearPendingFpuExceptions();

    uint32_t errorCode = sd_app_evt_wait();
    APP_ERROR_CHECK(errorCode);

    checkForErroneousPendingInterrupts();
}

static void clearPendingFpuExceptions()
{
    uint32_t fpscr = __get_FPSCR();
    __set_FPSCR(fpscr & ~0x9Fu);
    __DMB();
    NVIC_ClearPendingIRQ(FPU_IRQn);

    // Assert if a critical FPU exception is signaled.
    ASSERT((fpscr & 0x03) == 0);
}

static void checkForErroneousPendingInterrupts()
{
    // Are there random interrupts pending that are masked off but sd_app_evt_wait() thinks are important to app so
    // won't put device to sleep properly.
    const uint32_t interruptsUsedBySoftDevice = 0x4300f903;
    uint32_t ISPR0 = NVIC->ISPR[0] & ~interruptsUsedBySoftDevice;
    uint32_t ISPR1 = NVIC->ISPR[1] & ~(1 << (FPU_IRQn-32));
    if (ISPR0 || ISPR1)
    {
        __debugbreak();
    }
}

static void updateLCD()
{
    static uint32_t lastUiUpdate = 0;

    // Only want to send updates to LCD if something has changed and the LCD is on.
    uint32_t currUiUpdate = g_uiUpdates;
    if (!g_lcd.isOn() || currUiUpdate == lastUiUpdate)
    {
        return;
    }
    lastUiUpdate = currUiUpdate;

    // Prepare the LCD frame buffer for updating.
    g_lcd.waitForRefreshToComplete();
    g_lcd.fillScreen(ColorMemLCD::COLOR_WHITE);

    // General measurements of where things should be placed on the display.
    const uint32_t topMargin = 2;
    const uint32_t chargingBitmapWidth = 20;
    const uint32_t batteryIconXOffset = ColorMemLCD::DISP_WIDTH - g_batteryBitmapWidth - chargingBitmapWidth;
    const uint32_t bpmTopMargin = 5;
    const uint32_t throbbingHeartXCenter = 23;
    const uint32_t hrmBatteryRightMargin = 6;

    // UNDONE: Make these global constants at top of code.
    // Consider 4.2V full and 3.6V empty.
    const float maxVoltage = 4.2f;
    const float minVoltage = 3.6f;
    const float lowVoltage = 3.8f;
    const uint8_t lowHrmBatteryPercent = 20;

    // Calculate the battery capacity left based on voltage range. Cap it at 0%.
    float batteryVoltage = g_watchBatteryVoltage;
    float batteryPercent = (batteryVoltage - minVoltage) / (maxVoltage - minVoltage);
    if (batteryPercent < 0.0f)
    {
        batteryPercent = 0.0f;
    }
#ifdef UNDONE
    // UNDONE:
    logPrintF("batteryVoltage=%.2f\n", batteryVoltage);
    logPrintF("batteryPercent=%.2f\n", batteryPercent);
#endif // UNDONE

    // Set icon color based on how much battery is left: black for normal and red for low.
    uint16_t voltageColor = ColorMemLCD::COLOR_BLACK;
    if (batteryVoltage <= lowVoltage)
    {
        voltageColor = ColorMemLCD::COLOR_RED;
    }

    // Draw battery icon in required color.
    g_lcd.drawBitmap(batteryIconXOffset, topMargin - 1,
                     g_batteryBitmap, g_batteryBitmapWidth, g_batteryBitmapHeight, voltageColor);

    // Fill the inside of the battery icon with black and then place a rounded rect of white in the left portion of
    // the battery icon to indicate how much battery capacity is left.
    uint32_t rectWidth = batteryPercent * (g_batteryIconRectWidth - 8);
    g_lcd.fillRoundRect(batteryIconXOffset + 2, topMargin + 1,
                        g_batteryIconRectWidth - 4, g_batteryBitmapHeight - 4,
                        g_batteryIconRadius, ColorMemLCD::COLOR_BLACK);
    g_lcd.fillRoundRect(batteryIconXOffset + 4, topMargin + 3,
                        rectWidth, g_batteryBitmapHeight - 8,
                        g_batteryIconRadius, ColorMemLCD::COLOR_WHITE);


    // Output watch battery voltage on top of the battery icon.
    // Draw the 2 voltage characters separately and insert a narrow '.' between them to simulate proportional font.
    // Use XOR for drawing the voltage on top of the battery icon so that it will be black text on white fill or
    // white text on black fill. Use a canvas to draw the 'proportional' font so that it can be smoothed without
    // effecting the XOR operation since smoothing can touch some pixels multiple times.
    char buffer[3];
    snprintf(buffer, sizeof(buffer), "%2.0f", batteryVoltage * 10.0f);

    static GFXcanvas1 voltageCanvas(12+4+12, 16);
    voltageCanvas.fillScreen(0);
    voltageCanvas.drawChar(0, 0, buffer[0], 1, 1, 2);
    voltageCanvas.drawChar(7, 0, '.', 1, 1, 2);
    voltageCanvas.drawChar(12 + 4, 0, buffer[1], 1, 1, 2);

    g_lcd.enableXOR();
    g_lcd.drawBitmap(batteryIconXOffset + 8, topMargin+4,
                     voltageCanvas.getBuffer(), voltageCanvas.width(), voltageCanvas.height(), ColorMemLCD::COLOR_WHITE);
    g_lcd.disableXOR();

    // UNDONE: Add a charging icon if the USB power supply is connected and charging the battery.

    // Output BLE connection state or latest heart rate measurement.
    if (g_isHrmConnected)
    {
        const uint32_t charScale = 5;
        const uint32_t charWidth = charScale * 6;
        const uint32_t charHeight = charScale * 8;

        // Display the latest heart rate measurement.
        char buffer[4];
        int length = snprintf(buffer, sizeof(buffer), "%lu", g_heartRate);

        g_lcd.setTextSize(charScale);
        g_lcd.setTextColor(ColorMemLCD::COLOR_BLACK);
        g_lcd.setCursor(ColorMemLCD::DISP_WIDTH/2 - length * charWidth / 2,
                        topMargin + g_batteryBitmapHeight + bpmTopMargin);
        g_lcd.printf("%s", buffer);

        // Cap the battery percent at 100%.
        if (g_hrmBatteryLevel > 100)
        {
            g_hrmBatteryLevel = 100;
        }

        // Set fill color based on how much battery is left: green for normal and red for low.
        uint16_t voltageColor = ColorMemLCD::COLOR_GREEN;
        if (g_hrmBatteryLevel <= lowHrmBatteryPercent)
        {
            voltageColor = ColorMemLCD::COLOR_RED;
        }

        // Draw small battery icon in required color.
        g_lcd.drawBitmap(ColorMemLCD::DISP_WIDTH - g_batterySmallBitmapWidth - hrmBatteryRightMargin,
                         topMargin + g_batteryBitmapHeight + bpmTopMargin + charHeight/2 - g_batterySmallBitmapHeight/2 - 2,
                         g_batterySmallBitmap, g_batterySmallBitmapWidth, g_batterySmallBitmapHeight,
                         ColorMemLCD::COLOR_BLACK);

        // Fill the inside of the battery icon with black and then place a rounded rect of white in the left portion of
        // the battery icon to indicate how much battery capacity is left.
        uint32_t rectWidth = g_hrmBatteryLevel * (g_batterySmallIconRectWidth - 8) / 100;
        g_lcd.fillRoundRect(ColorMemLCD::DISP_WIDTH - g_batterySmallBitmapWidth - hrmBatteryRightMargin + 2,
                           topMargin + g_batteryBitmapHeight + bpmTopMargin + charHeight/2 - g_batterySmallBitmapHeight/2 - 2 + 2,
                           g_batterySmallIconRectWidth - 4, g_batterySmallBitmapHeight - 4,
                            g_batterySmallIconRadius, ColorMemLCD::COLOR_BLACK);
        g_lcd.fillRoundRect(ColorMemLCD::DISP_WIDTH - g_batterySmallBitmapWidth - hrmBatteryRightMargin + 4,
                            topMargin + g_batteryBitmapHeight + bpmTopMargin + charHeight/2 - g_batterySmallBitmapHeight/2 - 2 + 4,
                            rectWidth, g_batterySmallBitmapHeight - 8,
                            g_batterySmallIconRadius, voltageColor);

        if (g_heartRate > 0)
        {
            static uint16_t lastValidRRInterval = 1000/60;
            int32_t frameTime_ms = g_heartAnimation.drawAnimationFrame(g_lcd);
            if (frameTime_ms == 0)
            {
                uint16_t currRRInterval;
                if (!g_rrIntervals.read(currRRInterval))
                {
                    currRRInterval = lastValidRRInterval;
                }
                else
                {
                    lastValidRRInterval = currRRInterval;
                }
                frameTime_ms = g_heartAnimation.startAnimation(g_lcd, throbbingHeartXCenter,
                                                               topMargin + g_batteryBitmapHeight + bpmTopMargin + charHeight/2 - 2,
                                                                ColorMemLCD::COLOR_RED, currRRInterval);
            }
            ASSERT (frameTime_ms >= 0 && frameTime_ms < 1000 );
            if (frameTime_ms < 1)
            {
                frameTime_ms = 1;
            }
            ret_code_t errorCode = app_timer_start(g_animationTimer, APP_TIMER_TICKS(frameTime_ms, APP_TIMER_PRESCALER), NULL);
            APP_ERROR_CHECK(errorCode);
        }

    }
    else
    {
        // Draw a heart with BLE icon in the center if not connected.
        g_lcd.drawBitmap(ColorMemLCD::DISP_WIDTH/2 - g_heartBitmapWidth/2,
                        ColorMemLCD::DISP_HEIGHT/2 - g_heartBitmapHeight/2,
                        g_heartBitmap, g_heartBitmapWidth, g_heartBitmapHeight, ColorMemLCD::COLOR_RED);
        g_lcd.drawBitmap(ColorMemLCD::DISP_WIDTH/2 - g_bluetoothLargeBitmapWidth/2,
                        ColorMemLCD::DISP_HEIGHT/2 - g_bluetoothLargeBitmapHeight/2,
                        g_bluetoothLargeBitmap, g_bluetoothLargeBitmapWidth, g_bluetoothLargeBitmapHeight,
                        ColorMemLCD::COLOR_BLUE);
    }

    g_lcd.refresh();
}



// Break into debugger if any errors/asserts are detected at runtime.
// This handler is called when an application error is encountered in BLE stack or application code.
extern "C" void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    __debugbreak();
}

extern "C" void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    __debugbreak();
}


// Required to get C++ code to build.
extern "C" void __cxa_pure_virtual()
{
    ASSERT ( 0 );
    abort();
}
