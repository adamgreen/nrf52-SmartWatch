// Driver for the LPM013M126C Transflective LCD on the Bangle.js2 SmartWatch.
// Learned a lot from:
//      https://github.com/BigCorvus/ColorMemLCD code by Arthur Jordan (https://github.com/BigCorvus)
//      https://github.com/espruino/Espruino
#include <stdarg.h>
#include <string.h>
#include <nrf_assert.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include "ColorMemLCD.h"


// Command bytes sent at the beginning of each line of output to the LCD.
// All of these commands will have M1 set to 0 as EXTCOMIN is used for toggling COM.
// The bits are reversed from what data sheets shows since we send LSB first over SPI to make code for drawPixel()
// simpler.
// 3-bit Data-Update Mode
static const uint8_t COLOR_CMD_UPDATE = 0b00000001;
// All Clear with No Blink
static const uint8_t COLOR_CMD_ALL_CLEAR = 0b00000100;
// No Update with No Blink
static const uint8_t COLOR_CMD_NO_UPDATE = 0b00000000;
// Blinking to White
static const uint8_t COLOR_CMD_BLINKING_WHITE = 0b00011000;
// Blinking to Black
static const uint8_t COLOR_CMD_BLINKING_BLACK = 0b00001000;
// Blinking to Inverted Colours
static const uint8_t COLOR_CMD_INVERSION = 0b00101000;

// The width of a single line of the display in memory when we use 4-bits per pixel.
static const size_t LINE_WIDTH  = ColorMemLCD::DISP_WIDTH / 2;

// Frequencies used for the EXTCOMIN signal. Use a lower rate when backlight isn't on.
static const uint32_t EXTCOMIN_BACKLIGHT_ON_PWM_FREQUENCY = 120;
static const uint32_t EXTCOMIN_BACKLIGHT_OFF_PWM_FREQUENCY = 12;

// Single object instance.
static ColorMemLCD* g_pOLED;


ColorMemLCD::ColorMemLCD(const nrf_drv_spi_t* pSpiInstance, nrf_drv_pwm_t* pPWM,
                         uint8_t sclkPin, uint8_t siPin, uint8_t scsPin,
                         uint8_t extcominPin, uint8_t dispPin, uint8_t backlightPin,
                         app_irq_priority_t irqPriority /* = APP_IRQ_PRIORITY_LOWEST */)
: Adafruit_GFX(DISP_WIDTH, DISP_HEIGHT)
{
    m_pSpi = pSpiInstance;
    m_pPWM = pPWM;
    m_sclkPin = sclkPin;
    m_siPin = siPin;
    m_scsPin = scsPin;
    m_extcominPin= extcominPin;
    m_dispPin = dispPin;
    m_backlightPin = backlightPin;
    m_irqPriority = irqPriority;

    // Pre-fill in the buffer with the M and AG bits already set at the beginning of each row.
    memset(m_buffer, 0, sizeof(m_buffer));
    uint8_t* pRow = m_buffer;
    for (int row = 0 ; row < DISP_HEIGHT ; row++)
    {
        pRow[0] = COLOR_CMD_UPDATE;
        // SPI is configured to send bytes LSB first, to make drawPixel() code simpler.
        // This requires the bit reversal and shift here.
        pRow[1] = __RBIT(row + 1) >> 24;
        pRow += ROW_SPAN;
    }

    // Default text to the 3-bit color for white.
    setTextColor(COLOR_WHITE);
}

void ColorMemLCD::init()
{
    // Can only have one instance of this class instantiated.
    ASSERT ( g_pOLED == NULL );
    g_pOLED = this;

    // Set pin state before direction to make sure they start this way (no glitching)
    // SCS pin should init to 0 to indicate a command isn't being sent yet.
    nrf_gpio_pin_clear(m_scsPin);
    nrf_gpio_pin_clear(m_dispPin);
    nrf_gpio_pin_clear(m_backlightPin);
    nrf_gpio_pin_clear(m_extcominPin);

    nrf_gpio_cfg_output(m_scsPin);
    nrf_gpio_cfg_output(m_dispPin);
    nrf_gpio_cfg_output(m_backlightPin);
    nrf_gpio_cfg_output(m_extcominPin);

    // I am running it at 2MHz as indicated in the datasheet as the fastest rate.
    // I see that the Espuino port over clocks it to 4MHz.
    nrf_drv_spi_config_t spiConfig =
    {
        .sck_pin      = m_sclkPin,
        .mosi_pin     = m_siPin,
        .miso_pin     = NRF_DRV_SPI_PIN_NOT_USED,
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
        .irq_priority = m_irqPriority,
        .orc          = 0xFF,
        .frequency    = NRF_DRV_SPI_FREQ_2M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_LSB_FIRST,
    };
    int errorCode = nrf_drv_spi_init(m_pSpi, &spiConfig, staticHandleSpiEvent);
    APP_ERROR_CHECK(errorCode);

    clearDisplay();
    turnOn();
}

void ColorMemLCD::staticHandleSpiEvent(nrf_drv_spi_evt_t const * pEvent)
{
    ASSERT ( g_pOLED != NULL );

    g_pOLED->handleSpiEvent(pEvent);
}

void ColorMemLCD::handleSpiEvent(nrf_drv_spi_evt_t const * pEvent)
{
    // Can pull the SCS (chip select) low now that the SPI transaction is complete.
    nrf_gpio_pin_clear(m_scsPin);

    m_inProgress = false;
}

void ColorMemLCD::turnOn()
{
    nrf_gpio_pin_set(m_dispPin);

    // 30µs delay between DISP going active and first EXTCOMIN pulse.
    nrf_delay_us(30);

    updateExtcominPwmFrequency();

    // 30µs delay between EXTCOMIN pulse and start of data.
    nrf_delay_us(30);
}

void ColorMemLCD::turnOff()
{
    nrf_gpio_pin_clear(m_dispPin);
    turnBacklightOff();
}


bool ColorMemLCD::isOn()
{
    return nrf_gpio_pin_out_read(m_dispPin);
}


void ColorMemLCD::turnBacklightOn()
{
    nrf_gpio_pin_set(m_backlightPin);
    updateExtcominPwmFrequency();
}


void ColorMemLCD::turnBacklightOff()
{
    nrf_gpio_pin_clear(m_backlightPin);
    updateExtcominPwmFrequency();
}


bool ColorMemLCD::isBacklightOn()
{
    return nrf_gpio_pin_out_read(m_backlightPin);
}


void ColorMemLCD::updateExtcominPwmFrequency()
{
    disableExtcominPwm();

    if (!isOn())
    {
        // Just leave EXTCOMIN signal off if the display is turned off.
        return;
    }

    // Initialize the PWM used for pulsing EXTCOMIN for 4us at the specified rate of 120Hz when backlight is on
    // and 12Hz when off.
    const uint32_t BASE_FREQ = 250000;
    uint32_t frequency = EXTCOMIN_BACKLIGHT_OFF_PWM_FREQUENCY;
    if (isBacklightOn())
    {
        frequency = EXTCOMIN_BACKLIGHT_ON_PWM_FREQUENCY;
    }
    uint16_t counterTop = BASE_FREQ / frequency;
    nrf_drv_pwm_config_t pwmConfig =
    {
        .output_pins = {m_extcominPin, NRF_DRV_PWM_PIN_NOT_USED, NRF_DRV_PWM_PIN_NOT_USED, NRF_DRV_PWM_PIN_NOT_USED},
        .irq_priority = m_irqPriority,
        .base_clock = NRF_PWM_CLK_250kHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value = counterTop,
        .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode = NRF_PWM_STEP_AUTO
    };
    ret_code_t errorCode = nrf_drv_pwm_init(m_pPWM, &pwmConfig, staticHandlePwmEvent);
    APP_ERROR_CHECK(errorCode);
    m_isPwmRunning = true;

    // Setup the PWM 2µs pulse for the EXTCOMIN signal.
    memset(&m_dutyCycles, 0, sizeof(m_dutyCycles));
    m_dutyCycles.channel_0 = counterTop - 1;
    nrf_pwm_sequence_t pwmSequence =
    {
        .values = { .p_individual = &m_dutyCycles },
        .length = NRF_PWM_VALUES_LENGTH(m_dutyCycles),
        .repeats = 0,
        .end_delay = 0
    };
    uint32_t playbackFlags = NRF_DRV_PWM_FLAG_LOOP;
    nrf_drv_pwm_simple_playback(m_pPWM, &pwmSequence, 1, playbackFlags);
}

void ColorMemLCD::disableExtcominPwm()
{
    if (!m_isPwmRunning)
    {
        return;
    }

    nrf_drv_pwm_uninit(m_pPWM);
    m_isPwmRunning = false;
}

void ColorMemLCD::staticHandlePwmEvent(nrf_drv_pwm_evt_type_t eventType)
{
    ASSERT ( g_pOLED != NULL );

    g_pOLED->handlePwmEvent(eventType);
}

void ColorMemLCD::handlePwmEvent(nrf_drv_pwm_evt_type_t eventType)
{
    switch (eventType)
    {
        case NRF_DRV_PWM_EVT_FINISHED:
            break;
        case NRF_DRV_PWM_EVT_END_SEQ0:
            ASSERT ( false );
            break;
        case NRF_DRV_PWM_EVT_END_SEQ1:
            ASSERT ( false );
            break;
        case NRF_DRV_PWM_EVT_STOPPED:
            break;
    }
}

void ColorMemLCD::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    ASSERT ( x >= 0 && x < DISP_WIDTH );
    ASSERT ( y >= 0 && y < DISP_HEIGHT );

    // The color must be 3-bit.
    ASSERT ( (color & 0xFFF8) == 0 );

    // The bit offset within the row is x * 3-bpp.
    size_t bitOffset = x * BITS_PER_PIXEL;
    // The byte offset within the row is the bitOffset divided by number of bits in a byte.
    size_t byteOffset = bitOffset / BITS_PER_BYTE;
    // The starting offset of this pixel within the byte is the remainder after the previous division.
    bitOffset = bitOffset % 8;
    // Calculate pointer to this pixel within the row. Cast to 16-bit pointer so that we can deal with pixels
    // that span 2 bytes.
    size_t pixelOffset = ROW_SPAN * y + byteOffset;
    uint16_t* pPixel = (uint16_t*)&m_pFrameBuffer[pixelOffset];

    // Update the pixel.
    uint16_t pixel = *pPixel;
    pixel &= ~(7 << bitOffset);
    pixel |= color << bitOffset;
    *pPixel = pixel;

    if (y < m_minDirtyRow)
    {
        m_minDirtyRow = y;
    }
    if (y > m_maxDirtyRow)
    {
        m_maxDirtyRow = y;
    }
}


// Transfer the full frame of pixels from the in memory display buffer to the LCD itself.
void ColorMemLCD::refresh(bool block /* = true */)
{
    if (m_minDirtyRow > m_maxDirtyRow)
    {
        // Nothing has changed since last refresh so do nothing.
        return;
    }

    // Send the dirty lines to the LCD via SPI.
    // The M and AG header bits were pre-populated in the constructor.
    uint8_t* pStart = &m_buffer[m_minDirtyRow * ROW_SPAN];
    size_t length = (m_maxDirtyRow - m_minDirtyRow + 1) * ROW_SPAN + 2;
    sendCommandBuffer(pStart, length, block);

    m_minDirtyRow = DISP_HEIGHT-1;
    m_maxDirtyRow = 0;
}


// Send current command buffer to LCD over SPI and wait for it to complete.
void ColorMemLCD::sendCommandBuffer(const uint8_t* pBuffer, size_t bufferLength, bool block)
{
    // Pull the SCS (chip select) pin high. It will be pulled low in handleSpiEvent() once transaction has completed.
    nrf_gpio_pin_set(m_scsPin);
    nrf_delay_us(6);

    // Make sure that previous transfer has completed before starting this one.
    waitForRefreshToComplete();

    m_inProgress = true;
    uint32_t errorCode = nrf_drv_spi_transfer(m_pSpi, pBuffer, bufferLength, NULL, 0);
    APP_ERROR_CHECK(errorCode);

    if (block)
    {
        waitForRefreshToComplete();
    }
}


// Fill the display memory with the specified colour.
void ColorMemLCD::cls(uint16_t color)
{
    // The color must be 3-bit.
    ASSERT ( (color & 0xFFF8) == 0 );

    // Build up 3 bytes, 6 pixels, of color data to fill into a row buffer to be used later for filling in each of the
    // rows of the frame buffer.
    uint32_t fill6Pixels = color | (color << 3) | (color << 6) | (color << 9) |
                           (color << 12) | (color << 15) | (color << 18) | (color << 21);

    // Fill in a row length buffer (rowFill) 3 bytes (6 pixels) at a time.
    uint8_t rowFill[BYTES_PER_ROW];
    for (size_t i = 0 ; i < BYTES_PER_ROW ; i += 3)
    {
        memcpy(&rowFill[i], &fill6Pixels, 3);
    }

    // Now fill in each row of the frame buffer from rowFill.
    uint8_t* pRow = m_pFrameBuffer;
    for (int row = 0 ; row < DISP_HEIGHT ; row++)
    {
        memcpy(pRow, rowFill, sizeof(rowFill));
        pRow += ROW_SPAN;
    }

    // Mark all rows as dirty.
    m_minDirtyRow = 0;
    m_maxDirtyRow = DISP_HEIGHT - 1;
}


// Clears the in memory display buffer to black and then sends a single command to the LCD which commands it to do the
// same.
void ColorMemLCD::clearDisplay()
{
    cls(0);

    const uint8_t commandBuffer[] = { COLOR_CMD_ALL_CLEAR, 0x00 };
    sendCommandBuffer(commandBuffer, sizeof(commandBuffer), true);

    // Reset the dirty rows just like refresh() does.
    m_minDirtyRow = DISP_HEIGHT-1;
    m_maxDirtyRow = 0;
}


void ColorMemLCD::setBlinkMode(BlinkModes mode)
{
    uint8_t blink_cmd = 0x00;

    switch( mode ) {
        case BLINKMODE_NONE:
            /* Blinking None    */
            blink_cmd = COLOR_CMD_NO_UPDATE;
            break;
        case BLINKMODE_WHITE:
            /* Blinking White   */
            blink_cmd = COLOR_CMD_BLINKING_WHITE;
            break;
        case BLINKMODE_BLACK:
            /* Blinking Black   */
            blink_cmd = COLOR_CMD_BLINKING_BLACK;
            break;
        case BLINKMODE_INVERSE:
            /* Inversion Mode   */
            blink_cmd = COLOR_CMD_INVERSION;
            break;
        default:
            /* No Update */
            blink_cmd = COLOR_CMD_NO_UPDATE;
            break;
    }

    uint8_t cmdBuffer[] = { blink_cmd, 0x00 };
    sendCommandBuffer(cmdBuffer, sizeof(cmdBuffer), true);
}

int ColorMemLCD::printf(const char* pFormat, ...)
{
    char buffer[256];
    va_list vaList;

    va_start(vaList, pFormat);
    int result = vsnprintf(buffer, sizeof(buffer), pFormat, vaList);
    va_end(vaList);
    print(buffer);

    return result;
}
