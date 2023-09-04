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
// Class which displays an animation of a beating heart symbol.
#include <app_timer.h>
#include <nrf_assert.h>
#include "HeartAnimation.h"
#include "Bitmaps/HeartBitmap.h"

HeartAnimation::HeartAnimation(float minScale, float maxScale, float deltaScale)
{
    m_minScale = minScale;
    m_maxScale = maxScale;
    m_deltaScale = deltaScale;
}

bool HeartAnimation::init()
{
    int32_t frameCount = (uint32_t)((m_maxScale - m_minScale) / m_deltaScale) + 1;

    m_pFrames = (Frame*)malloc(frameCount * sizeof(*m_pFrames));
    if (m_pFrames == NULL)
    {
        deinit();
        return false;
    }

    float scale = m_maxScale;
    m_frameCount = 0;
    for (int32_t i = 0 ; i < frameCount ; i++)
    {
        m_pFrames[i] = downsampleMonoBitmap(g_heartBitmap, g_heartBitmapWidth, g_heartBitmapHeight, scale);
        if (m_pFrames[i].pBitmap == NULL)
        {
            deinit();
            return false;
        }
        m_frameCount++;
        scale -= m_deltaScale;
    }
    ASSERT ( scale < m_minScale );
    ASSERT ( m_frameCount == frameCount );

    m_ticksPerMillisecond = (float)APP_TIMER_CLOCK_FREQ / (NRF_RTC1->PRESCALER + 1.0f) / 1000.0f;
    m_index = 0;
    m_indexIncrement = 0;
    m_frameTicks = 0;
    m_prevUpdateTicks = 0;
    return true;
}

void HeartAnimation::deinit()
{
    if (m_pFrames != NULL)
    {
        for (int32_t i = 0 ; i < m_frameCount ; i++)
        {
            free(m_pFrames[i].pBitmap);
        }
        free(m_pFrames);
    }
    m_frameCount = 0;
    m_pFrames = NULL;
    m_index = 0;
    m_indexIncrement = 0;
    m_centerX = 0;
    m_centerY = 0;
    m_frameTicks = 0;
    m_prevUpdateTicks = 0;
}

HeartAnimation::Frame HeartAnimation::downsampleMonoBitmap(const uint8_t* pSrc, uint32_t srcWidth, uint32_t srcHeight, float ratio)
{
    float srcSize = 1.0f / ratio;
    uint32_t srcRowPitch = (srcWidth + 7) / 8;
    uint32_t width = ratio * srcWidth;
    uint32_t destRowPitch = (width + 7) / 8;
    uint32_t height = ratio * srcHeight;
    Frame frame = { .pBitmap = NULL, .width = 0, .height = 0 };

    frame.pBitmap = (uint8_t*)malloc(destRowPitch * height);
    if (frame.pBitmap == NULL)
    {
        return frame;
    }

    uint8_t* pCurr = frame.pBitmap;
    for (uint32_t row = 0 ; row < height ; row++)
    {
        uint8_t* pStart = pCurr;
        uint8_t pixels = 0;
        for (uint32_t column = 0 ; column < width ; column++)
        {
            float sum = 0.0f;
            float weightSum = 0.0f;
            bool isFirstRow = true;
            for (float row2 = row * srcSize ; row2 <= (row + 1) * srcSize ; row2 += 1.0f)
            {
                // Calculate weight for this row.
                uint32_t r = row2;
                bool isFirstColumn = true;
                float rowWeight = 1.0f;
                if (isFirstRow)
                {
                    rowWeight = (r + 1.0f) - row2;
                }
                else if (row2 + 1.0f > (row + 1) * srcSize)
                {
                    rowWeight = row2 - r;
                }

                for (float column2 = column * srcSize ; column2 <= (column + 1) * srcSize ; column2 += 1.0f)
                {
                    uint32_t c = column2;

                    float pixel = 0.0f;
                    if (pSrc[r * srcRowPitch + c / 8] & (0x80 >> (c % 8)))
                    {
                        pixel = 1.0f;
                    }

                    // Calculate weight for this column.
                    float columnWeight = 1.0f;
                    if (isFirstColumn)
                    {
                        columnWeight *= (c + 1.0f) - column2;
                    }
                    else if (column2 + 1.0f > (column + 1) * srcSize)
                    {
                        columnWeight *= column2 - c;
                    }

                    float weight = rowWeight * columnWeight;
                    sum += pixel * weight;
                    weightSum += weight;
                    isFirstColumn = false;
                }
                isFirstRow = false;
            }

            if (column % 8 == 0)
            {
                pixels = 0;
            }

            ASSERT ( sum / weightSum <= 1.0f );
            uint8_t pixel = (sum / weightSum) + 0.5f;
            if (pixel)
            {
                pixels |= 0x01;
            }

            if (column == width - 1)
            {
                *pCurr++ = pixels << (7 - column % 8);
            }
            else if (column % 8 == 7)
            {
                *pCurr++ = pixels;
            }
            else
            {
                pixels <<= 1;
            }
        }
        pCurr = pStart + destRowPitch;
    }

    frame.width = width;
    frame.height = height;
    return frame;
}

int32_t HeartAnimation::startAnimation(Adafruit_GFX& lcd, int16_t centerX, int16_t centerY, int16_t color, uint32_t time_ms)
{
    m_centerX = centerX;
    m_centerY = centerY;
    m_color = color;
    m_index = 0;
    m_indexIncrement = 1;

    uint32_t frameTime_ms = time_ms / (2 * m_frameCount - 2);
    m_frameTicks = frameTime_ms * m_ticksPerMillisecond + 0.5f;
    m_prevUpdateTicks = app_timer_cnt_get() - m_frameTicks;
    return drawAnimationFrame(lcd);
}

int32_t HeartAnimation::drawAnimationFrame(Adafruit_GFX& lcd)
{
    if (m_indexIncrement == 0)
    {
        // Animation has completed.
        return 0;
    }

    // Advance to next frame before calling drawBitmap if the time has advanced enough.
    int32_t elapsedTicks = tickDelta(app_timer_cnt_get(), m_prevUpdateTicks);
    if (elapsedTicks >= m_frameTicks)
    {
        m_prevUpdateTicks += m_frameTicks;

        if (m_indexIncrement > 0 && m_index == m_frameCount - 1)
        {
            m_index = m_frameCount - 2;
            m_indexIncrement = -1;
        }
        else if (m_indexIncrement < 0 && m_index == 0)
        {
            // Animation is complete.
            m_indexIncrement = 0;
            return 0;
        }
        else
        {
            m_index += m_indexIncrement;
        }
    }

    Frame* pFrame = &m_pFrames[m_index];
    lcd.drawBitmap(m_centerX - pFrame->width/2, m_centerY - pFrame->height/2,
                   pFrame->pBitmap, pFrame->width, pFrame->height, m_color);

    elapsedTicks = tickDelta(app_timer_cnt_get(), m_prevUpdateTicks);
    return millisecondsLeft(elapsedTicks);
}
