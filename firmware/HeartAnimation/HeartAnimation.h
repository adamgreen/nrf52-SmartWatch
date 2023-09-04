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
#ifndef HEART_ANIMATION_H_
#define HEART_ANIMATION_H_
#include <stdint.h>
#include <Adafruit_GFX.h>

class HeartAnimation
{
    public:
        HeartAnimation(float minScale, float maxScale, float deltaScale);
        ~HeartAnimation()
        {
            deinit();
        }

        bool init();
        void deinit();

        int32_t startAnimation(Adafruit_GFX& lcd, int16_t centerX, int16_t centerY, int16_t color, uint32_t time_ms);
        int32_t drawAnimationFrame(Adafruit_GFX& lcd);

    protected:
        struct Frame
        {
            uint8_t* pBitmap;
            uint32_t width;
            uint32_t height;
        };

        static Frame downsampleMonoBitmap(const uint8_t* pSrc, uint32_t srcWidth, uint32_t srcHeight, float ratio);
        int32_t tickDelta(uint32_t newer, uint32_t older)
        {
            return (int32_t)(((newer - older) & 0xFFFFFF) << 8) >> 8;
        }
        int32_t millisecondsLeft(int32_t elapsedTicks)
        {
            ASSERT ( elapsedTicks >= 0 );
            if (elapsedTicks > m_frameTicks)
            {
                return 0;
            }
            int32_t ticksLeft = m_frameTicks - elapsedTicks;
            return ticksLeft / m_ticksPerMillisecond + 0.5f;
        }

        Frame*    m_pFrames = NULL;
        float     m_minScale;
        float     m_maxScale;
        float     m_deltaScale;
        float     m_ticksPerMillisecond = 0.0f;
        uint32_t  m_prevUpdateTicks = 0;
        int32_t   m_frameTicks = 0;
        int32_t   m_frameCount = 0;
        int32_t   m_index = 0;
        int32_t   m_indexIncrement = 1;
        int16_t   m_centerX = 0;
        int16_t   m_centerY = 0;
        int16_t   m_color = 0;
};

#endif // HEART_ANIMATION_H_
