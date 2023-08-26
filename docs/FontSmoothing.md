# Font Smoothing
## Starting Point
The following photo shows the point from which I was starting. It shows the default Adafruit font scaled up by 4x on my actual watch LCD. It contains a lot of jaggies. Especially in the diagonals.

![Unsmoothed Font](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/photos/font-smoothing_no-smoothing.jpg?raw=true)

## October 25th, 2020 HaD Article
Back in 2020 I saw a HaD article about smoothing the same font.
* [Link to HaD article](https://hackaday.com/2020/10/25/smoothing-big-fonts-on-graphic-lcds/)
* [Link to source material](http://www.technoblogy.com/show?3AJ7)

The code described in this article detects diagonals like that found in the '0' and turns on two more pixels along the diagonal to smooth it out a bit.

![Original image from article](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/photos/font-smoothing_orig.jpg?raw=true)

This solution works when the font is scaled up by 2x but it fails at larger scaling ratios as more pixels are required for their proper smoothing.

## My Triangle Generalization
I generalized the code further by inserting triangles along the diagonals. In the 2x case the triangles end up turning on the same 2 pixels.

![Image of my triangle solution](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/photos/font-smoothing_my-solution.jpg?raw=true)

```c++
void Adafruit_GFX::drawSmoothChar(int16_t x, int16_t y, unsigned char c,
                                  uint16_t color, uint16_t bg, uint8_t size_x,
                                  uint8_t size_y)
{
    // Find beginning of character glyph in the font file.
    const uint8_t* pFont = &font[c * 5];

    // Read all 5 columns of the glyph into memory and place dummy bars on the far right and left so that the diagonal
    // detector doesn't go off on the edges erroneously. Also OR in an extra bit to each column to add a dummy row at
    // the bottom for the same reason.
    uint32_t glyph[5+2] = {
        0x1FFU,
        (uint32_t)pFont[0] | 0x100U,
        (uint32_t)pFont[1] | 0x100U,
        (uint32_t)pFont[2] | 0x100U,
        (uint32_t)pFont[3] | 0x100U,
        (uint32_t)pFont[4] | 0x100U,
        0x1FFU
    };

    // Can optimize the 2x scaling because one triangle is enough to fill in both pixels required for smoothing.
    bool scaleIs2x = (size_x == 2) && (size_y == 2);

    // Iterate over each column of the glyph.
    for (size_t i = 1 ; i <= 5 ; i++)
    {
        // Need access to the columns to the left and right of the current column to check for diagonals to be smoothed.
        uint32_t left = glyph[i-1];
        uint32_t current = glyph[i];
        uint32_t right = glyph[i+1];

        // Iterate over the rows of the glyph.
        for (size_t j = 0 ; j < 8 ; j++)
        {
            if (current & 1)
            {
                // This pixel of the glyph should be turned on.
                writeFillRect(x + i * size_x, y + j * size_y, size_x, size_y, color);

                // Check to see if this pixel is the upper part of a 1-pixel wide diagonal that should be smoothed.
                if ((current & 0b11) == 0b01)
                {
                    // The pixel just below this one is off so it might be a diagonal. Check the pixels to right/left.
                    if ((left & 0b11) == 0b10)
                    {
                        fillTriangle(      x + i * size_x, y + j * size_y,
                                           x + i * size_x, y + (j + 1) * size_y,
                                     x + (i - 1) * size_x, y + (j + 1) * size_y,
                                     color);
                        if (!scaleIs2x)
                        {
                            fillTriangle(      x + i * size_x - 1, y + (j + 2) * size_y - 1,
                                               x + i * size_x - 1, y + (j + 1) * size_y - 1,
                                         x + (i + 1) * size_x - 1, y + (j + 1) * size_y - 1,
                                         color);
                        }
                    }
                    if ((right & 0b11) == 0b10)
                    {
                        fillTriangle(x + (i + 1) * size_x - 1, y + j * size_y,
                                     x + (i + 1) * size_x - 1, y + (j + 1) * size_y,
                                     x + (i + 2) * size_x - 1, y + (j + 1) * size_y,
                                     color);
                        if (!scaleIs2x)
                        {
                            fillTriangle(x + (i + 1) * size_x, y + (j + 2) * size_y - 1,
                                         x + (i + 1) * size_x, y + (j + 1) * size_y - 1,
                                               x + i * size_x, y + (j + 1) * size_y - 1,
                                         color);
                        }
                    }
                }
            }

            // Advance to the next row of the glyph.
            left >>= 1;
            current >>= 1;
            right >>= 1;
        }
    }

    // Will look at this code a bit further down in these notes...
}
```
The following photo shows how the glyphs look once this smoothing has been applied.

![Photo of my triangle solution on LCD](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/photos/font-smoothing_init_smoothing.jpg?raw=true)

## More Triangles
The glyphs look much better now but they aren't perfect:
* The slash in the middle of the **'0'** and **'N'** are a bit jagged where they meet up with the verticals.
* The **'4'** looks like something took a bite out of it towards the top.
* ...

If we look at the **'4'** glyph more closely we can see that the bite mark is actually a triangle.

![4 with a bite out of it](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/photos/font-smoothing_four-init-smoothing.jpg?raw=true)

So my solution is to add **More Triangles**.

![4 with hinting](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/photos/font-smoothing_four-hint-smoothing.jpg?raw=true)

I have a table with an entry for each problem glyph. The table entries describe the triangles to be added to smooth out a particular glyph.

```c++
// The triangle hint arrays for each glyph that I want to add a bit of extra smoothing to.
// Just showing 2 of the hinting entries here as examples.
// See actual source code for the complete list.
//      '4'
static Triangle g_4_Hints[] =
{
    { Point{3, 0, TOP_LEFT}, Point{3, 1, TOP_LEFT}, Point{2, 1, TOP_LEFT} },
};
//      'M'
static Triangle g_M_Hints[] =
{
    { Point{0, 0, TOP_RIGHT}, Point{0, 1, TOP_RIGHT}, Point{1, 1, TOP_RIGHT} },
    { Point{4, 0, TOP_LEFT},  Point{4, 1, TOP_LEFT},  Point{3, 1, TOP_LEFT} },
};


// The sorted list of glyph hints that can be used from drawSmoothChar() to apply extra smoothing to the few
// glyphs which require it.
static Hint g_hints[] =
{
    // These must be in alphabetical order so that they can be found with bsearch().
    { g_0_Hints, count_of(g_0_Hints), '0' },
    { g_1_Hints, count_of(g_1_Hints), '1' },
    { g_3_Hints, count_of(g_3_Hints), '3' },
    { g_4_Hints, count_of(g_4_Hints), '4' },
    { g_M_Hints, count_of(g_M_Hints), 'M' },
    { g_N_Hints, count_of(g_N_Hints), 'N' },
    { g_R_Hints, count_of(g_R_Hints), 'R' },
    { g_Z_Hints, count_of(g_Z_Hints), 'Z' },
    { g_b_Hints, count_of(g_b_Hints), 'b' },
    { g_d_Hints, count_of(g_d_Hints), 'd' },
    { g_g_Hints, count_of(g_g_Hints), 'g' },
    { g_hnr_Hints, count_of(g_hnr_Hints), 'h' },
    { g_hnr_Hints, count_of(g_hnr_Hints), 'n' },
    { g_hnr_Hints, count_of(g_hnr_Hints), 'r' },
    { g_u_Hints, count_of(g_u_Hints), 'u' },
    { g_z_Hints, count_of(g_z_Hints), 'z' },
};


void Adafruit_GFX::drawSmoothChar(int16_t x, int16_t y, unsigned char c,
                                  uint16_t color, uint16_t bg, uint8_t size_x,
                                  uint8_t size_y)
{
    // Code previously looked at above in these notes...

    // Check for hints to apply additional smoothing to this glyph.
    Hint* pHint= (Hint*)bsearch((void*)(int)(int8_t)c, g_hints, count_of(g_hints), sizeof(g_hints[0]), compareHints);
    if (pHint == NULL)
    {
        return;
    }
    // Add in the triangles from the hint, scaled up to the correct size.
    for (size_t i = 0 ; i < pHint->triangleCount ; i++)
    {
        Triangle* pTriangle = &pHint->pTriangles[i];
        fillTriangle(pTriangle->m_1.calcX(x, size_x), pTriangle->m_1.calcY(y, size_y),
                     pTriangle->m_2.calcX(x, size_x), pTriangle->m_2.calcY(y, size_y),
                     pTriangle->m_3.calcX(x, size_x), pTriangle->m_3.calcY(y, size_y),
                     color);
    }
}
```

My final smoothing version of the glyph drawing code can be found in the [Adafruit_GFX::drawSmoothChar()](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/firmware/Adafruit-GFX-Library/Adafruit_GFX.cpp#L2156) method.

## Final Results
![Font smoothing with hinting](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/photos/font-smoothing_hint-smoothing.jpg?raw=true)