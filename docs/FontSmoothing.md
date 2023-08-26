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

My final smoothing version of the glyph drawing code can be found in the [Adafruit_GFX::drawSmoothChar()](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/firmware/Adafruit-GFX-Library/Adafruit_GFX.cpp#L2156) method.

## Final Results
![Font smoothing with hinting](https://github.com/adamgreen/nrf52-SmartWatch/blob/main/photos/font-smoothing_hint-smoothing.jpg?raw=true)