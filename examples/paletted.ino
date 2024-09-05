#include <EEPROM.h>
#include <SPI.h>
#include <GD2.h>

uint32_t addr_pal = 0xffc00UL;

void setup()
{
  GD.begin(0);
  GD.BitmapLayout(PALETTED8, GD.w, GD.h);
  GD.BitmapSize(NEAREST, BORDER, BORDER, GD.w, GD.h);
  GD.BitmapSource(0);
  GD.Begin(BITMAPS);
  GD.BlendFunc(ONE, ZERO);

  GD.ColorMask(1, 0, 0, 0);
  GD.PaletteSource(addr_pal + 2);
  GD.Vertex2f(0, 0);

  GD.ColorMask(0, 1, 0, 0);
  GD.PaletteSource(addr_pal + 1);
  GD.Vertex2f(0, 0);

  GD.ColorMask(0, 0, 1, 0);
  GD.PaletteSource(addr_pal + 0);
  GD.Vertex2f(0, 0);
  GD.swap();

  // randomize the palette
  for (int i = 0; i < 1024; i++)
    GD.wr(addr_pal + i, GD.random());
}

void loop()
{
  // fill the whole screen with random bytes
  for (int y = 0; y < GD.h; y++)
    for (int x = 0; x < GD.w; x++)
      GD.wr(800 * y + x, GD.random());
}
