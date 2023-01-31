//****************************************************************************
// OLED Driver - SSD1306 / SSD1309
//****************************************************************************

//****************************************************************************
#ifdef USE_OLED
//****************************************************************************

#include <Wire.h>  // Arduino standard I2C/Two-Wire Library

#include "oled.h"

//****************************************************************************

/**
 * This Library is written and optimized by Olivier Van den Eede(4ilo) in 2016
 * for Stm32 Uc and HAL-i2c lib's.
 *
 * To use this library with ssd1306 oled display you will need to customize the defines below.
 *
 * This library uses 2 extra files (fonts.c/h).
 * In this files are 3 different fonts you can use:
 *    - Font_7x10
 *    - Font_11x18
 *    - Font_16x26
 *
 */

#ifdef FONT
#include "fonts.h"
#endif

//****************************************************************************

// Screen object
static SSD1306_t SSD1306;

// Screenbuffer
uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

//****************************************************************************
//
//  Send a byte to the command register
//
static void ssd1306_WriteCommand(uint8_t command)
{
  if (!SSD1306.Initialized) return;

  // Begin transmission to the I2C slave device
  Wire.beginTransmission(SSD1306_I2C_ADDR);

  Wire.write((uint8_t)0); // Put a zero in the xmit buffer.

  // Queue command data array for transmission to the I2C device
  if (Wire.write(&command, (size_t)1) != 1)
  {
    Wire.write(0);              // Put a zero in the xmit buffer.
    Wire.endTransmission(true); // Send and Close the I2C interface.
    return;
  }

  // Transmit the bytes and a stop message to release the I2C bus.
  if (Wire.endTransmission(true))
  {
     if (Serial) Serial.println("WriteCommand Failed");
     SSD1306.ErrorTotal++;
     SSD1306.Error++;
     if (SSD1306.Error > 5)
     {
       SSD1306.Initialized = 0;
       while(1);
     }
  }
  else
    SSD1306.Error = 0;
}

//****************************************************************************

void ssd1306_Read(int Size, uint8_t *Data)
{
  uint8_t cmd[] = { 0x80,0x00,0xC0 };
  Wire.beginTransmission(SSD1306_I2C_ADDR);
  Wire.write(cmd, sizeof(cmd));
  Wire.endTransmission(false);
  Wire.requestFrom(SSD1306_I2C_ADDR, Size); // Request two byte
  while(Wire.available() < Size)
    delay(1); // Wait for data to be available
  Wire.readBytes(Data, Size);
}

//****************************************************************************

void ssd1306_Detect(void)
{
  uint8_t cmd[] = { 0x80,0x00,0xC0,0x55 }; // write pattern
  uint8_t data[3];

  ssd1306_Read(sizeof(data), data);

  if (Serial)
  {
    char string[32];
    sprintf(string, "Data %02X %02X", data[0], data[1]);
    Serial.println(string);
  }

  cmd[3] = data[1] ^ 0xC3;

  if (Serial)
  {
    char string[32];
    sprintf(string, "Cmd  %02X %02X %02X %02X", cmd[0], cmd[1], cmd[2], cmd[3]);
    Serial.println(string);
  }

  Wire.beginTransmission(SSD1306_I2C_ADDR);
  Wire.write(cmd, sizeof(cmd));
  Wire.endTransmission(true);

  ssd1306_Read(sizeof(data), data);

  if (Serial)
  {
    char string[32];
    sprintf(string, "Data %02X %02X %02X", data[0], data[1], data[2]);
    Serial.println(string);
  }

  if (data[1] == cmd[3])
  {
    if (Serial) Serial.println("SH1106 Detected");
    SSD1306.Initialized |= 2;
  }
}

//****************************************************************************
//
//  Initialize the oled screen
//
//****************************************************************************

// https://github.com/4ilo/ssd1306-stm32HAL/blob/master/Src/ssd1306.c

uint8_t ssd1306_Init(void)
{
  SSD1306.Initialized = 1; // This might get cleared if display fail

  delay(250); // Wait for the screen to boot

  ssd1306_Detect(); // SSD130x vs SH1106 check

  /* Init LCD */
  ssd1306_WriteCommand(0xAE); //display off

  if (!SSD1306.Initialized) return(0); // Exit quickly

  ssd1306_WriteCommand(0x20); //Set Memory Addressing Mode
  ssd1306_WriteCommand(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  ssd1306_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

  ssd1306_WriteCommand(0xC8); //Set COM Output Scan Direction
//  ssd1306_WriteCommand(SSD1306_WIDTH & 0x0F); //---set low column address
  ssd1306_WriteCommand(0x00); //---set low column address
  ssd1306_WriteCommand(0x10); //---set high column address

  ssd1306_WriteCommand(0x40); //--set start line address
  ssd1306_WriteCommand(0x81); //--set contrast control register
  ssd1306_WriteCommand(0xFF);
  ssd1306_WriteCommand(0xA1); //--set segment re-map 0 to 127
  ssd1306_WriteCommand(0xA6); //--set normal display

  ssd1306_WriteCommand(0xA8); //--set multiplex ratio(1 to 64)
#if 1
  ssd1306_WriteCommand(SSD1306_HEIGHT - 1); // 0x3F
#else
  ssd1306_WriteCommand(0x3F); //
#endif

  ssd1306_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  ssd1306_WriteCommand(0xD3); //-set display offset
  ssd1306_WriteCommand(0x00); //-not offset
  ssd1306_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
  ssd1306_WriteCommand(0xF0); //--set divide ratio
  ssd1306_WriteCommand(0xD9); //--set pre-charge period
  ssd1306_WriteCommand(0x22); //
  ssd1306_WriteCommand(0xDA); //--set com pins hardware configuration

#if 1
  if (SSD1306_HEIGHT == 64)
    ssd1306_WriteCommand(0x12);
  else
    ssd1306_WriteCommand(0x02);
#else
  ssd1306_WriteCommand(0x12);
#endif

  ssd1306_WriteCommand(0xDB); //--set vcomh
  ssd1306_WriteCommand(0x20); //0x20,0.77xVcc
  ssd1306_WriteCommand(0x8D); //--set DC-DC enable
  ssd1306_WriteCommand(0x14); //
  ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel

  // Clear screen
  ssd1306_Fill(Black);

  // Flush buffer to screen
  ssd1306_UpdateScreen();

  // Set default values for screen object
  SSD1306.CurrentX = 0;
  SSD1306.CurrentY = 0;

  return(1);
}

//****************************************************************************
//
//  Fill the whole screen with the given color
//
void ssd1306_Fill(SSD1306_COLOR color)
{
  /* Set memory */
  uint32_t i;

  for(i = 0; i < sizeof(SSD1306_Buffer); i++)
  {
    SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
  }

  SSD1306.Touched = 1;
}

//****************************************************************************
//
//  Write the screenbuffer with changed to the screen
//
void ssd1306_UpdateScreen(void)
{
  int i;

  if (!SSD1306.Initialized)
    return;

  if (!SSD1306.Touched)
    return;

  SSD1306.Touched = 0;

  for (i=0; i<(SSD1306_HEIGHT / 8); i++)
  {
    ssd1306_WriteCommand(0xB0 + i); // B0..B7 Line Number

    if (SSD1306.Initialized & 2)
      ssd1306_WriteCommand(0x02); // 00..0F Index in 2 pixels (SH1106)
    else
      ssd1306_WriteCommand(0x00); // 00..0F (SSD130x)

    ssd1306_WriteCommand(0x10); // 10..1F

    // Begin transmission to the I2C slave device
    Wire.beginTransmission(SSD1306_I2C_ADDR);

    Wire.write((uint8_t)0x40);              // Put a zero in the xmit buffer.

    // Queue command data array for transmission to the I2C device
    Wire.write(&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);

    // Transmit the bytes and a stop message to release the I2C bus.
    if (Wire.endTransmission(true))
    {
      if (Serial) Serial.println("UpdateScreen Failed");
      SSD1306.ErrorTotal++;
      SSD1306.Error++;
      if (SSD1306.Error > 5)
        SSD1306.Initialized = 0;
    }
    else
      SSD1306.Error = 0;
  }
}

//****************************************************************************
//
//  Draw one pixel in the screenbuffer
//  X => X Coordinate
//  Y => Y Coordinate
//  color => Pixel color
//
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
  if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
  {
    // Don't write outside the buffer
    return;
  }

  // Check if pixel should be inverted
  if (SSD1306.Inverted)
  {
    color = (SSD1306_COLOR)!color;
  }

  // Draw in the right color
  if (color == White)
  {
    SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
  }
  else
  {
    SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
  }

  SSD1306.Touched = 1;
}

//****************************************************************************
#ifdef FONT
//****************************************************************************
//
//  Draw 1 char to the screen buffer
//  ch    => char om weg te schrijven
//  Font  => Font waarmee we gaan schrijven
//  color   => Black or White
//
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
  uint32_t i, b, j;

  // Check remaining space on current line
  if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
    SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
  {
    // Not enough space on current line
    return 0;
  }

  // Use the font to write
  for (i = 0; i < Font.FontHeight; i++)
  {
    b = Font.data[(ch - 32) * Font.FontHeight + i];
    for (j = 0; j < Font.FontWidth; j++)
    {
      if ((b << j) & 0x8000)
      {
        ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
      }
      else
      {
        ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
      }
    }
  }

  // The current space is now taken
  SSD1306.CurrentX += Font.FontWidth;

  // Return written char for validation
  return(ch);
}

//****************************************************************************
//
//  Write full string to screenbuffer
//
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{
  // Write until null-byte
  while (*str)
  {
    if (ssd1306_WriteChar(*str, Font, color) != *str)
    {
      // Char could not be written
      return *str;
    }

    // Next char
    str++;
  }

  // Everything ok
  return(*str);
}

//****************************************************************************
#endif // FONT
//****************************************************************************
//
//  Position the cursor
//

void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
  SSD1306.CurrentX = x;
  SSD1306.CurrentY = y;
}

//****************************************************************************

// https://elixir.bootlin.com/linux/latest/source/lib/fonts/font_acorn_8x8.c

const uint8_t FontAcorn8_Table [] =
{
/* 20 */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*   */
/* 21 */  0x18, 0x3c, 0x3c, 0x18, 0x18, 0x00, 0x18, 0x00, /* ! */
/* 22 */  0x6C, 0x6C, 0x6C, 0x00, 0x00, 0x00, 0x00, 0x00, /* " */
/* 23 */  0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00, /* # */
/* 24 */  0x0C, 0x3F, 0x68, 0x3E, 0x0B, 0x7E, 0x18, 0x00, /* $ */
/* 25 */  0x60, 0x66, 0x0C, 0x18, 0x30, 0x66, 0x06, 0x00, /* % */
/* 26 */  0x38, 0x6C, 0x6C, 0x38, 0x6D, 0x66, 0x3B, 0x00, /* & */
/* 27 */  0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, /* ' */
/* 28 */  0x0C, 0x18, 0x30, 0x30, 0x30, 0x18, 0x0C, 0x00, /* ( */
/* 29 */  0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x18, 0x30, 0x00, /* ) */
/* 2A */  0x00, 0x18, 0x7E, 0x3C, 0x7E, 0x18, 0x00, 0x00, /* * */
/* 2B */  0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x00, 0x00, /* + */
/* 2C */  0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30, /* , */
/* 2D */  0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, /* - */
/* 2E */  0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, /* . */
/* 2F */  0x00, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x00, 0x00, /* / */
/* 30 */  0x3C, 0x66, 0x6E, 0x7E, 0x76, 0x66, 0x3C, 0x00, /* 0 */
/* 31 */  0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00, /* 1 */
/* 32 */  0x3C, 0x66, 0x06, 0x0C, 0x18, 0x30, 0x7E, 0x00, /* 2 */
/* 33 */  0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00, /* 3 */
/* 34 */  0x0C, 0x1C, 0x3C, 0x6C, 0x7E, 0x0C, 0x0C, 0x00, /* 4 */
/* 35 */  0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0x00, /* 5 */
/* 36 */  0x1C, 0x30, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00, /* 6 */
/* 37 */  0x7E, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x00, /* 7 */
/* 38 */  0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00, /* 8 */
/* 39 */  0x3C, 0x66, 0x66, 0x3E, 0x06, 0x0C, 0x38, 0x00, /* 9 */
/* 3A */  0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, /* : */
/* 3B */  0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x30, /* ; */
/* 3C */  0x0C, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0C, 0x00, /* < */
/* 3D */  0x00, 0x00, 0x7E, 0x00, 0x7E, 0x00, 0x00, 0x00, /* = */
/* 3E */  0x30, 0x18, 0x0C, 0x06, 0x0C, 0x18, 0x30, 0x00, /* > */
/* 3F */  0x3C, 0x66, 0x0C, 0x18, 0x18, 0x00, 0x18, 0x00, /* ? */
/* 40 */  0x3C, 0x66, 0x6E, 0x6A, 0x6E, 0x60, 0x3C, 0x00, /* @ */
/* 41 */  0x3C, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00, /* A */
/* 42 */  0x7C, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x7C, 0x00, /* B */
/* 43 */  0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00, /* C */
/* 44 */  0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00, /* D */
/* 45 */  0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x7E, 0x00, /* E */
/* 46 */  0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x00, /* F */
/* 47 */  0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x3C, 0x00, /* G */
/* 48 */  0x66, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00, /* H */
/* 49 */  0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00, /* I */
/* 4A */  0x3E, 0x0C, 0x0C, 0x0C, 0x0C, 0x6C, 0x38, 0x00, /* J */
/* 4B */  0x66, 0x6C, 0x78, 0x70, 0x78, 0x6C, 0x66, 0x00, /* K */
/* 4C */  0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00, /* L */
/* 4D */  0x63, 0x77, 0x7F, 0x6B, 0x6B, 0x63, 0x63, 0x00, /* M */
/* 4E */  0x66, 0x66, 0x76, 0x7E, 0x6E, 0x66, 0x66, 0x00, /* N */
/* 4F */  0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00, /* O */
/* 50 */  0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00, /* P */
/* 51 */  0x3C, 0x66, 0x66, 0x66, 0x6A, 0x6C, 0x36, 0x00, /* Q */
/* 52 */  0x7C, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0x00, /* R */
/* 53 */  0x3C, 0x66, 0x60, 0x3C, 0x06, 0x66, 0x3C, 0x00, /* S */
/* 54 */  0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, /* T */
/* 55 */  0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00, /* U */
/* 56 */  0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00, /* V */
/* 57 */  0x63, 0x63, 0x6B, 0x6B, 0x7F, 0x77, 0x63, 0x00, /* W */
/* 58 */  0x66, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x66, 0x00, /* X */
/* 59 */  0x66, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x00, /* Y */
/* 5A */  0x7E, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E, 0x00, /* Z */
/* 5B */  0x7C, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7C, 0x00, /* [ */
/* 5C */  0x00, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x00, 0x00, /* \ */
/* 5D */  0x3E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x3E, 0x00, /* ] */
/* 5E */  0x3C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ^ */
/* 5F */  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, /* _ */
/* 60 */  0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* ` */
/* 61 */  0x00, 0x00, 0x3C, 0x06, 0x3E, 0x66, 0x3E, 0x00, /* a */
/* 62 */  0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x7C, 0x00, /* b */
/* 63 */  0x00, 0x00, 0x3C, 0x66, 0x60, 0x66, 0x3C, 0x00, /* c */
/* 64 */  0x06, 0x06, 0x3E, 0x66, 0x66, 0x66, 0x3E, 0x00, /* d */
/* 65 */  0x00, 0x00, 0x3C, 0x66, 0x7E, 0x60, 0x3C, 0x00, /* e */
/* 66 */  0x1C, 0x30, 0x30, 0x7C, 0x30, 0x30, 0x30, 0x00, /* f */
/* 67 */  0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x3C, /* g */
/* 68 */  0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00, /* h */
/* 69 */  0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x3C, 0x00, /* i */
/* 6A */  0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x70, /* j */
/* 6B */  0x60, 0x60, 0x66, 0x6C, 0x78, 0x6C, 0x66, 0x00, /* k */
/* 6C */  0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, /* l */
/* 6D */  0x00, 0x00, 0x36, 0x7F, 0x6B, 0x6B, 0x63, 0x00, /* m */
/* 6E */  0x00, 0x00, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00, /* n */
/* 6F */  0x00, 0x00, 0x3C, 0x66, 0x66, 0x66, 0x3C, 0x00, /* o */
/* 70 */  0x00, 0x00, 0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, /* p */
/* 71 */  0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x07, /* q */
/* 72 */  0x00, 0x00, 0x6C, 0x76, 0x60, 0x60, 0x60, 0x00, /* r */
/* 73 */  0x00, 0x00, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x00, /* s */
/* 74 */  0x30, 0x30, 0x7C, 0x30, 0x30, 0x30, 0x1C, 0x00, /* t */
/* 75 */  0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x3E, 0x00, /* u */
/* 76 */  0x00, 0x00, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00, /* v */
/* 77 */  0x00, 0x00, 0x63, 0x6B, 0x6B, 0x7F, 0x36, 0x00, /* w */
/* 78 */  0x00, 0x00, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x00, /* x */
/* 79 */  0x00, 0x00, 0x66, 0x66, 0x66, 0x3E, 0x06, 0x3C, /* y */
/* 7A */  0x00, 0x00, 0x7E, 0x0C, 0x18, 0x30, 0x7E, 0x00, /* z */
/* 7B */  0x0C, 0x18, 0x18, 0x70, 0x18, 0x18, 0x0C, 0x00, /* { */
/* 7C */  0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, /* | */
/* 7D */  0x30, 0x18, 0x18, 0x0E, 0x18, 0x18, 0x30, 0x00, /* } */
/* 7E */  0x31, 0x6B, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, /* ~ */
/* 7F */  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, /*  */
};

//****************************************************************************

void ssd1306_OutString(const char *str)
{
#ifndef USE_OLED_TEXT
  int i, j;
  int line = 0, column = 0;
  int linemax = (SSD1306_HEIGHT == 64) ? 8 : 4;

  uint8_t *p = SSD1306_Buffer;

  //memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer) * 23 / 64);

  while(*str)
  {
    uint8_t ch = *str++;
    const uint8_t *q = &FontAcorn8_Table[(int)(ch - 0x20) * 8];
    if (ch == '\n')
    {
      line++;
      column = 0;
      if (line >= linemax)
        break;
      p = &SSD1306_Buffer[line * SSD1306_WIDTH];
      continue;
    }
    for(i=0; i<8; i++)
    {
      for(j=0; j<8; j++)
      {
        uint32_t mask = 0x80 >> j;
        uint8_t x = q[7-i] & mask;
        if (x)
          p[j] |= (0x80 >> i);
        else
          p[j] &= ~(0x80 >> i);
      }
    }
    p += 8;
    column++;
    if (column >= 16)
    {
      column = 0;
      line++;
      if (line >= linemax)
        break;
    }
  }

  SSD1306.Touched = 1;
#endif
}

//****************************************************************************

#define SSD1306_WIDTH_VIRTUAL_TEXT 80
#define SSD1306_HEIGHT_VIRTUAL_TEXT 8

uint8_t SSD1306_TextBuffer[SSD1306_WIDTH_VIRTUAL_TEXT * SSD1306_HEIGHT_VIRTUAL_TEXT];
uint32_t SSD1306_TextX;
uint32_t SSD1306_TextY;
uint32_t SSD1306_Update;

//****************************************************************************

void ssd1306_InitText(void)
{
  memset(SSD1306_TextBuffer, ' ', sizeof(SSD1306_TextBuffer)); // Fill withspaces
  SSD1306_TextX = 0;
  SSD1306_TextY = 0;
  SSD1306_Update = 0;
}

//****************************************************************************

void ssd1306_PaintText(void) // Render Text Buffer to Pixel Buffer
{
  uint8_t *p = SSD1306_Buffer;

  int i, j;
  int line, width;
  int linemax = (SSD1306_HEIGHT == 64) ? 8 : 4;

  if (SSD1306_Update == 0) return; // Unchanged

  SSD1306_Update = 0;

  for(line=0; line<linemax; line++)
  {
    for(width=0; width<16; width++)
    {
      uint8_t ch = SSD1306_TextBuffer[line*SSD1306_WIDTH_VIRTUAL_TEXT + width];
      const uint8_t *q = &FontAcorn8_Table[(ch - 0x20) * 8];

      for(i=0; i<8; i++) // 8x8 pixels
      {
        for(j=0; j<8; j++)
        {
          uint32_t mask = 0x80 >> j;
          uint8_t x = q[7-i] & mask;
          if (x)
            p[j] |= (0x80 >> i);
          else
            p[j] &= ~(0x80 >> i);
        }
      }
      p += 8; // next char position
    }
  }

  SSD1306.Touched = 1;
}

//****************************************************************************

void ssd1306_OutTextChar(char c)
{
  if (SSD1306_TextX >= SSD1306_WIDTH_VIRTUAL_TEXT)
  {
    SSD1306_TextY++;
    SSD1306_TextX = 0;
  }

  if (SSD1306_TextY >= SSD1306_HEIGHT_VIRTUAL_TEXT)
  {
    SSD1306_TextY = SSD1306_HEIGHT_VIRTUAL_TEXT-1;
    memcpy(SSD1306_TextBuffer, SSD1306_TextBuffer+SSD1306_WIDTH_VIRTUAL_TEXT, sizeof(SSD1306_TextBuffer)-SSD1306_WIDTH_VIRTUAL_TEXT);// Scroll
    memset(&SSD1306_TextBuffer[SSD1306_WIDTH_VIRTUAL_TEXT*(SSD1306_HEIGHT_VIRTUAL_TEXT-1)], ' ', SSD1306_WIDTH_VIRTUAL_TEXT);// Backfill spaces
    SSD1306_Update++;
  }

  if (c == '\r')
  {
    SSD1306_TextX = 0;
    return;
  }

  if (c == '\n')
  {
    SSD1306_TextX = 0; // lets infer this in case "\r\n" not paired
    SSD1306_TextY++;
    return;
  }

  SSD1306_TextBuffer[SSD1306_WIDTH_VIRTUAL_TEXT*SSD1306_TextY + SSD1306_TextX] = c;
  SSD1306_TextX++;
  SSD1306_Update++;
}

//****************************************************************************

void ssd1306_OutTextString(char *str)
{
  while(*str)
    ssd1306_OutTextChar(*str++);
  SSD1306_Update++;
}

//****************************************************************************

void OLED_Init(void)
{
  //Wire.pins(20, 21);
  Wire.begin();            // Initialize two-wire interface
  Wire.setClock(100000);  // Set I2C bus speed to 'Fast' if
                           // your Arduino supports 400KHz.

  ssd1306_Init();

#ifdef USE_OLED_TEXT
  ssd1306_InitText();
  delay(100);
  ssd1306_OutTextString("u-Blox ZED-F9P\nLoRa\nRover Two\nText Buffer\n");
  ssd1306_PaintText(); // Flush
#else
  delay(100);
  ssd1306_OutString("Lorem ipsum dolor sit amet, consectetur adipiscing elit, "
                    "sed do eiusmod tempor incididunt ut labore et dolore magna "
                    "aliqua. Ut enim ad minim veniam, quis nostrud exercitation "
                    "ullamco laboris nisi ut aliquip ex ea commodo consequat. "
                    "Duis aute irure dolor in reprehenderit in voluptate velit "
                    "esse cillum dolore eu fugiat nulla pariatur. Excepteur sint "
                    "occaecat cupidatat non proident, sunt in culpa qui officia "
                    "deserunt mollit anim id est laborum.");
  ssd1306_UpdateScreen();
#endif
}

//****************************************************************************

int ssd1306_State(void)
{
//  return(SSD1306.Initialized);

  if (SSD1306.Initialized)
    return(SSD1306.ErrorTotal);
  else
    return(-SSD1306.ErrorTotal);
}

//****************************************************************************
#endif // USE_OLED
//****************************************************************************

