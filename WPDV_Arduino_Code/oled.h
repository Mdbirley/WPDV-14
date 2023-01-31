//****************************************************************************
//
//  oled.h
//
//****************************************************************************

#ifndef _OLED_H
#define _OLED_H

//****************************************************************************

// I2C address
#define SSD1306_I2C_ADDR        (0x78 >> 1) // Arduino uses least significant 7-bits

//#define USE_OLED_TEXT // Via 8x16 OLED
//#define USE_SH1106

// SSD1306 width in pixels
#define SSD1306_WIDTH           128 // SSD1306
//#define SSD1306_WIDTH           132 // SH1106

// SSD1306 LCD height in pixels
#define SSD1306_HEIGHT          64 // 128x64
//#define SSD1306_HEIGHT          32 // 128x32

//****************************************************************************
//
//  Enumeration for screen colors
//

typedef enum {
  Black = 0x00, // Black color, no pixel
  White = 0x01  //Pixel is set. Color depends on LCD
} SSD1306_COLOR;

//
//  Struct to store transformations
//
typedef struct {
  uint32_t Initialized;
	uint32_t Touched;
  uint32_t Error;
  uint32_t ErrorTotal;

  uint16_t CurrentX;
  uint16_t CurrentY;
  uint8_t Inverted;
} SSD1306_t;

//****************************************************************************

extern uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

//****************************************************************************
//
//  Function definitions
//

extern void ssd1306_Fill(SSD1306_COLOR color);
extern void ssd1306_UpdateScreen(void);
extern void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
extern void ssd1306_OutString(const char *);

#ifdef USE_OLED_TEXT
extern void ssd1306_OutTextChar(char c);
extern void ssd1306_OutTextString(char *str);
extern void ssd1306_PaintText(void);
#endif // USE_OLED_TEXT

#ifdef FONT
extern char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
extern char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);
#endif // FONT

extern void ssd1306_SetCursor(uint8_t x, uint8_t y);

extern int ssd1306_State(void);

extern void OLED_Init(void);

//****************************************************************************

#endif // _OLED_H

//****************************************************************************
