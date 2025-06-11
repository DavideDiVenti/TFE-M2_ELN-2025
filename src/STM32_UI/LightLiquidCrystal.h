#ifndef LightLiquidCrystal_h
#define LightLiquidCrystal_h
#include <Arduino.h>

// Commandes LCD essentielles
#define LCD_CLEARDISPLAY 0x01
#define LCD_SETDDRAMADDR 0x80
#define LCD_DISPLAYCONTROL 0x08
#define LCD_ENTRYMODESET 0x04
#define LCD_FUNCTIONSET 0x20

// Flags minimaux nécessaires
#define LCD_DISPLAYON 0x04
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x8DOTS 0x00


class LiquidCrystal {
public:
  LiquidCrystal(uint8_t rs, uint8_t enable, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
  void begin(uint8_t cols, uint8_t rows);
  void clear();
  void command(uint8_t value);
  void setCursor(uint8_t col, uint8_t row);
  void print(const char* str);
  void print(unsigned long num); // Pour millis()/1000

private:
  void send(uint8_t value, uint8_t mode);
  void write4bits(uint8_t value);
  void pulseEnable();
  
  void write(uint8_t value);

  uint8_t _rs_pin;
  uint8_t _enable_pin;
  uint8_t _data_pins[4];
  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;
  uint8_t _numlines;
  uint8_t _row_offsets[4];
};

#endif