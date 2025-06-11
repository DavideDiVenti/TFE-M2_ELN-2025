#include "LightLiquidCrystal.h"

LiquidCrystal::LiquidCrystal(uint8_t rs, uint8_t enable, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  _rs_pin = rs;
  _enable_pin = enable;
  
  _data_pins[0] = d4;
  _data_pins[1] = d5;
  _data_pins[2] = d6;
  _data_pins[3] = d7;
  
  pinMode(_rs_pin, OUTPUT);
  pinMode(_enable_pin, OUTPUT);
  
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(_data_pins[i], OUTPUT);
  }

  _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
}

void LiquidCrystal::begin(uint8_t cols, uint8_t rows) {
  if (rows > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = rows;

  // Offsets pour chaque ligne
  _row_offsets[0] = 0x00;
  _row_offsets[1] = 0x40;
  _row_offsets[2] = 0x00 + cols;
  _row_offsets[3] = 0x40 + cols;
  
  // Initialisation de l'écran
  delayMicroseconds(50000);
  digitalWrite(_rs_pin, LOW);
  digitalWrite(_enable_pin, LOW);
  
  // Séquence d'initialisation pour mode 4 bits
  write4bits(0x03);
  delayMicroseconds(4500);
  write4bits(0x03);
  delayMicroseconds(4500);
  write4bits(0x03);
  delayMicroseconds(150);
  write4bits(0x02);
  
  // Configuration
  command(LCD_FUNCTIONSET | _displayfunction);
  _displaycontrol = LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
  clear();
  _displaymode = 0x02; // Entrée gauche à droite
  command(LCD_ENTRYMODESET | _displaymode);
}

void LiquidCrystal::clear() {
  command(LCD_CLEARDISPLAY);
  delayMicroseconds(2000);
}


void LiquidCrystal::setCursor(uint8_t col, uint8_t row) {
  row = row >= _numlines ? _numlines - 1 : row;
  command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}

void LiquidCrystal::print(const char* str) {
  while (*str) {
    write(*str++);
  }
}

void LiquidCrystal::print(unsigned long num) {
  // Optimisé pour les petits nombres comme millis()/1000
  char buf[10]; // Suffisant pour un compteur en secondes (millis()/1000)
  uint8_t i = 0;
  
  // Gestion du 0
  if (num == 0) {
    write('0');
    return;
  }
  
  // Conversion en chaîne (à l'envers)
  while (num > 0) {
    buf[i++] = '0' + (num % 10);
    num /= 10;
  }
  
  // Affichage des caractères dans le bon ordre
  while (i > 0) {
    write(buf[--i]);
  }
}

inline void LiquidCrystal::command(uint8_t value) {
  send(value, LOW);
}

inline void LiquidCrystal::write(uint8_t value) {
  send(value, HIGH);
}

void LiquidCrystal::send(uint8_t value, uint8_t mode) {
  digitalWrite(_rs_pin, mode);
  write4bits(value >> 4);
  write4bits(value);
}

void LiquidCrystal::pulseEnable() {
  digitalWrite(_enable_pin, LOW);
  delayMicroseconds(1);
  digitalWrite(_enable_pin, HIGH);
  delayMicroseconds(1);
  digitalWrite(_enable_pin, LOW);
  delayMicroseconds(100);
}

void LiquidCrystal::write4bits(uint8_t value) {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(_data_pins[i], (value >> i) & 0x01);
  }
  pulseEnable();
}